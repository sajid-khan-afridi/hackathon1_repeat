"""Logging middleware with correlation ID tracking and metrics collection.

This module provides:
- Correlation ID tracking for request tracing
- Structured JSON logging for production
- Response time metrics collection (p50/p95/p99)
"""

import json
import logging
import statistics
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from threading import Lock
from typing import Any, Callable

from fastapi import Request, Response
from starlette.middleware.base import BaseHTTPMiddleware

from app.config import get_settings

# Configure structured logging
logger = logging.getLogger("rag_chatbot")


@dataclass
class MetricsCollector:
    """Collects and calculates response time metrics.

    Maintains a sliding window of response times for percentile calculations.
    Thread-safe for concurrent access.
    """

    window_size: int = 1000
    _response_times: deque = field(default_factory=lambda: deque(maxlen=1000))
    _lock: Lock = field(default_factory=Lock)
    _total_requests: int = 0
    _error_count: int = 0
    _endpoint_times: dict = field(default_factory=dict)

    def record_response_time(
        self, duration_ms: float, endpoint: str = "", status_code: int = 200
    ) -> None:
        """Record a response time measurement.

        Args:
            duration_ms: Response time in milliseconds
            endpoint: The API endpoint path
            status_code: HTTP status code
        """
        with self._lock:
            self._response_times.append(duration_ms)
            self._total_requests += 1

            if status_code >= 400:
                self._error_count += 1

            # Track per-endpoint metrics
            if endpoint:
                if endpoint not in self._endpoint_times:
                    self._endpoint_times[endpoint] = deque(maxlen=100)
                self._endpoint_times[endpoint].append(duration_ms)

    def get_percentiles(self) -> dict[str, float]:
        """Calculate p50, p95, p99 percentiles.

        Returns:
            Dictionary with percentile values in milliseconds
        """
        with self._lock:
            if not self._response_times:
                return {"p50": 0.0, "p95": 0.0, "p99": 0.0}

            sorted_times = sorted(self._response_times)
            n = len(sorted_times)

            return {
                "p50": sorted_times[int(n * 0.50)] if n > 0 else 0.0,
                "p95": sorted_times[int(n * 0.95)] if n > 0 else 0.0,
                "p99": sorted_times[int(n * 0.99)] if n > 0 else 0.0,
            }

    def get_stats(self) -> dict[str, Any]:
        """Get comprehensive metrics statistics.

        Returns:
            Dictionary with all metrics
        """
        with self._lock:
            percentiles = self.get_percentiles()

            stats = {
                "total_requests": self._total_requests,
                "error_count": self._error_count,
                "error_rate": (
                    self._error_count / self._total_requests if self._total_requests > 0 else 0.0
                ),
                "percentiles": percentiles,
            }

            if self._response_times:
                times_list = list(self._response_times)
                stats["mean_ms"] = statistics.mean(times_list)
                stats["min_ms"] = min(times_list)
                stats["max_ms"] = max(times_list)
                if len(times_list) > 1:
                    stats["stddev_ms"] = statistics.stdev(times_list)

            return stats

    def get_endpoint_stats(self, endpoint: str) -> dict[str, float]:
        """Get metrics for a specific endpoint.

        Args:
            endpoint: The API endpoint path

        Returns:
            Dictionary with endpoint-specific metrics
        """
        with self._lock:
            if endpoint not in self._endpoint_times or not self._endpoint_times[endpoint]:
                return {}

            times = list(self._endpoint_times[endpoint])
            sorted_times = sorted(times)
            n = len(sorted_times)

            return {
                "count": n,
                "mean_ms": statistics.mean(times),
                "p50": sorted_times[int(n * 0.50)] if n > 0 else 0.0,
                "p95": sorted_times[int(n * 0.95)] if n > 0 else 0.0,
            }


# Global metrics collector instance
metrics_collector = MetricsCollector()


class LoggingMiddleware(BaseHTTPMiddleware):
    """Middleware to add correlation IDs and structured logging."""

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """Process request and add logging."""
        # Generate correlation ID
        correlation_id = str(uuid.uuid4())

        # Add to request state for access in endpoints
        request.state.correlation_id = correlation_id

        # Get client IP
        forwarded_for = request.headers.get("x-forwarded-for")
        if forwarded_for:
            client_ip = forwarded_for.split(",")[0].strip()
        else:
            client_ip = request.client.host if request.client else "unknown"

        # Log request start
        start_time = time.time()
        logger.info(
            "Request started",
            extra={
                "event": "request_start",
                "correlation_id": correlation_id,
                "method": request.method,
                "url": str(request.url),
                "client_ip": client_ip,
                "user_agent": request.headers.get("user-agent", "unknown"),
            },
        )

        # Process request
        try:
            response = await call_next(request)
        except Exception as e:
            # Log exception
            logger.error(
                "Request failed with exception",
                extra={
                    "event": "request_error",
                    "correlation_id": correlation_id,
                    "method": request.method,
                    "url": str(request.url),
                    "client_ip": client_ip,
                    "error": str(e),
                    "error_type": type(e).__name__,
                },
                exc_info=True if get_settings().is_development else False,
            )
            raise

        # Calculate duration
        duration = time.time() - start_time
        duration_ms = round(duration * 1000, 2)

        # Record metrics
        endpoint = request.url.path
        metrics_collector.record_response_time(duration_ms, endpoint, response.status_code)

        # Log request completion
        logger.info(
            "Request completed",
            extra={
                "event": "request_end",
                "correlation_id": correlation_id,
                "method": request.method,
                "url": str(request.url),
                "path": endpoint,
                "status_code": response.status_code,
                "duration_ms": duration_ms,
                "client_ip": client_ip,
            },
        )

        # Add correlation ID to response headers
        response.headers["X-Correlation-ID"] = correlation_id

        return response


def configure_logging() -> None:
    """Configure structured logging for the application."""
    settings = get_settings()

    # Configure root logger
    logging.basicConfig(
        level=getattr(logging, settings.log_level.upper()),
        format="%(message)s",
    )

    # Create structured logger
    structured_logger = logging.getLogger("rag_chatbot")
    structured_logger.setLevel(getattr(logging, settings.log_level.upper()))

    # Remove default handlers
    for handler in structured_logger.handlers[:]:
        structured_logger.removeHandler(handler)

    # Add console handler with structured formatter
    if settings.is_development:
        handler = logging.StreamHandler()
        handler.setFormatter(DevelopmentFormatter())
    else:
        handler = logging.StreamHandler()
        handler.setFormatter(ProductionFormatter())

    structured_logger.addHandler(handler)


class DevelopmentFormatter(logging.Formatter):
    """Formatter for development environment with detailed info."""

    def format(self, record: logging.LogRecord) -> str:
        """Format log record for development."""
        # Get structured data from extra
        extra_data = {}
        for key, value in record.__dict__.items():
            if key not in {
                "name",
                "msg",
                "args",
                "levelname",
                "levelno",
                "pathname",
                "filename",
                "module",
                "lineno",
                "funcName",
                "created",
                "msecs",
                "relativeCreated",
                "thread",
                "threadName",
                "processName",
                "process",
                "getMessage",
                "exc_info",
                "exc_text",
                "stack_info",
            }:
                extra_data[key] = value

        # Build structured message
        if extra_data:
            # Create a structured log message
            parts = []
            for key, value in extra_data.items():
                if key == "event":
                    parts.append(f"[{value.upper()}]")
                else:
                    parts.append(f"{key}={value}")

            structured_part = " | ".join(parts)
            return f"{record.getMessage()} | {structured_part}"
        else:
            return record.getMessage()


class ProductionFormatter(logging.Formatter):
    """Formatter for production environment with JSON output.

    Outputs structured JSON logs suitable for log aggregation systems
    like ELK, Datadog, or CloudWatch.
    """

    # Standard fields to exclude from extra data
    RESERVED_ATTRS = {
        "name",
        "msg",
        "args",
        "levelname",
        "levelno",
        "pathname",
        "filename",
        "module",
        "lineno",
        "funcName",
        "created",
        "msecs",
        "relativeCreated",
        "thread",
        "threadName",
        "processName",
        "process",
        "getMessage",
        "exc_info",
        "exc_text",
        "stack_info",
        "timestamp",
        "message",
        "asctime",
    }

    def format(self, record: logging.LogRecord) -> str:
        """Format log record as JSON.

        Output format:
        {
            "timestamp": "2025-01-01T12:00:00.000Z",
            "level": "INFO",
            "logger": "rag_chatbot",
            "message": "Request completed",
            "correlation_id": "uuid",
            "event": "request_end",
            ...additional fields
        }
        """
        # Create ISO 8601 timestamp
        timestamp = datetime.utcfromtimestamp(record.created).isoformat() + "Z"

        # Create base log entry
        log_entry: dict[str, Any] = {
            "timestamp": timestamp,
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
        }

        # Add location info for errors
        if record.levelno >= logging.ERROR:
            log_entry["location"] = {
                "file": record.filename,
                "line": record.lineno,
                "function": record.funcName,
            }

        # Add structured data from extra
        for key, value in record.__dict__.items():
            if key not in self.RESERVED_ATTRS:
                # Ensure value is JSON serializable
                try:
                    json.dumps(value)
                    log_entry[key] = value
                except (TypeError, ValueError):
                    log_entry[key] = str(value)

        # Add exception if present
        if record.exc_info:
            log_entry["exception"] = {
                "type": record.exc_info[0].__name__ if record.exc_info[0] else None,
                "message": str(record.exc_info[1]) if record.exc_info[1] else None,
                "traceback": self.formatException(record.exc_info),
            }

        return json.dumps(log_entry, default=str)


def get_metrics() -> dict[str, Any]:
    """Get current application metrics.

    Returns:
        Dictionary containing response time metrics and statistics
    """
    return metrics_collector.get_stats()


def get_metrics_percentiles() -> dict[str, float]:
    """Get response time percentiles (p50, p95, p99).

    Returns:
        Dictionary with percentile values in milliseconds
    """
    return metrics_collector.get_percentiles()


def get_endpoint_metrics(endpoint: str) -> dict[str, float]:
    """Get metrics for a specific endpoint.

    Args:
        endpoint: The API endpoint path (e.g., "/api/v1/query")

    Returns:
        Dictionary with endpoint-specific metrics
    """
    return metrics_collector.get_endpoint_stats(endpoint)
