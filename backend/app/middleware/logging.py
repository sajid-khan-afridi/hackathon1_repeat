"""
Logging middleware with correlation ID tracking and structured JSON logging.

This middleware:
- Generates/extracts correlation IDs for request tracing
- Logs structured JSON for easy parsing by log aggregators
- Tracks response time metrics (p50, p95, p99)
- Records request/response metadata
"""

import uuid
import time
import logging
import json
from collections import deque
from typing import Callable, Dict, Any, Deque
from fastapi import Request, Response
from starlette.middleware.base import BaseHTTPMiddleware

logger = logging.getLogger(__name__)

# Metrics storage (in-memory circular buffer)
# In production, use a proper metrics backend (Prometheus, StatsD, etc.)
_response_times: Deque[float] = deque(maxlen=1000)
_endpoint_metrics: Dict[str, Deque[float]] = {}


class StructuredFormatter(logging.Formatter):
    """
    Custom JSON formatter for structured logging.

    Outputs logs as JSON objects for easy parsing by log aggregation systems
    (e.g., CloudWatch, Datadog, ELK stack).
    """

    def format(self, record: logging.LogRecord) -> str:
        """Format log record as JSON."""
        log_data = {
            "timestamp": self.formatTime(record),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
        }

        # Add extra fields if present
        if hasattr(record, "correlation_id"):
            log_data["correlation_id"] = record.correlation_id
        if hasattr(record, "method"):
            log_data["method"] = record.method
        if hasattr(record, "path"):
            log_data["path"] = record.path
        if hasattr(record, "status_code"):
            log_data["status_code"] = record.status_code
        if hasattr(record, "duration_ms"):
            log_data["duration_ms"] = record.duration_ms
        if hasattr(record, "client_host"):
            log_data["client_host"] = record.client_host
        if hasattr(record, "user_id"):
            log_data["user_id"] = record.user_id

        # Add exception info if present
        if record.exc_info:
            log_data["exception"] = self.formatException(record.exc_info)

        return json.dumps(log_data)


def configure_structured_logging():
    """
    Configure structured JSON logging for the application.

    Call this during application startup.
    """
    # Create JSON formatter
    formatter = StructuredFormatter()

    # Configure root logger
    handler = logging.StreamHandler()
    handler.setFormatter(formatter)

    root_logger = logging.getLogger()
    root_logger.handlers = [handler]
    root_logger.setLevel(logging.INFO)


def calculate_percentile(values: Deque[float], percentile: float) -> float:
    """
    Calculate percentile from a collection of values.

    Args:
        values: Collection of numeric values
        percentile: Percentile to calculate (0-100)

    Returns:
        Percentile value
    """
    if not values:
        return 0.0

    sorted_values = sorted(values)
    index = int(len(sorted_values) * (percentile / 100))
    return sorted_values[min(index, len(sorted_values) - 1)]


def get_metrics_summary() -> Dict[str, Any]:
    """
    Get summary of response time metrics.

    Returns:
        Dict with p50, p95, p99 response times in milliseconds
    """
    if not _response_times:
        return {
            "total_requests": 0,
            "p50_ms": 0.0,
            "p95_ms": 0.0,
            "p99_ms": 0.0,
            "endpoint_metrics": {},
        }

    return {
        "total_requests": len(_response_times),
        "p50_ms": round(calculate_percentile(_response_times, 50), 2),
        "p95_ms": round(calculate_percentile(_response_times, 95), 2),
        "p99_ms": round(calculate_percentile(_response_times, 99), 2),
        "endpoint_metrics": {
            endpoint: {
                "count": len(times),
                "p50_ms": round(calculate_percentile(times, 50), 2),
                "p95_ms": round(calculate_percentile(times, 95), 2),
                "p99_ms": round(calculate_percentile(times, 99), 2),
            }
            for endpoint, times in _endpoint_metrics.items()
        },
    }


class LoggingMiddleware(BaseHTTPMiddleware):
    """
    Middleware for request/response logging with correlation IDs.

    Features:
    - Correlation ID tracking for request tracing
    - Structured JSON logging
    - Response time metrics collection (p50, p95, p99)
    - Request/response metadata logging
    """

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """Process request with correlation ID tracking and metrics."""
        # Generate or extract correlation ID
        correlation_id = request.headers.get("X-Correlation-ID", str(uuid.uuid4()))
        request.state.correlation_id = correlation_id

        # Extract user ID if authenticated (for future use)
        user_id = request.headers.get("X-User-ID")

        # Log request start
        start_time = time.time()
        logger.info(
            "Request started",
            extra={
                "correlation_id": correlation_id,
                "method": request.method,
                "path": request.url.path,
                "client_host": request.client.host if request.client else None,
                "user_id": user_id,
            },
        )

        # Process request
        try:
            response = await call_next(request)
        except Exception as e:
            # Log exception
            duration = time.time() - start_time
            logger.error(
                f"Request failed: {str(e)}",
                exc_info=True,
                extra={
                    "correlation_id": correlation_id,
                    "method": request.method,
                    "path": request.url.path,
                    "duration_ms": round(duration * 1000, 2),
                },
            )
            raise

        # Calculate duration in milliseconds
        duration_ms = (time.time() - start_time) * 1000

        # Store metrics
        _response_times.append(duration_ms)

        # Store per-endpoint metrics
        endpoint_key = f"{request.method} {request.url.path}"
        if endpoint_key not in _endpoint_metrics:
            _endpoint_metrics[endpoint_key] = deque(maxlen=1000)
        _endpoint_metrics[endpoint_key].append(duration_ms)

        # Log response
        logger.info(
            "Request completed",
            extra={
                "correlation_id": correlation_id,
                "method": request.method,
                "path": request.url.path,
                "status_code": response.status_code,
                "duration_ms": round(duration_ms, 2),
                "user_id": user_id,
            },
        )

        # Add correlation ID to response headers
        response.headers["X-Correlation-ID"] = correlation_id

        return response
