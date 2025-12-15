"""Health check endpoints."""

import asyncio
import logging
from datetime import datetime
from typing import Any, Dict

from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import JSONResponse

from app.config import get_settings
from app.models import HealthCheck, ErrorResponse
from app.middleware.logging import get_metrics, get_metrics_percentiles, get_endpoint_metrics

logger = logging.getLogger(__name__)
router = APIRouter()


@router.get("/health", response_model=HealthCheck)
async def health_check() -> HealthCheck:
    """Comprehensive health check of all services."""
    settings = get_settings()

    # Check all services in parallel
    services_status = await check_all_services()

    # Determine overall status
    all_healthy = all(status == "healthy" for status in services_status.values())
    overall_status = "healthy" if all_healthy else "degraded"

    return HealthCheck(
        status=overall_status,
        timestamp=datetime.utcnow(),
        services=services_status,
        version="0.1.0",
    )


@router.get("/health/ready")
async def readiness_check() -> JSONResponse:
    """Readiness probe for Kubernetes/container orchestration."""
    services_status = await check_all_services()

    # Check critical services for readiness
    critical_services = ["api", "database"]
    all_ready = all(services_status.get(service) == "healthy" for service in critical_services)

    if all_ready:
        return JSONResponse(status_code=200, content={"status": "ready"})

    return JSONResponse(
        status_code=503, content={"status": "not ready", "services": services_status}
    )


@router.get("/health/live")
async def liveness_check() -> JSONResponse:
    """Liveness probe for Kubernetes/container orchestration."""
    # Simple liveness check - if we can respond, we're live
    return JSONResponse(status_code=200, content={"status": "live"})


async def check_all_services() -> Dict[str, str]:
    """Check the health of all external services."""
    settings = get_settings()

    # Create tasks for all service checks
    tasks = {
        "api": check_api_health(),
        "database": check_database_health(settings.database_url),
        "qdrant": check_qdrant_health(settings.qdrant_url, settings.qdrant_api_key),
        "openai": check_openai_health(settings.openai_api_key),
    }

    # Execute all checks concurrently
    results = await asyncio.gather(*tasks.values(), return_exceptions=True)

    # Map results back to service names
    services_status = {}
    for i, (service, _) in enumerate(tasks.items()):
        result = results[i]
        if isinstance(result, Exception):
            logger.error(f"Health check failed for {service}: {result}")
            services_status[service] = "unhealthy"
        else:
            services_status[service] = result

    return services_status


async def check_api_health() -> str:
    """Check API health (always healthy if we're running)."""
    return "healthy"


async def check_database_health(database_url: str) -> str:
    """Check PostgreSQL database connectivity."""
    try:
        import asyncpg

        # Attempt to connect and run a simple query
        conn = await asyncpg.connect(database_url, timeout=5.0)

        # Test query
        await conn.fetchval("SELECT 1")

        # Close connection
        await conn.close()

        logger.debug("Database health check passed")
        return "healthy"

    except Exception as e:
        logger.warning(f"Database health check failed: {e}")
        return "unhealthy"


async def check_qdrant_health(qdrant_url: str, qdrant_api_key: str) -> str:
    """Check Qdrant vector database connectivity."""
    try:
        from qdrant_client import QdrantClient
        from qdrant_client.http.exceptions import UnexpectedResponse

        # Create client
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key, timeout=5.0)

        # Check health by getting collections
        collections = client.get_collections()

        # Verify our collection exists (or doesn't, that's ok too)
        collection_names = [c.name for c in collections.collections]

        logger.debug(f"Qdrant health check passed. Collections: {collection_names}")
        return "healthy"

    except UnexpectedResponse as e:
        logger.warning(f"Qdrant health check failed: {e}")
        return "unhealthy"
    except Exception as e:
        logger.warning(f"Qdrant health check error: {e}")
        return "unknown"


async def check_openai_health(openai_api_key: str) -> str:
    """Check OpenAI API connectivity."""
    try:
        from openai import OpenAI

        settings = get_settings()

        # Create client
        client = OpenAI(api_key=openai_api_key)

        # Simple API call - list models
        models = client.models.list()

        # Check if we have access to required models
        model_ids = [m.id for m in models.data]
        required_models = [settings.rag_embedding_model, settings.rag_llm_model]

        missing_models = [m for m in required_models if m not in model_ids]
        if missing_models:
            logger.warning(f"Missing required OpenAI models: {missing_models}")
            return "degraded"

        logger.debug("OpenAI health check passed")
        return "healthy"

    except Exception as e:
        logger.warning(f"OpenAI health check failed: {e}")
        return "unhealthy"


@router.get("/metrics")
async def metrics_endpoint() -> Dict[str, Any]:
    """Get application performance metrics.

    Returns response time percentiles (p50, p95, p99) and request statistics.

    Returns:
        Dictionary containing:
        - total_requests: Total number of requests processed
        - error_count: Number of requests that resulted in errors
        - error_rate: Percentage of requests that failed
        - percentiles: Response time percentiles in milliseconds
        - mean_ms: Average response time
        - min_ms: Minimum response time
        - max_ms: Maximum response time
    """
    return get_metrics()


@router.get("/metrics/percentiles")
async def percentiles_endpoint() -> Dict[str, float]:
    """Get response time percentiles only.

    Returns:
        Dictionary with p50, p95, p99 values in milliseconds
    """
    return get_metrics_percentiles()


@router.get("/metrics/endpoint/{path:path}")
async def endpoint_metrics(path: str) -> Dict[str, float]:
    """Get metrics for a specific API endpoint.

    Args:
        path: The endpoint path (e.g., "api/v1/query")

    Returns:
        Dictionary with endpoint-specific metrics
    """
    # Ensure path starts with /
    if not path.startswith("/"):
        path = "/" + path

    metrics = get_endpoint_metrics(path)
    if not metrics:
        return {"message": f"No metrics available for {path}"}
    return metrics
