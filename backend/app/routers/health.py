"""
Health check router for monitoring service status.
"""

import asyncio
from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel
from typing import Dict, Any
from app.config import settings
import asyncpg
from qdrant_client import QdrantClient
from openai import AsyncOpenAI

router = APIRouter()


class HealthStatus(BaseModel):
    """Health check response model."""

    status: str
    database: str
    vector_store: str
    llm: str
    details: Dict[str, Any]


@router.get("/health", response_model=HealthStatus)
async def health_check() -> HealthStatus:
    """
    Check health of all external dependencies.

    Returns:
        HealthStatus with status of database, vector store, and LLM.

    Raises:
        HTTPException: If any critical service is unavailable.
    """
    results = {
        "database": "unknown",
        "vector_store": "unknown",
        "llm": "unknown",
        "details": {},
    }

    # Check PostgreSQL database
    try:
        conn = await asyncpg.connect(settings.database_url, timeout=5.0)
        await conn.fetchval("SELECT 1")
        await conn.close()
        results["database"] = "healthy"
        results["details"]["database"] = "Connected successfully"
    except Exception as e:
        results["database"] = "unhealthy"
        results["details"]["database"] = f"Connection failed: {str(e)}"

    # Check Qdrant vector store
    try:
        qdrant = QdrantClient(url=settings.qdrant_url, api_key=settings.qdrant_api_key)
        collections = qdrant.get_collections()
        # Check if our collection exists
        collection_exists = any(
            c.name == settings.qdrant_collection for c in collections.collections
        )
        if collection_exists:
            # Get collection info including vector count
            collection_info = qdrant.get_collection(settings.qdrant_collection)
            vector_count = collection_info.points_count
            if vector_count > 0:
                results["vector_store"] = "healthy"
                results["details"]["vector_store"] = f"Collection '{settings.qdrant_collection}' has {vector_count} vectors"
            else:
                results["vector_store"] = "degraded"
                results["details"]["vector_store"] = f"Collection '{settings.qdrant_collection}' exists but has 0 vectors - content not indexed!"
        else:
            results["vector_store"] = "degraded"
            results["details"][
                "vector_store"
            ] = f"Collection '{settings.qdrant_collection}' not found - run indexing script"
    except Exception as e:
        results["vector_store"] = "unhealthy"
        results["details"]["vector_store"] = f"Connection failed: {str(e)}"

    # Check OpenAI API
    try:
        client = AsyncOpenAI(api_key=settings.openai_api_key)
        # Simple API test - list models
        await client.models.list()
        results["llm"] = "healthy"
        results["details"]["llm"] = "OpenAI API accessible"
    except Exception as e:
        results["llm"] = "unhealthy"
        results["details"]["llm"] = f"API call failed: {str(e)}"

    # Determine overall status
    if all(v == "healthy" for v in [results["database"], results["vector_store"], results["llm"]]):
        overall_status = "healthy"
    elif results["database"] == "unhealthy" or results["llm"] == "unhealthy":
        overall_status = "unhealthy"
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Critical services unavailable",
        )
    else:
        overall_status = "degraded"

    return HealthStatus(status=overall_status, **results)
