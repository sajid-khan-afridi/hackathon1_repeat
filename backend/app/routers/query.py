"""
Query router for RAG chatbot endpoints.
"""
import re
import logging
import asyncio
from fastapi import APIRouter, HTTPException, status, Request
from fastapi.responses import JSONResponse
from typing import Dict, Any

from app.models.query import QueryRequest, QueryResponse
from app.services.rag_service import rag_service
from app.config import settings

router = APIRouter()
logger = logging.getLogger(__name__)

# HTML/XSS sanitization pattern
HTML_TAG_PATTERN = re.compile(r"<[^>]+>")


def sanitize_input(text: str) -> str:
    """
    Sanitize user input to prevent XSS attacks.

    Removes HTML tags and suspicious characters while preserving
    legitimate text and special characters used in technical questions.

    Args:
        text: Raw user input

    Returns:
        Sanitized text with HTML tags removed
    """
    # Remove HTML tags
    sanitized = HTML_TAG_PATTERN.sub("", text)

    # Remove null bytes and control characters (except newlines and tabs)
    sanitized = "".join(
        char for char in sanitized if char.isprintable() or char in "\n\t"
    )

    return sanitized.strip()


@router.post("/query", response_model=QueryResponse, status_code=status.HTTP_200_OK)
async def query_endpoint(request: QueryRequest) -> QueryResponse:
    """
    Process a user query through the RAG pipeline.

    This endpoint:
    1. Validates and sanitizes the input query
    2. Retrieves relevant context from the vector database
    3. Generates an AI-powered answer using the LLM
    4. Saves the conversation to chat history
    5. Returns the answer with source citations and confidence score

    Args:
        request: QueryRequest with query, filters, session_id, etc.

    Returns:
        QueryResponse with answer, sources, confidence, session_id, tokens_used

    Raises:
        HTTPException 400: For validation errors (empty query, invalid filters)
        HTTPException 503: For service unavailability (LLM timeout, vector store down)
        HTTPException 500: For unexpected server errors

    Example request:
    ```json
    {
        "query": "What is inverse kinematics?",
        "filters": {
            "module": 3,
            "difficulty": "intermediate"
        },
        "top_k": 5
    }
    ```

    Example response:
    ```json
    {
        "answer": "Inverse kinematics (IK) is the process of determining joint angles...",
        "sources": [
            {
                "chapter_id": "module-3-chapter-1",
                "chapter_title": "Inverse Kinematics",
                "relevance_score": 0.92,
                "excerpt": "Inverse kinematics is covered in detail...",
                "position": 1
            }
        ],
        "confidence": 0.89,
        "session_id": "123e4567-e89b-12d3-a456-426614174000",
        "tokens_used": {
            "input_tokens": 245,
            "output_tokens": 127,
            "total_tokens": 372
        }
    }
    ```
    """
    logger.info(f"Received query request: '{request.query[:50]}...'")

    # Input sanitization - prevent XSS attacks
    try:
        original_query = request.query
        request.query = sanitize_input(request.query)

        if not request.query:
            logger.warning("Query became empty after sanitization")
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Query is empty or contains only invalid characters",
            )

        if original_query != request.query:
            logger.warning(
                f"Query was sanitized: '{original_query[:30]}...' -> '{request.query[:30]}...'"
            )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Input sanitization failed: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid input: {str(e)}",
        )

    # Process query with timeout (30 seconds for LLM operations)
    try:
        response = await asyncio.wait_for(
            rag_service.process_query(request), timeout=30.0
        )

        logger.info(
            f"Query processed successfully (confidence={response.confidence}, "
            f"sources={len(response.sources)}, tokens={response.tokens_used.total_tokens})"
        )

        return response

    except asyncio.TimeoutError:
        logger.error("Query processing timed out after 30 seconds")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Request timeout - the query took too long to process. Please try again.",
        )

    except HTTPException:
        # Re-raise HTTP exceptions (already have proper status codes)
        raise

    except Exception as e:
        error_msg = str(e).lower()

        # Detect specific error types and return appropriate status codes

        # Vector store unavailable
        if "vector" in error_msg or "qdrant" in error_msg:
            logger.error(f"Vector store error: {str(e)}", exc_info=True)
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="Vector search service is temporarily unavailable. Please try again later.",
            )

        # LLM service unavailable
        if "openai" in error_msg or "llm" in error_msg or "embedding" in error_msg:
            logger.error(f"LLM service error: {str(e)}", exc_info=True)
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="AI service is temporarily unavailable. Please try again later.",
            )

        # Database errors
        if "database" in error_msg or "postgres" in error_msg or "asyncpg" in error_msg:
            logger.error(f"Database error: {str(e)}", exc_info=True)
            # Database errors are non-critical (conversation still works)
            # Continue processing but log the error
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="An error occurred while saving conversation history.",
            )

        # Generic server error
        logger.error(f"Unexpected error during query processing: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An unexpected error occurred. Please try again later.",
        )


@router.middleware("http")
async def add_rate_limit_headers(request: Request, call_next):
    """
    Add rate limit headers to responses (placeholder for future implementation).

    Headers added:
    - X-RateLimit-Limit: Maximum requests allowed per window
    - X-RateLimit-Remaining: Requests remaining in current window
    - X-RateLimit-Reset: Unix timestamp when the limit resets

    Note: Actual rate limiting logic is not implemented yet.
    This is a placeholder that returns static values.
    """
    response = await call_next(request)

    # Placeholder rate limit headers
    # TODO: Implement actual rate limiting with Redis or in-memory cache
    response.headers["X-RateLimit-Limit"] = str(settings.rate_limit_anonymous)
    response.headers["X-RateLimit-Remaining"] = str(settings.rate_limit_anonymous)
    response.headers["X-RateLimit-Reset"] = "0"

    return response


@router.get("/query/health")
async def query_health() -> Dict[str, Any]:
    """
    Health check endpoint for query service.

    Returns:
        Status information for RAG pipeline components
    """
    from app.services.vector_service import vector_service
    from app.services.llm_service import llm_service
    from app.services.chat_service import chat_service

    health_status = {
        "status": "healthy",
        "components": {
            "vector_search": "unknown",
            "llm": "unknown",
            "database": "unknown",
        },
    }

    # Check vector service
    try:
        vector_healthy = await vector_service.health_check()
        health_status["components"]["vector_search"] = (
            "healthy" if vector_healthy else "unhealthy"
        )
    except Exception as e:
        logger.error(f"Vector service health check failed: {str(e)}")
        health_status["components"]["vector_search"] = "unhealthy"

    # Check LLM service
    try:
        llm_healthy = await llm_service.health_check()
        health_status["components"]["llm"] = "healthy" if llm_healthy else "unhealthy"
    except Exception as e:
        logger.error(f"LLM service health check failed: {str(e)}")
        health_status["components"]["llm"] = "unhealthy"

    # Check chat service
    try:
        chat_healthy = await chat_service.health_check()
        health_status["components"]["database"] = (
            "healthy" if chat_healthy else "unhealthy"
        )
    except Exception as e:
        logger.error(f"Chat service health check failed: {str(e)}")
        health_status["components"]["database"] = "unhealthy"

    # Determine overall status
    all_healthy = all(
        status == "healthy" for status in health_status["components"].values()
    )
    health_status["status"] = "healthy" if all_healthy else "degraded"

    return health_status
