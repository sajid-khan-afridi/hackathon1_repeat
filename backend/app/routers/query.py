"""
Query router for RAG chatbot endpoints.
"""

import re
import logging
import asyncio
import json
from datetime import datetime, timedelta, timezone
from collections import defaultdict
from typing import Dict, Any, AsyncGenerator, Tuple
from fastapi import APIRouter, HTTPException, status, Request
from fastapi.responses import JSONResponse, StreamingResponse

from app.models.query import QueryRequest, QueryResponse, StreamChunk
from app.services.rag_service import rag_service
from app.config import settings

router = APIRouter()
logger = logging.getLogger(__name__)

# ============================================
# Rate Limiting
# ============================================

# In-memory rate limit store: {identifier: [(timestamp, count), ...]}
rate_limit_store: Dict[str, list[Tuple[datetime, int]]] = defaultdict(list)


def get_rate_limit_info(request: Request) -> Tuple[str, int, int, int]:
    """
    Get rate limit information for the current request.

    Args:
        request: FastAPI Request object

    Returns:
        Tuple of (identifier, limit, remaining, reset_timestamp)
    """
    # Get user_id from request.state (set by UserIdentificationMiddleware)
    user_id = getattr(request.state, "user_id", None)

    # Determine identifier and limit
    if user_id:
        # Authenticated user
        identifier = f"user:{user_id}"
        limit = settings.rate_limit_authenticated  # 50/hr
    else:
        # Anonymous user - use IP address
        client_ip = request.client.host if request.client else "unknown"
        identifier = f"ip:{client_ip}"
        limit = settings.rate_limit_anonymous  # 10/hr

    # Clean up old entries (older than 1 hour)
    now = datetime.now(timezone.utc)
    cutoff = now - timedelta(hours=1)
    rate_limit_store[identifier] = [
        (ts, count) for ts, count in rate_limit_store[identifier] if ts > cutoff
    ]

    # Count requests in the last hour
    used = sum(count for ts, count in rate_limit_store[identifier])
    remaining = max(0, limit - used)

    # Reset time is 1 hour from the oldest request (or now if no requests)
    if rate_limit_store[identifier]:
        oldest_timestamp = min(ts for ts, _ in rate_limit_store[identifier])
        reset_timestamp = int((oldest_timestamp + timedelta(hours=1)).timestamp())
    else:
        reset_timestamp = int((now + timedelta(hours=1)).timestamp())

    return identifier, limit, remaining, reset_timestamp


def check_rate_limit(request: Request) -> None:
    """
    Check if the request exceeds rate limits.

    Raises:
        HTTPException 429: If rate limit is exceeded
    """
    identifier, limit, remaining, reset_timestamp = get_rate_limit_info(request)

    if remaining <= 0:
        user_id = getattr(request.state, "user_id", None)
        logger.warning(
            f"Rate limit exceeded for {'user ' + str(user_id) if user_id else 'IP ' + identifier.split(':')[1]}"
        )
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail={
                "message": "Rate limit exceeded. Please try again later or sign up for higher limits.",
                "limit": limit,
                "remaining": 0,
                "reset": reset_timestamp,
                "authenticated": user_id is not None,
            },
        )

    # Record this request
    now = datetime.now(timezone.utc)
    rate_limit_store[identifier].append((now, 1))


def add_rate_limit_headers(response: JSONResponse | StreamingResponse, request: Request) -> None:
    """
    Add rate limit headers to response.

    Args:
        response: Response object
        request: Request object
    """
    _, limit, remaining, reset_timestamp = get_rate_limit_info(request)

    response.headers["X-RateLimit-Limit"] = str(limit)
    response.headers["X-RateLimit-Remaining"] = str(remaining)
    response.headers["X-RateLimit-Reset"] = str(reset_timestamp)

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
    sanitized = "".join(char for char in sanitized if char.isprintable() or char in "\n\t")

    return sanitized.strip()


async def generate_sse_stream(query_request: QueryRequest) -> AsyncGenerator[str, None]:
    """
    Generate Server-Sent Events stream for query response.

    Args:
        query_request: QueryRequest to process

    Yields:
        SSE-formatted strings (data: {json}\n\n)
    """
    try:
        async for chunk in rag_service.process_query_stream(query_request):
            # Convert StreamChunk to JSON and format as SSE
            chunk_dict = chunk.model_dump(exclude_none=True)
            sse_data = f"data: {json.dumps(chunk_dict)}\n\n"
            yield sse_data

            # If streaming is done, close the connection
            if chunk.done:
                break

    except Exception as e:
        logger.error(f"Streaming error: {str(e)}", exc_info=True)
        # Send error chunk
        error_chunk = StreamChunk(
            chunk=f"Error: {str(e)}",
            done=True,
            sources=[],
            confidence=0.0,
        )
        yield f"data: {json.dumps(error_chunk.model_dump(exclude_none=True))}\n\n"


@router.post("/query", status_code=status.HTTP_200_OK)
async def query_endpoint(request: Request, query_request: QueryRequest):
    """
    Process a user query through the RAG pipeline with auth-aware rate limiting.

    Rate Limits:
    - Anonymous users: 10 queries/hour
    - Authenticated users: 50 queries/hour

    This endpoint supports both streaming and non-streaming modes:
    - Streaming: Include "Accept: text/event-stream" header to receive SSE stream
    - Non-streaming: Default JSON response with complete answer

    This endpoint:
    1. Checks rate limits (raises 429 if exceeded)
    2. Validates and sanitizes the input query
    3. Retrieves relevant context from the vector database
    4. Generates an AI-powered answer using the LLM (streaming or non-streaming)
    5. Saves the conversation to chat history
    6. Returns the answer with source citations, confidence score, and rate limit headers

    Args:
        request: FastAPI Request object
        query_request: QueryRequest with query, filters, session_id, etc.

    Returns:
        - StreamingResponse with SSE events if Accept: text/event-stream
        - QueryResponse JSON if standard request

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

    Example streaming response (SSE):
    ```
    data: {"chunk": "Inverse", "done": false}

    data: {"chunk": " kinematics", "done": false}

    data: {"chunk": "", "done": true, "sources": [...], "confidence": 0.89, "session_id": "..."}
    ```

    Example non-streaming response (JSON):
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
    # Check rate limits first (before processing)
    check_rate_limit(request)

    # Link query to authenticated user (if logged in)
    user_id = getattr(request.state, "user_id", None)
    if user_id and not query_request.user_id:
        # Set user_id from middleware (cookie-based auth)
        query_request.user_id = user_id
        logger.debug(f"Linked query to authenticated user: {user_id}")

    logger.info(f"Received query request: '{query_request.query[:50]}...'")

    # Check if client wants streaming response
    accept_header = request.headers.get("accept", "")
    wants_streaming = "text/event-stream" in accept_header

    # Input sanitization - prevent XSS attacks
    try:
        original_query = query_request.query
        query_request.query = sanitize_input(query_request.query)

        if not query_request.query:
            logger.warning("Query became empty after sanitization")
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Query is empty or contains only invalid characters",
            )

        if original_query != query_request.query:
            logger.warning(
                f"Query was sanitized: '{original_query[:30]}...' -> '{query_request.query[:30]}...'"
            )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Input sanitization failed: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid input: {str(e)}",
        )

    # Return streaming response if requested
    if wants_streaming:
        logger.info("Returning streaming response (SSE)")
        # Get rate limit info for headers
        _, limit, remaining, reset_timestamp = get_rate_limit_info(request)

        response = StreamingResponse(
            generate_sse_stream(query_request),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no",  # Disable nginx buffering
                "X-RateLimit-Limit": str(limit),
                "X-RateLimit-Remaining": str(remaining),
                "X-RateLimit-Reset": str(reset_timestamp),
            },
        )
        return response

    # Process query with timeout (30 seconds for LLM operations) - non-streaming
    try:
        query_response = await asyncio.wait_for(rag_service.process_query(query_request), timeout=30.0)

        logger.info(
            f"Query processed successfully (confidence={query_response.confidence}, "
            f"sources={len(query_response.sources)}, tokens={query_response.tokens_used.total_tokens})"
        )

        # Get rate limit info for headers
        _, limit, remaining, reset_timestamp = get_rate_limit_info(request)

        # Return JSON response with rate limit headers
        return JSONResponse(
            content=query_response.model_dump(),
            headers={
                "X-RateLimit-Limit": str(limit),
                "X-RateLimit-Remaining": str(remaining),
                "X-RateLimit-Reset": str(reset_timestamp),
            },
        )

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
        health_status["components"]["vector_search"] = "healthy" if vector_healthy else "unhealthy"
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
        health_status["components"]["database"] = "healthy" if chat_healthy else "unhealthy"
    except Exception as e:
        logger.error(f"Chat service health check failed: {str(e)}")
        health_status["components"]["database"] = "unhealthy"

    # Determine overall status
    all_healthy = all(status == "healthy" for status in health_status["components"].values())
    health_status["status"] = "healthy" if all_healthy else "degraded"

    return health_status
