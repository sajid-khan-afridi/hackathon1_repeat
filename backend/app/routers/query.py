"""Query router for the RAG chatbot API."""

import logging
import time
from typing import Dict, Any, Optional

from fastapi import APIRouter, Request, HTTPException, Depends
from fastapi.responses import StreamingResponse
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from slowapi import Limiter
from slowapi.util import get_remote_address

from app.config import get_settings
from app.models.query import (
    QueryRequest,
    QueryResponse,
    ErrorResponse,
    RateLimitResponse,
    OffTopicResponse,
    LowConfidenceResponse,
)
from app.models.chat import GetSessionResponse, DeleteSessionResponse, ChatSession, ChatMessage
from app.services.rag_service import rag_service
from app.services.chat_service import chat_service
from app.services.rate_limiter import (
    limiter,
    get_user_identifier,
    is_authenticated,
    format_retry_after,
)

logger = logging.getLogger(__name__)
router = APIRouter()
security = HTTPBearer(auto_error=False)
settings = get_settings()


async def extract_user_id(
    request: Request, credentials: Optional[HTTPAuthorizationCredentials] = Depends(security)
) -> str:
    """Extract user identifier from request for dependency injection."""
    # Try to get from Authorization header first
    if credentials:
        # In a real implementation, you'd validate the JWT token
        # For now, we'll use the token as the identifier
        return f"auth:{credentials.credentials[:8]}"

    # Fall back to IP address for anonymous users
    forwarded_for = request.headers.get("x-forwarded-for")
    if forwarded_for:
        ip = forwarded_for.split(",")[0].strip()
    else:
        ip = request.client.host if request.client else "unknown"

    return f"anon:{ip}"


@router.post("/query")
@limiter.limit(f"{settings.rate_limit_anonymous}/hour")
async def query(
    body: QueryRequest, request: Request, user_id: str = Depends(extract_user_id)
) -> Dict[str, Any]:
    """
    Process a user query and return an answer with citations.

    This endpoint implements the core RAG (Retrieval-Augmented Generation) functionality.
    It searches the textbook content for relevant passages and generates an answer
    based on the retrieved context.

    Args:
        body: Query request containing the question and optional filters
        request: The HTTP request object
        user_id: User identifier for rate limiting

    Returns:
        Response containing the answer, sources, and metadata

    Raises:
        HTTPException: For various error conditions
    """
    correlation_id = getattr(request.state, "correlation_id", "unknown")
    logger.info(f"Processing query {correlation_id}: {body.query[:100]}...")

    try:
        # Check if client wants streaming response
        accept_header = request.headers.get("accept", "")
        wants_stream = "text/event-stream" in accept_header

        if wants_stream:
            # Return streaming response
            return StreamingResponse(
                stream_response(body, user_id, correlation_id),
                media_type="text/event-stream",
                headers={
                    "Cache-Control": "no-cache",
                    "Connection": "keep-alive",
                    "X-Accel-Buffering": "no",  # Disable nginx buffering
                },
            )
        else:
            # Process query normally
            return await rag_service.process_query(body, user_id)

    except TimeoutError:
        logger.error(f"Query timeout {correlation_id}")
        raise HTTPException(
            status_code=503,
            detail=ErrorResponse(
                error="Query processing timed out",
                code="TIMEOUT_ERROR",
                suggestedAction="Please try again with a shorter query",
            ).dict(),
        )

    except ConnectionError as e:
        logger.error(f"Connection error {correlation_id}: {e}")
        raise HTTPException(
            status_code=503,
            detail=ErrorResponse(
                error="External service unavailable",
                code="SERVICE_UNAVAILABLE",
                suggestedAction="Please try again in a few moments",
            ).dict(),
        )

    except ValueError as e:
        logger.warning(f"Invalid query {correlation_id}: {e}")
        raise HTTPException(
            status_code=400, detail=ErrorResponse(error=str(e), code="INVALID_QUERY").dict()
        )

    except Exception as e:
        logger.error(f"Unexpected error {correlation_id}: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail=ErrorResponse(error="Internal server error", code="INTERNAL_ERROR").dict(),
        )


async def stream_response(request: QueryRequest, user_id: str, correlation_id: str):
    """Stream response chunks for Server-Sent Events."""
    start_time = time.time()

    try:
        # Get session
        session_id = await rag_service._get_or_create_session(request.sessionId, user_id)

        # Send initial chunk
        yield f"data: {format_sse_chunk({'type': 'start', 'sessionId': session_id})}\n\n"

        # Create embedding
        query_embedding = await rag_service.llm_service.create_embedding(request.query)

        # Search for context
        context_chunks = await rag_service.vector_service.search_context(
            query_embedding=query_embedding,
            limit=rag_service.settings.rag_max_context_chunks,
            score_threshold=rag_service.settings.rag_min_confidence,
            filters=rag_service._prepare_filters(request.filters),
        )

        # Check if we have context
        if not context_chunks:
            off_topic = await rag_service._handle_off_topic_query(request.query, session_id)
            yield f"data: {format_sse_chunk({'type': 'off_topic', **off_topic})}\n\n"
            return

        # Calculate confidence
        confidence = rag_service._calculate_confidence(context_chunks)

        # Prepare conversation history
        conversation_history = None
        if session_id:
            conversation_history = await rag_service.chat_service.get_conversation_context(
                session_id, limit=rag_service.settings.chat_history_limit
            )

        # Stream the answer
        answer_chunks = []
        async for chunk in rag_service.llm_service.generate_answer_stream(
            request.query, context_chunks, conversation_history
        ):
            answer_chunks.append(chunk)
            yield f"data: {format_sse_chunk({'type': 'chunk', 'chunk': chunk})}\n\n"

        # Combine answer
        answer = "".join(answer_chunks)

        # Create citations
        citations = rag_service._create_citations(context_chunks)

        # Calculate metrics
        response_time = time.time() - start_time
        estimated_tokens = len(answer.split()) + len(request.query.split()) * 2
        token_usage = {
            "prompt_tokens": int(estimated_tokens * 1.3),
            "completion_tokens": int(estimated_tokens * 0.7),
            "total_tokens": estimated_tokens,
        }

        # Send final chunk with all metadata
        final_chunk = {
            "type": "end",
            "answer": answer,
            "confidence": confidence,
            "sources": [c.dict() for c in citations],
            "tokenUsage": token_usage,
            "responseTime": response_time,
            "filterMessage": rag_service._get_filter_message(
                rag_service._prepare_filters(request.filters), confidence
            ),
        }

        yield f"data: {format_sse_chunk(final_chunk)}\n\n"

        # Save to session
        if session_id:
            await rag_service.chat_service.add_message(
                session_id=session_id, role="user", content=request.query
            )
            await rag_service.chat_service.add_message(
                session_id=session_id, role="assistant", content=answer, token_usage=token_usage
            )

    except Exception as e:
        logger.error(f"Streaming error {correlation_id}: {e}")
        yield f"data: {format_sse_chunk({'type': 'error', 'error': str(e)})}\n\n"


def format_sse_chunk(data: Dict) -> str:
    """Format data for Server-Sent Events."""
    import json

    return json.dumps(data)


@router.get("/chat/sessions/{session_id}")
async def get_session(session_id: str, http_request: Request) -> GetSessionResponse:
    """
    Get session details and conversation history.

    Args:
        session_id: The session ID to retrieve
        http_request: The HTTP request object

    Returns:
        Session details and all messages in the session

    Raises:
        HTTPException: If session not found
    """
    correlation_id = getattr(http_request.state, "correlation_id", "unknown")
    logger.info(f"Getting session {session_id} [{correlation_id}]")

    try:
        # Get session
        session = await chat_service.get_session(session_id=session_id)
        if not session:
            raise HTTPException(
                status_code=404,
                detail=ErrorResponse(error="Session not found", code="SESSION_NOT_FOUND").dict(),
            )

        # Get messages
        messages = await chat_service.get_messages(session_id)

        return GetSessionResponse(session=session, messages=messages)

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting session {session_id}: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail=ErrorResponse(error="Failed to retrieve session", code="INTERNAL_ERROR").dict(),
        )


@router.delete("/chat/sessions/{session_id}")
async def delete_session(session_id: str, http_request: Request) -> DeleteSessionResponse:
    """
    Delete a session and all its messages.

    Args:
        session_id: The session ID to delete
        http_request: The HTTP request object

    Returns:
        Deletion status

    Raises:
        HTTPException: If session not found or deletion fails
    """
    correlation_id = getattr(http_request.state, "correlation_id", "unknown")
    logger.info(f"Deleting session {session_id} [{correlation_id}]")

    try:
        # Check if session exists
        session = await chat_service.get_session(session_id=session_id)
        if not session:
            raise HTTPException(
                status_code=404,
                detail=ErrorResponse(error="Session not found", code="SESSION_NOT_FOUND").dict(),
            )

        # Delete session
        success = await chat_service.delete_session(session_id)

        if success:
            return DeleteSessionResponse(
                success=True, message=f"Session {session_id} deleted successfully"
            )
        else:
            raise HTTPException(
                status_code=500,
                detail=ErrorResponse(
                    error="Failed to delete session", code="DELETION_FAILED"
                ).dict(),
            )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting session {session_id}: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail=ErrorResponse(error="Failed to delete session", code="INTERNAL_ERROR").dict(),
        )
