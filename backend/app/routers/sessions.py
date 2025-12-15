"""
Session management endpoints for chat history.
"""
import logging
from fastapi import APIRouter, HTTPException, status, Path
from uuid import UUID
from app.services.chat_service import chat_service
from app.models.chat import SessionResponse, ChatSession, ChatMessage
from typing import List

router = APIRouter()
logger = logging.getLogger(__name__)


@router.get("/sessions/{session_id}", response_model=SessionResponse)
async def get_session(
    session_id: UUID = Path(..., description="Session UUID"),
) -> SessionResponse:
    """
    Retrieve session with full conversation history.

    Args:
        session_id: UUID of the session to retrieve

    Returns:
        SessionResponse with session data and all messages

    Raises:
        HTTPException 404: Session not found
        HTTPException 500: Database error
    """
    try:
        # Get session data
        session = await chat_service.get_session(session_id)

        if not session:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Session {session_id} not found",
            )

        # Get conversation history (all messages, not just last 5)
        messages = await chat_service.get_conversation_history(session_id, limit=1000)

        # Convert to response format
        message_objects = []
        for msg in messages:
            message_objects.append(
                ChatMessage(
                    id=msg.get("id"),
                    session_id=session_id,
                    role=msg["role"],
                    content=msg["content"],
                    tokens_used=msg.get("tokens_used", {}),
                    confidence=msg.get("confidence"),
                    created_at=msg.get("created_at"),
                )
            )

        logger.info(f"Retrieved session {session_id} with {len(message_objects)} messages")

        return SessionResponse(
            session=ChatSession(**session),
            messages=message_objects,
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error retrieving session {session_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to retrieve session",
        )


@router.delete("/sessions/{session_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_session(
    session_id: UUID = Path(..., description="Session UUID"),
) -> None:
    """
    Delete session and all associated messages and citations.

    Args:
        session_id: UUID of the session to delete

    Returns:
        204 No Content on success

    Raises:
        HTTPException 404: Session not found
        HTTPException 500: Database error
    """
    try:
        # Verify session exists
        session = await chat_service.get_session(session_id)

        if not session:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Session {session_id} not found",
            )

        # Delete session (cascades to messages and citations)
        await chat_service.delete_session(session_id)

        logger.info(f"Deleted session {session_id}")

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting session {session_id}: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to delete session",
        )
