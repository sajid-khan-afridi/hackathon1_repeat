from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks
from typing import List, Optional
import time
import uuid
import logging

from ..models.chat import ChatRequest, ChatResponse, SourceDocument, ConversationHistory
from ..services.rag_service import RAGService
from ..services.conversation_service import ConversationService

logger = logging.getLogger(__name__)
router = APIRouter()

# Initialize services
rag_service = RAGService()
conversation_service = ConversationService()


@router.post("/query", response_model=ChatResponse)
async def chat_query(
    request: ChatRequest,
    background_tasks: BackgroundTasks,
    user: dict = Depends(lambda: {"user_id": "dev_user"})  # TODO: Use actual auth dependency
):
    """
    Process a chat query using RAG (Retrieval-Augmented Generation)
    """
    try:
        start_time = time.time()

        # Generate or use existing conversation ID
        conversation_id = request.conversation_id or str(uuid.uuid4())

        # Log the query in background
        background_tasks.add_task(
            conversation_service.add_message,
            conversation_id,
            "user",
            request.query
        )

        # Process query with RAG
        result = await rag_service.query(
            query=request.query,
            context=request.context,
            temperature=request.temperature,
            max_tokens=request.max_tokens,
            include_sources=request.include_sources
        )

        response_time = time.time() - start_time

        # Create response
        response = ChatResponse(
            answer=result["answer"],
            conversation_id=conversation_id,
            sources=result.get("sources", []),
            query=request.query,
            response_time=response_time,
            model_used=result.get("model_used")
        )

        # Log assistant response in background
        background_tasks.add_task(
            conversation_service.add_message,
            conversation_id,
            "assistant",
            response.answer
        )

        return response

    except Exception as e:
        logger.error(f"Chat query error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to process chat query")


@router.get("/conversations", response_model=List[ConversationHistory])
async def get_conversations(
    limit: int = 10,
    offset: int = 0,
    user: dict = Depends(lambda: {"user_id": "dev_user"})
):
    """
    Get user's conversation history
    """
    try:
        conversations = await conversation_service.get_user_conversations(
            user_id=user["user_id"],
            limit=limit,
            offset=offset
        )
        return conversations
    except Exception as e:
        logger.error(f"Get conversations error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to retrieve conversations")


@router.get("/conversations/{conversation_id}", response_model=ConversationHistory)
async def get_conversation(
    conversation_id: str,
    user: dict = Depends(lambda: {"user_id": "dev_user"})
):
    """
    Get a specific conversation by ID
    """
    try:
        conversation = await conversation_service.get_conversation(
            conversation_id=conversation_id,
            user_id=user["user_id"]
        )

        if not conversation:
            raise HTTPException(status_code=404, detail="Conversation not found")

        return conversation
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Get conversation error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to retrieve conversation")


@router.delete("/conversations/{conversation_id}")
async def delete_conversation(
    conversation_id: str,
    user: dict = Depends(lambda: {"user_id": "dev_user"})
):
    """
    Delete a conversation
    """
    try:
        success = await conversation_service.delete_conversation(
            conversation_id=conversation_id,
            user_id=user["user_id"]
        )

        if not success:
            raise HTTPException(status_code=404, detail="Conversation not found")

        return {"message": "Conversation deleted successfully"}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Delete conversation error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to delete conversation")


@router.post("/feedback")
async def submit_feedback(
    conversation_id: str,
    message_id: str,
    rating: int,
    comment: Optional[str] = None,
    user: dict = Depends(lambda: {"user_id": "dev_user"})
):
    """
    Submit feedback for a chat response
    """
    try:
        if not 1 <= rating <= 5:
            raise HTTPException(status_code=400, detail="Rating must be between 1 and 5")

        await conversation_service.add_feedback(
            conversation_id=conversation_id,
            message_id=message_id,
            user_id=user["user_id"],
            rating=rating,
            comment=comment
        )

        return {"message": "Feedback submitted successfully"}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Submit feedback error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to submit feedback")