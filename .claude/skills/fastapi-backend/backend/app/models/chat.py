from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime


class SourceDocument(BaseModel):
    """Represents a source document used in RAG response"""
    content: str
    source: str
    page: Optional[int] = None
    score: float = Field(ge=0.0, le=1.0)
    metadata: Optional[dict] = {}


class ChatRequest(BaseModel):
    """Request model for chat queries"""
    query: str = Field(..., min_length=1, max_length=1000, description="User's query/question")
    conversation_id: Optional[str] = None
    context: Optional[str] = None
    temperature: Optional[float] = Field(0.7, ge=0.0, le=2.0)
    max_tokens: Optional[int] = Field(500, ge=50, le=2000)
    include_sources: Optional[bool] = True


class ChatResponse(BaseModel):
    """Response model for chat queries"""
    answer: str
    conversation_id: str
    sources: List[SourceDocument] = []
    query: str
    response_time: float
    model_used: Optional[str] = None
    timestamp: datetime = Field(default_factory=datetime.utcnow)


class ConversationHistory(BaseModel):
    """Model for conversation history"""
    conversation_id: str
    messages: List[dict]
    created_at: datetime
    updated_at: datetime