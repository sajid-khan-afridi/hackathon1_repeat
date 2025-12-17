"""
Pydantic models for chat sessions and messages.
"""

from pydantic import BaseModel, Field, UUID4
from typing import Optional, Dict, Any, List
from datetime import datetime
from enum import Enum


class MessageRole(str, Enum):
    """Message role enumeration."""

    USER = "user"
    ASSISTANT = "assistant"


class ChatSession(BaseModel):
    """Chat session model."""

    id: UUID4 = Field(..., description="Unique session identifier")
    user_id: Optional[UUID4] = Field(None, description="User ID for authenticated sessions")
    session_token: str = Field(..., description="Token for anonymous session identification")
    created_at: datetime = Field(..., description="Session creation timestamp")
    last_activity_at: datetime = Field(..., description="Last message timestamp")
    metadata: Dict[str, Any] = Field(default_factory=dict, description="Extensible metadata")

    class Config:
        from_attributes = True


class ChatMessage(BaseModel):
    """Chat message model."""

    id: UUID4 = Field(..., description="Unique message identifier")
    session_id: UUID4 = Field(..., description="Parent session reference")
    role: MessageRole = Field(..., description="Message author role")
    content: str = Field(..., min_length=1, max_length=10000, description="Message text content")
    tokens_used: Dict[str, int] = Field(
        default_factory=dict, description="Token usage: {input, output, total}"
    )
    confidence: Optional[float] = Field(
        None, ge=0.0, le=1.0, description="Confidence score (assistant only)"
    )
    created_at: datetime = Field(..., description="Message timestamp")

    class Config:
        from_attributes = True
        use_enum_values = True


class SourceCitationDB(BaseModel):
    """Source citation database model."""

    id: UUID4 = Field(..., description="Unique citation identifier")
    message_id: UUID4 = Field(..., description="Parent message reference")
    chapter_id: str = Field(..., description="Chapter identifier")
    chapter_title: str = Field(..., description="Chapter title")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Relevance score")
    excerpt: str = Field(..., description="Text excerpt")
    position: int = Field(..., ge=1, description="Position in ranked results")

    class Config:
        from_attributes = True


class SessionCreate(BaseModel):
    """Request model for creating a session."""

    user_id: Optional[UUID4] = Field(None, description="User ID for authenticated sessions")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Session metadata")


class SessionResponse(BaseModel):
    """Response model for session operations."""

    session: ChatSession = Field(..., description="Session data")
    messages: List[ChatMessage] = Field(..., description="Session messages")
