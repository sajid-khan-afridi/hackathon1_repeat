"""Chat session and message models."""

from datetime import datetime
from typing import List, Optional
from enum import Enum

from pydantic import BaseModel, Field
import uuid


class MessageRole(str, Enum):
    """Role of a message sender."""

    USER = "user"
    ASSISTANT = "assistant"


class ChatMessage(BaseModel):
    """Individual chat message."""

    id: str = Field(default_factory=lambda: str(uuid.uuid4()), description="Message ID")
    sessionId: str = Field(..., description="Session ID this message belongs to")
    role: MessageRole = Field(..., description="Message role (user/assistant)")
    content: str = Field(..., description="Message content")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Message timestamp")
    tokenUsage: Optional[dict] = Field(None, description="Token usage for assistant messages")
    metadata: Optional[dict] = Field(None, description="Additional metadata")


class ChatSession(BaseModel):
    """Chat session information."""

    id: str = Field(default_factory=lambda: str(uuid.uuid4()), description="Session ID")
    sessionToken: str = Field(
        default_factory=lambda: str(uuid.uuid4()), description="Session token for client"
    )
    userId: Optional[str] = Field(None, description="User ID (if authenticated)")
    createdAt: datetime = Field(
        default_factory=datetime.utcnow, description="Session creation time"
    )
    lastActivityAt: datetime = Field(
        default_factory=datetime.utcnow, description="Last activity time"
    )
    messageCount: int = Field(default=0, description="Number of messages in session")
    isActive: bool = Field(default=True, description="Whether session is active")
    metadata: Optional[dict] = Field(None, description="Additional session metadata")

    class Config:
        json_encoders = {datetime: lambda v: v.isoformat()}


class CreateSessionRequest(BaseModel):
    """Request to create a new chat session."""

    userId: Optional[str] = Field(None, description="Optional user ID")


class CreateSessionResponse(BaseModel):
    """Response when creating a new session."""

    sessionId: str = Field(..., description="Session ID")
    sessionToken: str = Field(..., description="Session token for client")


class GetSessionResponse(BaseModel):
    """Response when retrieving a session."""

    session: ChatSession = Field(..., description="Session details")
    messages: List[ChatMessage] = Field(..., description="Messages in the session")


class DeleteSessionResponse(BaseModel):
    """Response when deleting a session."""

    success: bool = Field(..., description="Whether deletion was successful")
    message: str = Field(..., description="Deletion status message")


class SessionStats(BaseModel):
    """Statistics about a session."""

    sessionId: str = Field(..., description="Session ID")
    messageCount: int = Field(..., description="Total messages")
    userMessages: int = Field(..., description="User messages")
    assistantMessages: int = Field(..., description="Assistant messages")
    totalTokens: int = Field(..., description="Total tokens used")
    sessionDuration: Optional[float] = Field(None, description="Session duration in seconds")
    firstMessageAt: Optional[datetime] = Field(None, description="First message timestamp")
    lastMessageAt: Optional[datetime] = Field(None, description="Last message timestamp")
