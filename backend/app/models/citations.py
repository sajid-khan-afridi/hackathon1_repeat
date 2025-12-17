"""
Pydantic models for source citations.
"""

from pydantic import BaseModel, Field, UUID4
from typing import List


class CitationCreate(BaseModel):
    """Model for creating a source citation."""

    message_id: UUID4 = Field(..., description="Parent message ID")
    chapter_id: str = Field(..., description="Chapter identifier")
    chapter_title: str = Field(..., description="Chapter title")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Relevance score")
    excerpt: str = Field(..., max_length=500, description="Text excerpt")
    position: int = Field(..., ge=1, description="Position in ranked results")


class Citation(BaseModel):
    """Citation model with all fields."""

    id: UUID4 = Field(..., description="Citation ID")
    message_id: UUID4 = Field(..., description="Parent message ID")
    chapter_id: str = Field(..., description="Chapter identifier")
    chapter_title: str = Field(..., description="Chapter title")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Relevance score")
    excerpt: str = Field(..., description="Text excerpt")
    position: int = Field(..., ge=1, description="Position in ranked results")

    class Config:
        from_attributes = True


class MessageWithCitations(BaseModel):
    """Message with associated citations."""

    message_id: UUID4 = Field(..., description="Message ID")
    role: str = Field(..., description="Message role")
    content: str = Field(..., description="Message content")
    confidence: float = Field(None, description="Confidence score")
    citations: List[Citation] = Field(..., description="Associated citations")
