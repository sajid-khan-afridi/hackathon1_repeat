"""
Chapter progress models for personalization engine.
Pydantic models for tracking user's chapter interaction state.
"""

from pydantic import BaseModel, Field
from uuid import UUID
from datetime import datetime
from typing import Literal, Optional

# Type definitions
ProgressStatus = Literal["started", "completed"]


class ChapterProgressBase(BaseModel):
    """Base chapter progress model."""

    chapter_id: str = Field(..., description="Identifier for the chapter (e.g., 'module-1/ros-intro')")
    status: ProgressStatus = Field(..., description="Current progress status")


class ChapterProgressCreate(BaseModel):
    """Model for creating a new chapter progress record."""

    user_id: UUID = Field(..., description="User this progress belongs to")
    chapter_id: str = Field(..., description="Identifier for the chapter")
    status: ProgressStatus = Field(default="started", description="Initial progress status")


class ChapterProgressUpdate(BaseModel):
    """Model for updating chapter progress."""

    status: ProgressStatus | None = None
    is_bookmarked: bool | None = None


class ChapterProgressMarkStarted(BaseModel):
    """Model for marking a chapter as started."""

    chapter_id: str = Field(..., description="Chapter identifier")


class ChapterProgressMarkCompleted(BaseModel):
    """Model for marking a chapter as completed."""

    chapter_id: str = Field(..., description="Chapter identifier")


class ChapterProgressToggleBookmark(BaseModel):
    """Model for toggling chapter bookmark."""

    chapter_id: str = Field(..., description="Chapter identifier")


class ChapterProgressResponse(BaseModel):
    """Chapter progress response model for API responses."""

    id: UUID
    user_id: UUID
    chapter_id: str
    status: ProgressStatus
    is_bookmarked: bool
    started_at: datetime
    completed_at: Optional[datetime]
    updated_at: datetime

    class Config:
        from_attributes = True


class ChapterProgressInDB(BaseModel):
    """Chapter progress model with all database fields."""

    id: UUID
    user_id: UUID
    chapter_id: str
    status: ProgressStatus
    is_bookmarked: bool
    started_at: datetime
    completed_at: Optional[datetime]
    updated_at: datetime

    class Config:
        from_attributes = True
