"""
Chapter metadata models for personalization engine.
Pydantic models for chapter difficulty, prerequisites, and learning goal tags.
"""

from pydantic import BaseModel, Field
from datetime import datetime
from typing import List, Literal

# Type definitions
DifficultyLevel = Literal["beginner", "intermediate", "advanced"]
LearningGoalTag = Literal["practical", "theoretical", "research"]


class ChapterMetadataBase(BaseModel):
    """Base chapter metadata model."""

    chapter_id: str = Field(..., description="Unique identifier for the chapter (e.g., 'module-1/ros-intro')")
    module_number: int = Field(..., gt=0, description="Module number for prerequisite ordering")
    title: str = Field(..., max_length=500, description="Human-readable chapter title")
    difficulty_level: DifficultyLevel = Field(..., description="Difficulty classification")
    prerequisites: List[str] = Field(default_factory=list, description="Array of chapter_id strings that must be completed first")
    requires_hardware: bool = Field(default=False, description="Whether chapter requires physical robot hardware")
    learning_goal_tags: List[LearningGoalTag] = Field(default_factory=list, description="Array of learning goal tags")


class ChapterMetadataCreate(ChapterMetadataBase):
    """Model for creating new chapter metadata."""
    pass


class ChapterMetadataUpdate(BaseModel):
    """Model for updating chapter metadata (all fields optional)."""

    module_number: int | None = Field(None, gt=0)
    title: str | None = Field(None, max_length=500)
    difficulty_level: DifficultyLevel | None = None
    prerequisites: List[str] | None = None
    requires_hardware: bool | None = None
    learning_goal_tags: List[LearningGoalTag] | None = None


class ChapterMetadataResponse(ChapterMetadataBase):
    """Chapter metadata response model for API responses."""

    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class ChapterMetadataInDB(ChapterMetadataBase):
    """Chapter metadata model with all database fields."""

    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
