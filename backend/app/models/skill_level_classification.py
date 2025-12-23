"""
Skill level classification models for personalization engine.
Pydantic models for computed skill level tier based on user profile.
"""

from pydantic import BaseModel, Field
from uuid import UUID
from datetime import datetime
from typing import Literal, Dict, Any

# Type definitions
SkillLevel = Literal["beginner", "intermediate", "advanced"]


class SkillLevelClassificationBase(BaseModel):
    """Base skill level classification model."""

    skill_level: SkillLevel = Field(..., description="Computed skill tier")


class SkillLevelClassificationCreate(BaseModel):
    """Model for creating a new skill level classification."""

    user_id: UUID = Field(..., description="User this classification belongs to")
    skill_level: SkillLevel = Field(..., description="Computed skill tier")
    based_on_profile: Dict[str, Any] = Field(..., description="Snapshot of profile attributes used for classification")


class SkillLevelClassificationUpdate(BaseModel):
    """Model for updating skill level classification."""

    skill_level: SkillLevel = Field(..., description="Updated skill tier")
    based_on_profile: Dict[str, Any] = Field(..., description="Updated profile snapshot")


class SkillLevelClassificationResponse(BaseModel):
    """Skill level classification response model for API responses."""

    id: UUID
    user_id: UUID
    skill_level: SkillLevel
    calculated_at: datetime
    updated_at: datetime
    based_on_profile: Dict[str, Any]

    class Config:
        from_attributes = True


class SkillLevelClassificationInDB(BaseModel):
    """Skill level classification model with all database fields."""

    id: UUID
    user_id: UUID
    skill_level: SkillLevel
    calculated_at: datetime
    updated_at: datetime
    based_on_profile: Dict[str, Any]

    class Config:
        from_attributes = True
