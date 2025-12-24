"""
User profile models for personalization.
Pydantic models for user learning preferences.
"""

from pydantic import BaseModel, ConfigDict
from pydantic.alias_generators import to_camel
from uuid import UUID
from datetime import datetime
from typing import Optional, Literal

# Profile field value types
ExperienceLevel = Literal["beginner", "intermediate", "advanced"]
ROSFamiliarity = Literal["none", "basic", "proficient"]
HardwareAccess = Literal["simulation_only", "jetson_kit", "full_robot_lab"]
LearningGoal = Literal["career_transition", "academic_research", "hobby"]
PreferredLanguage = Literal["python", "cpp", "both"]


class ProfileBase(BaseModel):
    """Base profile model with learning preference fields."""

    model_config = ConfigDict(
        alias_generator=to_camel,
        populate_by_name=True,  # Accept both snake_case and camelCase
    )

    experience_level: Optional[ExperienceLevel] = None
    ros_familiarity: Optional[ROSFamiliarity] = None
    hardware_access: Optional[HardwareAccess] = None
    learning_goal: Optional[LearningGoal] = None
    preferred_language: Optional[PreferredLanguage] = None


class ProfileCreate(ProfileBase):
    """Model for creating a new profile."""

    pass


class ProfileUpdate(ProfileBase):
    """Model for updating profile fields (all fields optional)."""

    pass


class ProfileResponse(ProfileBase):
    """Profile response model for API responses."""

    model_config = ConfigDict(
        alias_generator=to_camel,
        populate_by_name=True,
        from_attributes=True,
        serialize_by_alias=True,  # Serialize to camelCase for frontend
    )

    id: UUID
    user_id: UUID
    is_complete: bool
    created_at: datetime
    updated_at: datetime


class ProfileInDB(ProfileBase):
    """Profile model with all database fields."""

    model_config = ConfigDict(
        alias_generator=to_camel,
        populate_by_name=True,
        from_attributes=True,
    )

    id: UUID
    user_id: UUID
    is_complete: bool = False
    created_at: datetime
    updated_at: datetime
