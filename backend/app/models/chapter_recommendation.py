"""
Chapter recommendation models for personalization engine.
Pydantic models for recommended chapters (ephemeral, computed on-demand).
"""

from pydantic import BaseModel, Field
from uuid import UUID
from datetime import datetime
from typing import List


class ChapterRecommendation(BaseModel):
    """Model for a recommended chapter (computed, not stored in database)."""

    user_id: UUID = Field(..., description="User this recommendation is for")
    chapter_id: str = Field(..., description="Identifier for the recommended chapter")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Computed relevance score (0.0-1.0)")
    recommended_at: datetime = Field(..., description="When recommendation was generated")
    reason: str = Field(..., description="Human-readable explanation of why this chapter was recommended")


class ChapterRecommendationResponse(BaseModel):
    """Chapter recommendation response model for API responses."""

    chapter_id: str
    relevance_score: float
    recommended_at: datetime
    reason: str
    # Include chapter metadata for display
    title: str | None = None
    difficulty_level: str | None = None
    module_number: int | None = None


class RecommendationsRequest(BaseModel):
    """Request model for getting recommendations."""

    force_refresh: bool = Field(default=False, description="Force recompute instead of using cache")


class RecommendationsResponse(BaseModel):
    """Response model containing list of recommendations."""

    user_id: UUID
    recommendations: List[ChapterRecommendationResponse]
    generated_at: datetime
    from_cache: bool = Field(..., description="Whether recommendations came from cache")
