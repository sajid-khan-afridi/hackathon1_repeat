from pydantic import BaseModel, Field
from typing import Optional, Dict, Any
from datetime import datetime


class UserPreferences(BaseModel):
    """User preferences for personalization"""
    language: str = Field("en", max_length=10)
    theme: str = Field("light", regex="^(light|dark|auto)$")
    response_length: str = Field("medium", regex="^(short|medium|long)$")
    response_style: str = Field("neutral", regex="^(casual|neutral|formal)$")
    notifications_enabled: bool = True
    auto_save_conversations: bool = True


class UserProfileUpdate(BaseModel):
    """User profile update request"""
    username: Optional[str] = Field(None, min_length=3, max_length=50)
    full_name: Optional[str] = Field(None, max_length=100)
    avatar_url: Optional[str] = None
    bio: Optional[str] = Field(None, max_length=500)


class UserProfileResponse(BaseModel):
    """User profile response"""
    user_id: str
    username: str
    full_name: Optional[str] = None
    email: str
    avatar_url: Optional[str] = None
    bio: Optional[str] = None
    preferences: UserPreferences
    statistics: Optional[Dict[str, Any]] = {}
    created_at: datetime
    updated_at: datetime


class ConversationPreferences(BaseModel):
    """Conversation-specific preferences"""
    auto_title: bool = True
    save_to_history: bool = True
    share_analytics: bool = False
    custom_instructions: Optional[str] = None


class UserAnalytics(BaseModel):
    """User analytics for personalization"""
    total_conversations: int = 0
    total_queries: int = 0
    average_response_time: float = 0.0
    favorite_topics: list[str] = []
    query_patterns: Dict[str, int] = {}
    feedback_score: Optional[float] = None