from fastapi import APIRouter, HTTPException, Depends, Query
from typing import List, Optional
import logging

from ..models.personalize import (
    UserProfileUpdate, UserProfileResponse, UserPreferences,
    ConversationPreferences, UserAnalytics
)
from ..services.user_service import UserService
from ..services.analytics_service import AnalyticsService

logger = logging.getLogger(__name__)
router = APIRouter()

# Initialize services
user_service = UserService()
analytics_service = AnalyticsService()


async def get_current_user():
    """
    TODO: Replace with actual auth dependency
    """
    return {"user_id": "dev_user", "email": "dev@example.com"}


@router.get("/profile", response_model=UserProfileResponse)
async def get_user_profile(current_user: dict = Depends(get_current_user)):
    """
    Get user profile with preferences and analytics
    """
    try:
        profile = await user_service.get_user_profile(current_user["user_id"])
        if not profile:
            raise HTTPException(status_code=404, detail="User profile not found")

        # Get user analytics
        analytics = await analytics_service.get_user_analytics(current_user["user_id"])

        return UserProfileResponse(
            user_id=profile["user_id"],
            username=profile["username"],
            full_name=profile.get("full_name"),
            email=profile["email"],
            avatar_url=profile.get("avatar_url"),
            bio=profile.get("bio"),
            preferences=UserPreferences(**profile.get("preferences", {})),
            statistics=analytics,
            created_at=profile["created_at"],
            updated_at=profile["updated_at"]
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Get profile error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to retrieve user profile")


@router.put("/profile", response_model=UserProfileResponse)
async def update_user_profile(
    profile_update: UserProfileUpdate,
    current_user: dict = Depends(get_current_user)
):
    """
    Update user profile
    """
    try:
        updated_profile = await user_service.update_user_profile(
            user_id=current_user["user_id"],
            profile_data=profile_update.dict(exclude_unset=True)
        )

        if not updated_profile:
            raise HTTPException(status_code=404, detail="User profile not found")

        # Get user analytics
        analytics = await analytics_service.get_user_analytics(current_user["user_id"])

        return UserProfileResponse(
            user_id=updated_profile["user_id"],
            username=updated_profile["username"],
            full_name=updated_profile.get("full_name"),
            email=updated_profile["email"],
            avatar_url=updated_profile.get("avatar_url"),
            bio=updated_profile.get("bio"),
            preferences=UserPreferences(**updated_profile.get("preferences", {})),
            statistics=analytics,
            created_at=updated_profile["created_at"],
            updated_at=updated_profile["updated_at"]
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Update profile error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to update user profile")


@router.get("/preferences", response_model=UserPreferences)
async def get_user_preferences(current_user: dict = Depends(get_current_user)):
    """
    Get user preferences
    """
    try:
        preferences = await user_service.get_user_preferences(current_user["user_id"])

        if not preferences:
            # Return default preferences if none exist
            return UserPreferences()

        return UserPreferences(**preferences)

    except Exception as e:
        logger.error(f"Get preferences error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to retrieve user preferences")


@router.put("/preferences", response_model=UserPreferences)
async def update_user_preferences(
    preferences: UserPreferences,
    current_user: dict = Depends(get_current_user)
):
    """
    Update user preferences
    """
    try:
        updated_preferences = await user_service.update_user_preferences(
            user_id=current_user["user_id"],
            preferences=preferences.dict()
        )

        return UserPreferences(**updated_preferences)

    except Exception as e:
        logger.error(f"Update preferences error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to update user preferences")


@router.get("/analytics", response_model=UserAnalytics)
async def get_user_analytics(current_user: dict = Depends(get_current_user)):
    """
    Get user analytics and statistics
    """
    try:
        analytics = await analytics_service.get_user_analytics(current_user["user_id"])
        return UserAnalytics(**analytics)

    except Exception as e:
        logger.error(f"Get analytics error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to retrieve user analytics")


@router.get("/conversations/preferences")
async def get_conversation_preferences(
    conversation_id: Optional[str] = Query(None),
    current_user: dict = Depends(get_current_user)
):
    """
    Get conversation-specific preferences
    """
    try:
        if conversation_id:
            preferences = await user_service.get_conversation_preferences(
                conversation_id=conversation_id,
                user_id=current_user["user_id"]
            )
        else:
            # Get default conversation preferences
            preferences = await user_service.get_default_conversation_preferences(
                current_user["user_id"]
            )

        return preferences or ConversationPreferences()

    except Exception as e:
        logger.error(f"Get conversation preferences error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to retrieve conversation preferences")


@router.put("/conversations/preferences")
async def update_conversation_preferences(
    conversation_id: str,
    preferences: ConversationPreferences,
    current_user: dict = Depends(get_current_user)
):
    """
    Update conversation-specific preferences
    """
    try:
        updated_preferences = await user_service.update_conversation_preferences(
            conversation_id=conversation_id,
            user_id=current_user["user_id"],
            preferences=preferences.dict()
        )

        return updated_preferences

    except Exception as e:
        logger.error(f"Update conversation preferences error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to update conversation preferences")


@router.post("/feedback/preferences")
async def update_feedback_preferences(
    query_type: str,
    preferred_response_style: Optional[str] = None,
    preferred_response_length: Optional[str] = None,
    current_user: dict = Depends(get_current_user)
):
    """
    Update preferences based on user feedback patterns
    """
    try:
        # This endpoint would be called when users provide feedback
        # to learn their preferences over time
        await analytics_service.update_feedback_preferences(
            user_id=current_user["user_id"],
            query_type=query_type,
            preferred_style=preferred_response_style,
            preferred_length=preferred_response_length
        )

        return {"message": "Feedback preferences updated successfully"}

    except Exception as e:
        logger.error(f"Update feedback preferences error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to update feedback preferences")


@router.get("/recommendations")
async def get_personalization_recommendations(
    current_user: dict = Depends(get_current_user)
):
    """
    Get personalization recommendations based on user behavior
    """
    try:
        recommendations = await analytics_service.get_personalization_recommendations(
            current_user["user_id"]
        )

        return recommendations

    except Exception as e:
        logger.error(f"Get recommendations error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to get recommendations")