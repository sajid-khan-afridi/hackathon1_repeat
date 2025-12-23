"""
User profile router for profile management endpoints.
"""

import logging
from uuid import UUID

from fastapi import APIRouter, Depends, HTTPException, status, Response

from app.middleware.auth import AuthenticatedUser, get_current_user
from app.models.profile import ProfileCreate, ProfileUpdate, ProfileResponse
from app.services.profile_service import profile_service
from app.services.auth_service import auth_service

logger = logging.getLogger(__name__)

router = APIRouter()


@router.options("/profile")
async def options_profile():
    """
    Handle OPTIONS preflight for /profile endpoint.
    CORS middleware should handle this, but explicit handler ensures it works.
    """
    return Response(
        status_code=200,
        headers={
            "Access-Control-Allow-Origin": "*",
            "Access-Control-Allow-Methods": "GET, POST, PUT, DELETE, OPTIONS",
            "Access-Control-Allow-Headers": "*",
            "Access-Control-Allow-Credentials": "true",
        }
    )


@router.get("/profile", response_model=ProfileResponse)
async def get_profile(
    current_user: AuthenticatedUser = Depends(get_current_user),
):
    """
    Get current user's profile.

    Implements FR-017 (retrieve profile).

    Returns:
        ProfileResponse: User's learning preferences

    Raises:
        404: Profile not found
    """
    profile = await profile_service.get_profile(current_user.user_id)

    if not profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={
                "error": {
                    "code": "PROFILE_NOT_FOUND",
                    "message": "Profile not found for this user",
                }
            },
        )

    return profile


@router.put("/profile", response_model=ProfileResponse)
async def update_profile(
    profile_data: ProfileUpdate,
    current_user: AuthenticatedUser = Depends(get_current_user),
):
    """
    Update current user's profile.

    Implements FR-017 (persist profile answers) and FR-018 (update profile).

    Args:
        profile_data: Profile fields to update (partial update supported)

    Returns:
        ProfileResponse: Updated profile with is_complete flag

    Raises:
        404: Profile not found
    """
    try:
        profile = await profile_service.update_profile(current_user.user_id, profile_data)
        logger.info(f"Profile updated for user {current_user.user_id}, complete: {profile.is_complete}")
        return profile
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={
                "error": {
                    "code": "PROFILE_NOT_FOUND",
                    "message": str(e),
                }
            },
        )


@router.post("/profile", response_model=ProfileResponse, status_code=status.HTTP_201_CREATED)
async def create_profile(
    profile_data: ProfileCreate,
    current_user: AuthenticatedUser = Depends(get_current_user),
):
    """
    Create a new profile for the current user.

    This endpoint is typically called after signup to complete the profile wizard.

    Args:
        profile_data: Initial profile data

    Returns:
        ProfileResponse: Created profile

    Raises:
        409: Profile already exists
    """
    try:
        profile = await profile_service.create_profile(current_user.user_id, profile_data)
        logger.info(f"Profile created for user {current_user.user_id}, complete: {profile.is_complete}")
        return profile
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail={
                "error": {
                    "code": "PROFILE_EXISTS",
                    "message": str(e),
                }
            },
        )


@router.post("/profile/skip", response_model=ProfileResponse)
async def skip_profile_wizard(
    current_user: AuthenticatedUser = Depends(get_current_user),
):
    """
    Skip profile wizard by creating an empty profile.

    Implements FR-016 (allow users to skip profile wizard).
    Users can complete their profile later via settings.

    Returns:
        ProfileResponse: Empty profile with is_complete=False
    """
    profile = await profile_service.skip_profile(current_user.user_id)
    logger.info(f"Profile wizard skipped for user {current_user.user_id}")
    return profile


@router.get("/sessions")
async def get_active_sessions(
    current_user: AuthenticatedUser = Depends(get_current_user),
):
    """
    Get all active sessions for the current user.

    Implements FR-014 (support concurrent sessions on multiple devices).

    Returns:
        List of active sessions with device info, IP, and timestamps

    Example response:
        [
            {
                "session_id": "uuid",
                "device": "Chrome on Windows",
                "ip_address": "192.168.1.1",
                "created_at": "2025-12-20T10:00:00",
                "expires_at": "2026-01-19T10:00:00"
            }
        ]
    """
    sessions = await auth_service.get_active_sessions(current_user.user_id)
    logger.info(f"Retrieved {len(sessions)} active sessions for user {current_user.user_id}")
    return {"sessions": sessions}


@router.delete("/sessions/{session_id}")
async def revoke_session(
    session_id: UUID,
    current_user: AuthenticatedUser = Depends(get_current_user),
):
    """
    Revoke a specific session for the current user.

    Implements FR-014 (support concurrent sessions on multiple devices).
    Allows users to log out from specific devices.

    Args:
        session_id: UUID of the session to revoke

    Returns:
        Success message

    Raises:
        404: Session not found or already revoked
        403: Session belongs to a different user
    """
    success = await auth_service.revoke_session(current_user.user_id, session_id)

    if not success:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={
                "error": {
                    "code": "SESSION_NOT_FOUND",
                    "message": "Session not found or already revoked",
                }
            },
        )

    logger.info(f"Session {session_id} revoked for user {current_user.user_id}")
    return {"message": "Session revoked successfully", "session_id": str(session_id)}
