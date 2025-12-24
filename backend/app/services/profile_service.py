"""
Profile service for user learning preference management.
Handles database operations for user_profiles table.
"""

import logging
from typing import Optional
from uuid import UUID

import asyncpg

from app.config import settings
from app.models.profile import ProfileCreate, ProfileUpdate, ProfileResponse, ProfileInDB

logger = logging.getLogger(__name__)


class ProfileService:
    """Service for user profile database operations."""

    async def _get_connection(self) -> asyncpg.Connection:
        """Get a database connection."""
        return await asyncpg.connect(settings.database_url)

    async def get_profile(self, user_id: UUID) -> Optional[ProfileResponse]:
        """
        Get user's profile.

        Args:
            user_id: User's UUID

        Returns:
            ProfileResponse if found, None otherwise
        """
        conn = await self._get_connection()
        try:
            row = await conn.fetchrow(
                """
                SELECT id, user_id, experience_level, ros_familiarity, hardware_access,
                       learning_goal, preferred_language, is_complete, created_at, updated_at
                FROM user_profiles
                WHERE user_id = $1
                """,
                user_id,
            )
            if row:
                return ProfileResponse(**dict(row))
            return None
        finally:
            await conn.close()

    async def create_profile(self, user_id: UUID, profile_data: ProfileCreate) -> ProfileResponse:
        """
        Create a new profile for a user.

        Args:
            user_id: User's UUID
            profile_data: Profile data to create

        Returns:
            Created ProfileResponse

        Raises:
            ValueError: If profile already exists for user
        """
        conn = await self._get_connection()
        try:
            row = await conn.fetchrow(
                """
                INSERT INTO user_profiles (
                    user_id, experience_level, ros_familiarity,
                    hardware_access, learning_goal, preferred_language
                )
                VALUES ($1, $2, $3, $4, $5, $6)
                RETURNING id, user_id, experience_level, ros_familiarity, hardware_access,
                          learning_goal, preferred_language, is_complete, created_at, updated_at
                """,
                user_id,
                profile_data.experience_level,
                profile_data.ros_familiarity,
                profile_data.hardware_access,
                profile_data.learning_goal,
                profile_data.preferred_language,
            )
            profile = ProfileResponse(**dict(row))
            logger.info(f"Created profile for user: {user_id}, complete: {profile.is_complete}")
            return profile
        except asyncpg.UniqueViolationError:
            raise ValueError("Profile already exists for this user")
        finally:
            await conn.close()

    async def update_profile(self, user_id: UUID, profile_data: ProfileUpdate) -> ProfileResponse:
        """
        Update user's profile.
        Implements FR-017 (persist profile answers) and FR-018 (update profile).

        Args:
            user_id: User's UUID
            profile_data: Profile data to update (only provided fields are updated)

        Returns:
            Updated ProfileResponse

        Raises:
            ValueError: If profile doesn't exist
        """
        # Build dynamic UPDATE query based on provided fields
        update_fields = []
        values = []
        param_idx = 1

        for field, value in profile_data.model_dump(exclude_unset=True, by_alias=False).items():
            if value is not None:
                update_fields.append(f"{field} = ${param_idx}")
                values.append(value)
                param_idx += 1

        if not update_fields:
            # If no fields to update, just fetch current profile
            existing = await self.get_profile(user_id)
            if not existing:
                raise ValueError("Profile not found")
            return existing

        values.append(user_id)
        query = f"""
            UPDATE user_profiles
            SET {', '.join(update_fields)}
            WHERE user_id = ${param_idx}
            RETURNING id, user_id, experience_level, ros_familiarity, hardware_access,
                      learning_goal, preferred_language, is_complete, created_at, updated_at
        """

        conn = await self._get_connection()
        try:
            row = await conn.fetchrow(query, *values)
            if not row:
                raise ValueError("Profile not found")

            profile = ProfileResponse(**dict(row))
            logger.info(f"Updated profile for user: {user_id}, complete: {profile.is_complete}")

            # T054: Invalidate recommendation cache when profile is updated
            # Profile changes (skill level, learning goals, hardware) affect recommendations
            try:
                from app.services.recommendation_service import get_recommendation_service
                recommendation_service = get_recommendation_service()
                recommendation_service.invalidate_cache(str(user_id))
                logger.debug(f"Invalidated recommendation cache for user {user_id} after profile update")
            except Exception as cache_error:
                # Log error but don't fail the request if cache invalidation fails
                logger.warning(
                    f"Failed to invalidate recommendation cache for user {user_id}: {cache_error}"
                )

            return profile
        finally:
            await conn.close()

    async def skip_profile(self, user_id: UUID) -> ProfileResponse:
        """
        Mark profile wizard as skipped by creating an empty profile.
        Implements FR-016 (allow users to skip profile wizard).

        Args:
            user_id: User's UUID

        Returns:
            Created empty ProfileResponse with is_complete=False
        """
        conn = await self._get_connection()
        try:
            # Try to insert empty profile or return existing
            row = await conn.fetchrow(
                """
                INSERT INTO user_profiles (user_id)
                VALUES ($1)
                ON CONFLICT (user_id) DO UPDATE
                SET updated_at = NOW()
                RETURNING id, user_id, experience_level, ros_familiarity, hardware_access,
                          learning_goal, preferred_language, is_complete, created_at, updated_at
                """,
                user_id,
            )
            profile = ProfileResponse(**dict(row))
            logger.info(f"Profile wizard skipped for user: {user_id}")
            return profile
        finally:
            await conn.close()

    async def is_profile_complete(self, user_id: UUID) -> bool:
        """
        Check if user's profile is complete.

        Args:
            user_id: User's UUID

        Returns:
            True if profile is complete, False otherwise
        """
        conn = await self._get_connection()
        try:
            row = await conn.fetchrow(
                """
                SELECT is_complete
                FROM user_profiles
                WHERE user_id = $1
                """,
                user_id,
            )
            return row["is_complete"] if row else False
        finally:
            await conn.close()


# Global profile service instance
profile_service = ProfileService()
