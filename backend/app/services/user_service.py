"""
User service for user CRUD operations.
Handles database operations for users table.
"""

import logging
from datetime import datetime, timedelta, timezone
from typing import Optional
from uuid import UUID

import asyncpg

from app.config import settings
from app.models.user import UserCreate, UserInDB, UserResponse
from app.models.profile import ProfileResponse
from app.services.password_service import password_service

logger = logging.getLogger(__name__)

# Account lockout configuration
MAX_FAILED_ATTEMPTS = 10
LOCKOUT_DURATION_MINUTES = 15


class UserService:
    """Service for user database operations."""

    async def _get_connection(self) -> asyncpg.Connection:
        """Get a database connection."""
        return await asyncpg.connect(settings.database_url)

    async def get_user_by_id(self, user_id: UUID) -> Optional[UserInDB]:
        """
        Get a user by their ID.

        Args:
            user_id: User's UUID

        Returns:
            UserInDB if found, None otherwise
        """
        conn = await self._get_connection()
        try:
            row = await conn.fetchrow(
                """
                SELECT id, email, password_hash, google_id, github_id, created_at, updated_at,
                       last_login_at, is_active, failed_login_attempts, locked_until
                FROM users
                WHERE id = $1
                """,
                user_id,
            )
            if row:
                return UserInDB(**dict(row))
            return None
        finally:
            await conn.close()

    async def get_user_by_email(self, email: str) -> Optional[UserInDB]:
        """
        Get a user by their email address.

        Args:
            email: User's email

        Returns:
            UserInDB if found, None otherwise
        """
        conn = await self._get_connection()
        try:
            row = await conn.fetchrow(
                """
                SELECT id, email, password_hash, google_id, github_id, created_at, updated_at,
                       last_login_at, is_active, failed_login_attempts, locked_until
                FROM users
                WHERE email = $1
                """,
                email.lower(),
            )
            if row:
                return UserInDB(**dict(row))
            return None
        finally:
            await conn.close()

    async def get_user_by_google_id(self, google_id: str) -> Optional[UserInDB]:
        """
        Get a user by their Google OAuth ID.

        Args:
            google_id: Google subject ID

        Returns:
            UserInDB if found, None otherwise
        """
        conn = await self._get_connection()
        try:
            row = await conn.fetchrow(
                """
                SELECT id, email, password_hash, google_id, github_id, created_at, updated_at,
                       last_login_at, is_active, failed_login_attempts, locked_until
                FROM users
                WHERE google_id = $1
                """,
                google_id,
            )
            if row:
                return UserInDB(**dict(row))
            return None
        finally:
            await conn.close()

    async def get_user_by_github_id(self, github_id: str) -> Optional[UserInDB]:
        """
        Get a user by their GitHub OAuth ID.

        Args:
            github_id: GitHub user ID

        Returns:
            UserInDB if found, None otherwise
        """
        conn = await self._get_connection()
        try:
            row = await conn.fetchrow(
                """
                SELECT id, email, password_hash, google_id, github_id, created_at, updated_at,
                       last_login_at, is_active, failed_login_attempts, locked_until
                FROM users
                WHERE github_id = $1
                """,
                github_id,
            )
            if row:
                return UserInDB(**dict(row))
            return None
        finally:
            await conn.close()


    async def link_google_account(self, user_id: UUID, google_id: str) -> None:
        """
        Link a Google account to an existing user.
        Implements FR-008 (auto-link accounts with matching email).

        Args:
            user_id: User's UUID
            google_id: Google subject ID to link
        """
        conn = await self._get_connection()
        try:
            await conn.execute(
                """
                UPDATE users
                SET google_id = $2, updated_at = NOW()
                WHERE id = $1
                """,
                user_id,
                google_id,
            )
            logger.info(f"Linked Google account to user: {user_id}")
        finally:
            await conn.close()

    async def link_github_account(self, user_id: UUID, github_id: str) -> None:
        """
        Link a GitHub account to an existing user.
        Implements FR-008 (auto-link accounts with matching email).

        Args:
            user_id: User's UUID
            github_id: GitHub user ID to link
        """
        conn = await self._get_connection()
        try:
            await conn.execute(
                """
                UPDATE users
                SET github_id = $2, updated_at = NOW()
                WHERE id = $1
                """,
                user_id,
                github_id,
            )
            logger.info(f"Linked GitHub account to user: {user_id}")
        finally:
            await conn.close()

    async def create_user_from_google(self, email: str, google_id: str) -> UserInDB:
        """
        Create a new user from Google OAuth.

        Args:
            email: User's email from Google
            google_id: Google subject ID

        Returns:
            Created UserInDB

        Raises:
            ValueError: If email or google_id already exists
        """
        conn = await self._get_connection()
        try:
            row = await conn.fetchrow(
                """
                INSERT INTO users (email, google_id)
                VALUES ($1, $2)
                RETURNING id, email, password_hash, google_id, github_id, created_at, updated_at,
                          last_login_at, is_active, failed_login_attempts, locked_until
                """,
                email.lower(),
                google_id,
            )
            user = UserInDB(**dict(row))
            logger.info(f"Created user from Google OAuth: {user.id}")
            return user
        except asyncpg.UniqueViolationError as e:
            if "email" in str(e):
                raise ValueError("Email already registered")
            elif "google_id" in str(e):
                raise ValueError("Google account already linked")
            raise
        finally:
            await conn.close()

    async def create_user_from_github(self, email: str, github_id: str) -> UserInDB:
        """
        Create a new user from GitHub OAuth.

        Args:
            email: User's email from GitHub
            github_id: GitHub user ID

        Returns:
            Created UserInDB

        Raises:
            ValueError: If email or github_id already exists
        """
        conn = await self._get_connection()
        try:
            row = await conn.fetchrow(
                """
                INSERT INTO users (email, github_id)
                VALUES ($1, $2)
                RETURNING id, email, password_hash, google_id, github_id, created_at, updated_at,
                          last_login_at, is_active, failed_login_attempts, locked_until
                """,
                email.lower(),
                github_id,
            )
            user = UserInDB(**dict(row))
            logger.info(f"Created user from GitHub OAuth: {user.id}")
            return user
        except asyncpg.UniqueViolationError as e:
            if "email" in str(e):
                raise ValueError("Email already registered")
            elif "github_id" in str(e):
                raise ValueError("GitHub account already linked")
            raise
        finally:
            await conn.close()
    async def create_user(self, user_data: UserCreate) -> UserInDB:
        """
        Create a new user with email/password.

        Args:
            user_data: User registration data

        Returns:
            Created UserInDB

        Raises:
            ValueError: If email already exists
        """
        # Hash password
        password_hash = password_service.hash_password(user_data.password)

        conn = await self._get_connection()
        try:
            row = await conn.fetchrow(
                """
                INSERT INTO users (email, password_hash)
                VALUES ($1, $2)
                RETURNING id, email, password_hash, google_id, github_id, created_at, updated_at,
                          last_login_at, is_active, failed_login_attempts, locked_until
                """,
                user_data.email.lower(),
                password_hash,
            )
            user = UserInDB(**dict(row))
            logger.info(f"Created user: {user.id}")
            return user
        except asyncpg.UniqueViolationError:
            raise ValueError("Email already registered")
        finally:
            await conn.close()

    async def create_empty_profile(self, user_id: UUID) -> None:
        """
        Create an empty profile for a new user.

        Args:
            user_id: User's UUID
        """
        conn = await self._get_connection()
        try:
            await conn.execute(
                """
                INSERT INTO user_profiles (user_id)
                VALUES ($1)
                ON CONFLICT (user_id) DO NOTHING
                """,
                user_id,
            )
            logger.debug(f"Created empty profile for user: {user_id}")
        finally:
            await conn.close()

    async def update_last_login(self, user_id: UUID) -> None:
        """
        Update user's last login timestamp.

        Args:
            user_id: User's UUID
        """
        conn = await self._get_connection()
        try:
            await conn.execute(
                """
                UPDATE users
                SET last_login_at = NOW(), failed_login_attempts = 0, locked_until = NULL
                WHERE id = $1
                """,
                user_id,
            )
            logger.debug(f"Updated last login for user: {user_id}")
        finally:
            await conn.close()

    async def increment_failed_attempts(self, user_id: UUID) -> int:
        """
        Increment failed login attempts and potentially lock account.

        Args:
            user_id: User's UUID

        Returns:
            New failed attempt count
        """
        conn = await self._get_connection()
        try:
            row = await conn.fetchrow(
                """
                UPDATE users
                SET failed_login_attempts = failed_login_attempts + 1,
                    locked_until = CASE
                        WHEN failed_login_attempts + 1 >= $2
                        THEN NOW() + INTERVAL '$3 minutes'
                        ELSE locked_until
                    END
                WHERE id = $1
                RETURNING failed_login_attempts
                """,
                user_id,
                MAX_FAILED_ATTEMPTS,
                LOCKOUT_DURATION_MINUTES,
            )
            count = row["failed_login_attempts"] if row else 0
            if count >= MAX_FAILED_ATTEMPTS:
                logger.warning(f"Account locked for user: {user_id}")
            return count
        finally:
            await conn.close()

    async def check_account_lock(self, user_id: UUID) -> Optional[datetime]:
        """
        Check if account is locked.

        Args:
            user_id: User's UUID

        Returns:
            locked_until datetime if locked, None otherwise
        """
        conn = await self._get_connection()
        try:
            row = await conn.fetchrow(
                """
                SELECT locked_until
                FROM users
                WHERE id = $1 AND locked_until > NOW()
                """,
                user_id,
            )
            if row and row["locked_until"]:
                return row["locked_until"]
            return None
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

    async def get_user_profile(self, user_id: UUID) -> Optional[ProfileResponse]:
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

    async def to_response(self, user: UserInDB) -> UserResponse:
        """
        Convert UserInDB to UserResponse with profile_complete flag.

        Args:
            user: UserInDB instance

        Returns:
            UserResponse with computed fields
        """
        profile_complete = await self.is_profile_complete(user.id)
        return UserResponse(
            id=user.id,
            email=user.email,
            created_at=user.created_at,
            is_active=user.is_active,
            has_google=user.google_id is not None,
            has_github=user.github_id is not None,
            profile_complete=profile_complete,
        )


# Global user service instance
user_service = UserService()
