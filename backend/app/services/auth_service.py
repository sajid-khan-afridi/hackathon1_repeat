"""
Authentication service for signup, login, logout, and token refresh.
Orchestrates user service, JWT service, and password service.
"""

import hashlib
import logging
import secrets
from datetime import datetime, timedelta, timezone
from typing import Optional, Tuple
from uuid import UUID

import asyncpg

from app.config import settings
from app.models.user import UserCreate, UserLogin, UserInDB, UserResponse
from app.models.profile import ProfileResponse
from app.services.jwt_service import jwt_service
from app.services.password_service import password_service
from app.services.user_service import user_service

logger = logging.getLogger(__name__)


def log_auth_event(
    event_type: str,
    user_id: Optional[UUID] = None,
    email: Optional[str] = None,
    ip_address: Optional[str] = None,
    user_agent: Optional[str] = None,
    success: bool = True,
    error_code: Optional[str] = None,
    additional_data: Optional[dict] = None,
) -> None:
    """
    Log authentication events in structured format (FR-028).

    Args:
        event_type: Type of auth event (signup, login, logout, refresh, etc.)
        user_id: User's UUID if available
        email: User's email if available
        ip_address: Client IP address
        user_agent: Client user agent
        success: Whether the operation succeeded
        error_code: Error code if operation failed
        additional_data: Additional contextual data
    """
    log_data = {
        "event": event_type,
        "success": success,
    }

    if user_id:
        log_data["user_id"] = str(user_id)
    if email:
        log_data["email"] = email
    if ip_address:
        log_data["ip_address"] = ip_address
    if user_agent:
        log_data["user_agent"] = user_agent
    if error_code:
        log_data["error_code"] = error_code
    if additional_data:
        log_data.update(additional_data)

    # Log at appropriate level based on success/event type
    if not success:
        logger.warning(f"Auth event: {event_type}", extra=log_data)
    elif event_type in ("login", "signup", "logout"):
        logger.info(f"Auth event: {event_type}", extra=log_data)
    else:
        logger.debug(f"Auth event: {event_type}", extra=log_data)


class AuthResult:
    """Result of an authentication operation."""

    def __init__(
        self,
        success: bool,
        user: Optional[UserResponse] = None,
        profile: Optional[ProfileResponse] = None,
        access_token: Optional[str] = None,
        refresh_token: Optional[str] = None,
        error_code: Optional[str] = None,
        error_message: Optional[str] = None,
        locked_until: Optional[datetime] = None,
    ):
        self.success = success
        self.user = user
        self.profile = profile
        self.access_token = access_token
        self.refresh_token = refresh_token
        self.error_code = error_code
        self.error_message = error_message
        self.locked_until = locked_until


class AuthService:
    """Service for authentication operations."""

    async def _get_connection(self) -> asyncpg.Connection:
        """Get a database connection."""
        return await asyncpg.connect(settings.database_url)

    def _hash_token(self, token: str) -> str:
        """Hash a refresh token for storage."""
        return hashlib.sha256(token.encode()).hexdigest()

    async def _store_refresh_token(
        self,
        user_id: UUID,
        refresh_token: str,
        user_agent: Optional[str] = None,
        ip_address: Optional[str] = None,
    ) -> None:
        """Store refresh token hash in database."""
        token_hash = self._hash_token(refresh_token)
        expires_at = datetime.now(timezone.utc) + timedelta(
            days=settings.jwt_refresh_token_expire_days
        )

        conn = await self._get_connection()
        try:
            await conn.execute(
                """
                INSERT INTO refresh_tokens (user_id, token_hash, expires_at, user_agent, ip_address)
                VALUES ($1, $2, $3, $4, $5)
                """,
                user_id,
                token_hash,
                expires_at,
                user_agent,
                ip_address,
            )
            logger.debug(f"Stored refresh token for user: {user_id}")
        finally:
            await conn.close()

    async def _verify_refresh_token(self, refresh_token: str) -> Optional[UUID]:
        """
        Verify a refresh token is valid and not revoked.

        Returns user_id if valid, None otherwise.
        """
        token_hash = self._hash_token(refresh_token)

        conn = await self._get_connection()
        try:
            row = await conn.fetchrow(
                """
                SELECT user_id
                FROM refresh_tokens
                WHERE token_hash = $1
                  AND expires_at > NOW()
                  AND revoked_at IS NULL
                """,
                token_hash,
            )
            if row:
                return row["user_id"]
            return None
        finally:
            await conn.close()

    async def _revoke_refresh_token(self, refresh_token: str) -> bool:
        """Revoke a refresh token."""
        token_hash = self._hash_token(refresh_token)

        conn = await self._get_connection()
        try:
            result = await conn.execute(
                """
                UPDATE refresh_tokens
                SET revoked_at = NOW()
                WHERE token_hash = $1 AND revoked_at IS NULL
                """,
                token_hash,
            )
            return result == "UPDATE 1"
        finally:
            await conn.close()

    async def _log_login_attempt(
        self,
        email: str,
        ip_address: str,
        user_agent: Optional[str],
        success: bool,
        failure_reason: Optional[str] = None,
    ) -> None:
        """Log a login attempt for security auditing."""
        conn = await self._get_connection()
        try:
            await conn.execute(
                """
                INSERT INTO login_attempts (email, ip_address, user_agent, success, failure_reason)
                VALUES ($1, $2, $3, $4, $5)
                """,
                email.lower(),
                ip_address,
                user_agent,
                success,
                failure_reason,
            )
        except Exception as e:
            # Don't fail auth on logging errors
            logger.warning(f"Failed to log login attempt: {e}")
        finally:
            await conn.close()

    async def signup(
        self,
        user_data: UserCreate,
        user_agent: Optional[str] = None,
        ip_address: Optional[str] = None,
    ) -> AuthResult:
        """
        Register a new user with email/password.

        Args:
            user_data: User registration data
            user_agent: Client user agent
            ip_address: Client IP address

        Returns:
            AuthResult with user and tokens on success
        """
        try:
            # Check if email already exists
            existing_user = await user_service.get_user_by_email(user_data.email)
            if existing_user:
                return AuthResult(
                    success=False,
                    error_code="EMAIL_EXISTS",
                    error_message="Email already registered",
                )

            # Create user
            user = await user_service.create_user(user_data)

            # Create empty profile
            await user_service.create_empty_profile(user.id)

            # Generate tokens
            access_token = jwt_service.create_access_token(user.id, user.email)
            refresh_token = jwt_service.create_refresh_token(user.id, user.email)

            # Store refresh token
            await self._store_refresh_token(
                user.id, refresh_token, user_agent, ip_address
            )

            # Get user response with profile status
            user_response = await user_service.to_response(user)
            profile = await user_service.get_user_profile(user.id)

            # Structured logging for signup (FR-028)
            log_auth_event(
                event_type="signup",
                user_id=user.id,
                email=user.email,
                ip_address=ip_address,
                user_agent=user_agent,
                success=True,
            )

            return AuthResult(
                success=True,
                user=user_response,
                profile=profile,
                access_token=access_token,
                refresh_token=refresh_token,
            )

        except ValueError as e:
            log_auth_event(
                event_type="signup",
                email=user_data.email,
                ip_address=ip_address,
                user_agent=user_agent,
                success=False,
                error_code="VALIDATION_ERROR",
                additional_data={"error_message": str(e)},
            )
            return AuthResult(
                success=False,
                error_code="VALIDATION_ERROR",
                error_message=str(e),
            )
        except Exception as e:
            log_auth_event(
                event_type="signup",
                email=user_data.email,
                ip_address=ip_address,
                user_agent=user_agent,
                success=False,
                error_code="INTERNAL_ERROR",
                additional_data={"exception": str(e)},
            )
            return AuthResult(
                success=False,
                error_code="INTERNAL_ERROR",
                error_message="An error occurred during signup",
            )

    async def login(
        self,
        login_data: UserLogin,
        user_agent: Optional[str] = None,
        ip_address: str = "unknown",
    ) -> AuthResult:
        """
        Authenticate user with email/password.

        Args:
            login_data: Login credentials
            user_agent: Client user agent
            ip_address: Client IP address

        Returns:
            AuthResult with user and tokens on success
        """
        email = login_data.email.lower()

        # Get user by email
        user = await user_service.get_user_by_email(email)

        if not user:
            await self._log_login_attempt(
                email, ip_address, user_agent, False, "invalid_email"
            )
            log_auth_event(
                event_type="login",
                email=email,
                ip_address=ip_address,
                user_agent=user_agent,
                success=False,
                error_code="INVALID_CREDENTIALS",
                additional_data={"failure_reason": "invalid_email"},
            )
            return AuthResult(
                success=False,
                error_code="INVALID_CREDENTIALS",
                error_message="Invalid email or password",
            )

        # Check if account is locked
        locked_until = await user_service.check_account_lock(user.id)
        if locked_until:
            await self._log_login_attempt(
                email, ip_address, user_agent, False, "account_locked"
            )
            log_auth_event(
                event_type="login",
                user_id=user.id,
                email=email,
                ip_address=ip_address,
                user_agent=user_agent,
                success=False,
                error_code="ACCOUNT_LOCKED",
                additional_data={"locked_until": str(locked_until)},
            )
            return AuthResult(
                success=False,
                error_code="ACCOUNT_LOCKED",
                error_message="Account locked due to too many failed attempts",
                locked_until=locked_until,
            )

        # Check if account is active
        if not user.is_active:
            await self._log_login_attempt(
                email, ip_address, user_agent, False, "account_inactive"
            )
            log_auth_event(
                event_type="login",
                user_id=user.id,
                email=email,
                ip_address=ip_address,
                user_agent=user_agent,
                success=False,
                error_code="ACCOUNT_INACTIVE",
            )
            return AuthResult(
                success=False,
                error_code="ACCOUNT_INACTIVE",
                error_message="Account is inactive",
            )

        # Check password
        if not user.password_hash:
            # User signed up with Google only
            await self._log_login_attempt(
                email, ip_address, user_agent, False, "no_password"
            )
            log_auth_event(
                event_type="login",
                user_id=user.id,
                email=email,
                ip_address=ip_address,
                user_agent=user_agent,
                success=False,
                error_code="NO_PASSWORD",
                additional_data={"failure_reason": "oauth_only_account"},
            )
            return AuthResult(
                success=False,
                error_code="NO_PASSWORD",
                error_message="Please use Google login for this account",
            )

        if not password_service.verify_password(login_data.password, user.password_hash):
            # Increment failed attempts
            await user_service.increment_failed_attempts(user.id)
            await self._log_login_attempt(
                email, ip_address, user_agent, False, "invalid_password"
            )
            log_auth_event(
                event_type="login",
                user_id=user.id,
                email=email,
                ip_address=ip_address,
                user_agent=user_agent,
                success=False,
                error_code="INVALID_CREDENTIALS",
                additional_data={"failure_reason": "invalid_password"},
            )
            return AuthResult(
                success=False,
                error_code="INVALID_CREDENTIALS",
                error_message="Invalid email or password",
            )

        # Success - update last login and generate tokens
        await user_service.update_last_login(user.id)

        access_token = jwt_service.create_access_token(user.id, user.email)
        refresh_token = jwt_service.create_refresh_token(user.id, user.email)

        # Store refresh token
        await self._store_refresh_token(user.id, refresh_token, user_agent, ip_address)

        # Log success
        await self._log_login_attempt(email, ip_address, user_agent, True)

        # Get user response
        user_response = await user_service.to_response(user)
        profile = await user_service.get_user_profile(user.id)

        # Structured logging for successful login (FR-028)
        log_auth_event(
            event_type="login",
            user_id=user.id,
            email=email,
            ip_address=ip_address,
            user_agent=user_agent,
            success=True,
        )

        return AuthResult(
            success=True,
            user=user_response,
            profile=profile,
            access_token=access_token,
            refresh_token=refresh_token,
        )

    async def logout(self, refresh_token: str, user_id: Optional[UUID] = None) -> bool:
        """
        Log out user by revoking refresh token.

        Args:
            refresh_token: Refresh token to revoke
            user_id: User's UUID for logging

        Returns:
            True if successful
        """
        success = await self._revoke_refresh_token(refresh_token)
        if success:
            # Structured logging for logout (FR-028)
            log_auth_event(
                event_type="logout",
                user_id=user_id,
                success=True,
            )
        return success

    async def refresh_tokens(
        self,
        refresh_token: str,
        user_agent: Optional[str] = None,
        ip_address: Optional[str] = None,
    ) -> AuthResult:
        """
        Refresh access token using refresh token.

        Args:
            refresh_token: Valid refresh token
            user_agent: Client user agent
            ip_address: Client IP address

        Returns:
            AuthResult with new tokens on success
        """
        # Verify JWT structure of refresh token
        payload = jwt_service.verify_token(refresh_token, expected_type="refresh")
        if not payload:
            log_auth_event(
                event_type="token_refresh",
                ip_address=ip_address,
                user_agent=user_agent,
                success=False,
                error_code="INVALID_TOKEN",
            )
            return AuthResult(
                success=False,
                error_code="INVALID_TOKEN",
                error_message="Invalid or expired refresh token",
            )

        # Verify token is stored and not revoked
        user_id = await self._verify_refresh_token(refresh_token)
        if not user_id:
            log_auth_event(
                event_type="token_refresh",
                ip_address=ip_address,
                user_agent=user_agent,
                success=False,
                error_code="TOKEN_REVOKED",
            )
            return AuthResult(
                success=False,
                error_code="TOKEN_REVOKED",
                error_message="Refresh token has been revoked",
            )

        # Get user
        user = await user_service.get_user_by_id(user_id)
        if not user or not user.is_active:
            log_auth_event(
                event_type="token_refresh",
                user_id=user_id,
                ip_address=ip_address,
                user_agent=user_agent,
                success=False,
                error_code="USER_NOT_FOUND",
            )
            return AuthResult(
                success=False,
                error_code="USER_NOT_FOUND",
                error_message="User not found or inactive",
            )

        # Generate new access token
        new_access_token = jwt_service.create_access_token(user.id, user.email)

        # Optionally rotate refresh token (for enhanced security)
        # Revoke old token and issue new one
        await self._revoke_refresh_token(refresh_token)
        new_refresh_token = jwt_service.create_refresh_token(user.id, user.email)
        await self._store_refresh_token(user.id, new_refresh_token, user_agent, ip_address)

        user_response = await user_service.to_response(user)

        # Structured logging for token refresh (FR-028)
        log_auth_event(
            event_type="token_refresh",
            user_id=user.id,
            email=user.email,
            ip_address=ip_address,
            user_agent=user_agent,
            success=True,
        )

        return AuthResult(
            success=True,
            user=user_response,
            access_token=new_access_token,
            refresh_token=new_refresh_token,
        )

    async def get_current_user(self, user_id: UUID) -> Optional[Tuple[UserResponse, ProfileResponse]]:
        """
        Get current user info by ID.

        Args:
            user_id: User's UUID

        Returns:
            Tuple of (UserResponse, ProfileResponse) or None
        """
        user = await user_service.get_user_by_id(user_id)
        if not user:
            return None

        user_response = await user_service.to_response(user)
        profile = await user_service.get_user_profile(user_id)

        return user_response, profile

    async def get_active_sessions(self, user_id: UUID) -> list[dict]:
        """
        Get all active sessions for a user.

        Args:
            user_id: User's UUID

        Returns:
            List of active session information (FR-014)
        """
        conn = await self._get_connection()
        try:
            rows = await conn.fetch(
                """
                SELECT id, user_agent, ip_address, created_at, expires_at
                FROM refresh_tokens
                WHERE user_id = $1
                  AND expires_at > NOW()
                  AND revoked_at IS NULL
                ORDER BY created_at DESC
                """,
                user_id,
            )

            sessions = []
            for row in rows:
                # Parse user agent to extract device/browser info
                user_agent = row["user_agent"] or "Unknown"
                sessions.append(
                    {
                        "session_id": str(row["id"]),
                        "device": self._parse_device_from_user_agent(user_agent),
                        "ip_address": row["ip_address"],
                        "created_at": row["created_at"].isoformat(),
                        "expires_at": row["expires_at"].isoformat(),
                    }
                )

            return sessions
        finally:
            await conn.close()

    def _parse_device_from_user_agent(self, user_agent: str) -> str:
        """
        Parse device/browser information from user agent string.

        Args:
            user_agent: User agent string

        Returns:
            Human-readable device description
        """
        ua_lower = user_agent.lower()

        # Detect browser
        if "chrome" in ua_lower and "edg" not in ua_lower:
            browser = "Chrome"
        elif "safari" in ua_lower and "chrome" not in ua_lower:
            browser = "Safari"
        elif "firefox" in ua_lower:
            browser = "Firefox"
        elif "edg" in ua_lower:
            browser = "Edge"
        else:
            browser = "Unknown Browser"

        # Detect OS
        if "windows" in ua_lower:
            os = "Windows"
        elif "mac" in ua_lower:
            os = "macOS"
        elif "linux" in ua_lower:
            os = "Linux"
        elif "android" in ua_lower:
            os = "Android"
        elif "iphone" in ua_lower or "ipad" in ua_lower:
            os = "iOS"
        else:
            os = "Unknown OS"

        return f"{browser} on {os}"

    async def revoke_session(
        self, user_id: UUID, session_id: UUID
    ) -> bool:
        """
        Revoke a specific session for a user.

        Args:
            user_id: User's UUID (for authorization)
            session_id: Session ID to revoke

        Returns:
            True if session was revoked, False if not found or unauthorized
        """
        conn = await self._get_connection()
        try:
            # Verify session belongs to user before revoking
            result = await conn.execute(
                """
                UPDATE refresh_tokens
                SET revoked_at = NOW()
                WHERE id = $1
                  AND user_id = $2
                  AND revoked_at IS NULL
                """,
                session_id,
                user_id,
            )

            if result == "UPDATE 1":
                log_auth_event(
                    event_type="session_revoked",
                    user_id=user_id,
                    success=True,
                    additional_data={"session_id": str(session_id)},
                )
                return True

            return False
        finally:
            await conn.close()


# Global auth service instance
auth_service = AuthService()
