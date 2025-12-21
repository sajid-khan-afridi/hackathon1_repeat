"""
JWT service for token generation and verification.
Implements RS256 signing with RSA keys per research.md.
"""

import logging
from datetime import datetime, timedelta, timezone
from typing import Optional
from uuid import UUID

import jwt
from fastapi import Request, Response

from app.config import settings

logger = logging.getLogger(__name__)

# Cookie configuration
ACCESS_TOKEN_COOKIE = "access_token"
REFRESH_TOKEN_COOKIE = "refresh_token"


class JWTService:
    """Service for JWT token operations with RS256 signing."""

    def __init__(self):
        """Initialize JWT service with configuration."""
        self._private_key: Optional[str] = None
        self._public_key: Optional[str] = None
        self._algorithm = settings.jwt_algorithm
        self._access_token_expire_minutes = settings.jwt_access_token_expire_minutes
        self._refresh_token_expire_days = settings.jwt_refresh_token_expire_days

    @property
    def private_key(self) -> str:
        """Get the private key for signing tokens."""
        if self._private_key is None:
            if not settings.jwt_private_key:
                raise ValueError("JWT_PRIVATE_KEY not configured")
            # Handle escaped newlines in environment variable
            self._private_key = settings.jwt_private_key.replace("\\n", "\n")
        return self._private_key

    @property
    def public_key(self) -> str:
        """Get the public key for verifying tokens."""
        if self._public_key is None:
            if not settings.jwt_public_key:
                raise ValueError("JWT_PUBLIC_KEY not configured")
            # Handle escaped newlines in environment variable
            self._public_key = settings.jwt_public_key.replace("\\n", "\n")
        return self._public_key

    def create_access_token(self, user_id: UUID, email: str) -> str:
        """
        Create a new access token.

        Args:
            user_id: User's UUID
            email: User's email

        Returns:
            Encoded JWT access token
        """
        now = datetime.now(timezone.utc)
        expires = now + timedelta(minutes=self._access_token_expire_minutes)

        payload = {
            "sub": str(user_id),
            "email": email,
            "type": "access",
            "iat": now,
            "exp": expires,
        }

        token = jwt.encode(payload, self.private_key, algorithm=self._algorithm)
        logger.debug(f"Created access token for user {user_id}")
        return token

    def create_refresh_token(self, user_id: UUID, email: str) -> str:
        """
        Create a new refresh token.

        Args:
            user_id: User's UUID
            email: User's email

        Returns:
            Encoded JWT refresh token
        """
        now = datetime.now(timezone.utc)
        expires = now + timedelta(days=self._refresh_token_expire_days)

        payload = {
            "sub": str(user_id),
            "email": email,
            "type": "refresh",
            "iat": now,
            "exp": expires,
        }

        token = jwt.encode(payload, self.private_key, algorithm=self._algorithm)
        logger.debug(f"Created refresh token for user {user_id}")
        return token

    def verify_token(self, token: str, expected_type: str = "access") -> Optional[dict]:
        """
        Verify and decode a JWT token.

        Args:
            token: The JWT token to verify
            expected_type: Expected token type ("access" or "refresh")

        Returns:
            Token payload if valid, None otherwise
        """
        try:
            payload = jwt.decode(
                token, self.public_key, algorithms=[self._algorithm]
            )

            # Verify token type
            if payload.get("type") != expected_type:
                logger.warning(
                    f"Token type mismatch: expected {expected_type}, "
                    f"got {payload.get('type')}"
                )
                return None

            return payload

        except jwt.ExpiredSignatureError:
            logger.debug("Token has expired")
            return None
        except jwt.InvalidTokenError as e:
            logger.warning(f"Invalid token: {e}")
            return None

    def get_token_expiry_seconds(self) -> int:
        """Get access token expiry time in seconds."""
        return self._access_token_expire_minutes * 60

    def set_auth_cookies(
        self,
        response: Response,
        access_token: str,
        refresh_token: str,
    ) -> None:
        """
        Set authentication cookies on response.

        Args:
            response: FastAPI Response object
            access_token: Access token to set
            refresh_token: Refresh token to set
        """
        # Access token cookie (24 hours)
        response.set_cookie(
            key=ACCESS_TOKEN_COOKIE,
            value=access_token,
            httponly=True,
            secure=settings.is_production,  # HTTPS only in production
            samesite="strict",
            max_age=self._access_token_expire_minutes * 60,
            path="/",
        )

        # Refresh token cookie (30 days)
        response.set_cookie(
            key=REFRESH_TOKEN_COOKIE,
            value=refresh_token,
            httponly=True,
            secure=settings.is_production,
            samesite="strict",
            max_age=self._refresh_token_expire_days * 24 * 60 * 60,
            path="/",
        )

        logger.debug("Set auth cookies on response")

    def clear_auth_cookies(self, response: Response) -> None:
        """
        Clear authentication cookies.

        Args:
            response: FastAPI Response object
        """
        response.delete_cookie(
            key=ACCESS_TOKEN_COOKIE,
            path="/",
            httponly=True,
            secure=settings.is_production,
            samesite="strict",
        )
        response.delete_cookie(
            key=REFRESH_TOKEN_COOKIE,
            path="/",
            httponly=True,
            secure=settings.is_production,
            samesite="strict",
        )
        logger.debug("Cleared auth cookies")

    def get_access_token_from_request(self, request: Request) -> Optional[str]:
        """
        Extract access token from request cookies.

        Args:
            request: FastAPI Request object

        Returns:
            Access token if present, None otherwise
        """
        return request.cookies.get(ACCESS_TOKEN_COOKIE)

    def get_refresh_token_from_request(self, request: Request) -> Optional[str]:
        """
        Extract refresh token from request cookies.

        Args:
            request: FastAPI Request object

        Returns:
            Refresh token if present, None otherwise
        """
        return request.cookies.get(REFRESH_TOKEN_COOKIE)


# Global JWT service instance
jwt_service = JWTService()
