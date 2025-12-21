"""
Authentication middleware for JWT validation.
Provides dependency injection for authenticated routes.
"""

import logging
from typing import Optional
from uuid import UUID

from fastapi import Depends, HTTPException, Request, status

from app.services.jwt_service import jwt_service

logger = logging.getLogger(__name__)


class AuthenticatedUser:
    """Represents an authenticated user from JWT token."""

    def __init__(self, user_id: UUID, email: str):
        self.user_id = user_id
        self.email = email

    def __repr__(self) -> str:
        return f"AuthenticatedUser(user_id={self.user_id}, email={self.email})"


async def get_current_user(request: Request) -> AuthenticatedUser:
    """
    FastAPI dependency to get the current authenticated user.

    Extracts and validates the JWT access token from cookies.
    Raises HTTPException 401 if not authenticated.

    Args:
        request: FastAPI Request object

    Returns:
        AuthenticatedUser with user_id and email

    Raises:
        HTTPException: 401 if not authenticated
    """
    token = jwt_service.get_access_token_from_request(request)

    if not token:
        logger.debug("No access token in request")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={
                "error": {
                    "code": "NOT_AUTHENTICATED",
                    "message": "Not authenticated",
                }
            },
        )

    payload = jwt_service.verify_token(token, expected_type="access")

    if not payload:
        logger.debug("Invalid or expired access token")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={
                "error": {
                    "code": "INVALID_TOKEN",
                    "message": "Invalid or expired token",
                }
            },
        )

    try:
        user_id = UUID(payload["sub"])
        email = payload["email"]
        return AuthenticatedUser(user_id=user_id, email=email)
    except (KeyError, ValueError) as e:
        logger.warning(f"Invalid token payload: {e}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={
                "error": {
                    "code": "INVALID_TOKEN",
                    "message": "Invalid token payload",
                }
            },
        )


async def get_optional_user(request: Request) -> Optional[AuthenticatedUser]:
    """
    FastAPI dependency to optionally get the current user.

    Returns None if not authenticated instead of raising exception.
    Useful for routes that behave differently for authenticated vs anonymous users.

    Args:
        request: FastAPI Request object

    Returns:
        AuthenticatedUser if authenticated, None otherwise
    """
    token = jwt_service.get_access_token_from_request(request)

    if not token:
        return None

    payload = jwt_service.verify_token(token, expected_type="access")

    if not payload:
        return None

    try:
        user_id = UUID(payload["sub"])
        email = payload["email"]
        return AuthenticatedUser(user_id=user_id, email=email)
    except (KeyError, ValueError):
        return None


def require_auth(user: AuthenticatedUser = Depends(get_current_user)) -> AuthenticatedUser:
    """
    Dependency alias for requiring authentication.

    Use this in route dependencies to require authentication:
        @router.get("/protected", dependencies=[Depends(require_auth)])

    Args:
        user: Injected authenticated user

    Returns:
        The authenticated user
    """
    return user
