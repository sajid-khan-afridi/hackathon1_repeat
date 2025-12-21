"""
Rate limiting middleware for user identification.

Extracts user_id from JWT cookies and stores it in request.state
for use by the rate limiter.
"""

import logging
from fastapi import Request
from starlette.middleware.base import BaseHTTPMiddleware
from typing import Callable, Optional
from uuid import UUID

from app.services.jwt_service import jwt_service

logger = logging.getLogger(__name__)


class UserIdentificationMiddleware(BaseHTTPMiddleware):
    """
    Middleware to extract user identification for rate limiting.

    Checks for JWT authentication cookies and sets request.state.user_id
    if the user is authenticated. Falls back to IP-based identification
    for anonymous users.
    """

    async def dispatch(self, request: Request, call_next: Callable):
        """
        Process request to extract user identifier.

        Args:
            request: FastAPI Request object
            call_next: Next middleware/endpoint in chain

        Returns:
            Response from next middleware/endpoint
        """
        # Initialize user_id as None (anonymous user)
        request.state.user_id = None

        # Check for access token in cookies
        access_token = jwt_service.get_access_token_from_request(request)

        if access_token:
            # Extract user_id from token
            user_id = self._extract_user_id_from_token(access_token)

            if user_id:
                request.state.user_id = user_id
                logger.debug(f"Authenticated user identified: {user_id}")

        # Continue processing request
        response = await call_next(request)

        return response

    def _extract_user_id_from_token(self, token: str) -> Optional[UUID]:
        """
        Extract user_id from JWT token.

        Validates token signature, checks expiration, and extracts user_id from payload.

        Args:
            token: JWT token string

        Returns:
            User ID (UUID) if token is valid, None otherwise
        """
        try:
            payload = jwt_service.verify_token(token, expected_type="access")
            if payload:
                user_id_str = payload.get("sub")
                if user_id_str:
                    return UUID(user_id_str)
        except Exception as e:
            logger.debug(f"Failed to extract user_id from token: {e}")

        return None
