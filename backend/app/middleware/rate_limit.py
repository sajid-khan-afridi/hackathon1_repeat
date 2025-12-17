"""
Rate limiting middleware for user identification.

Extracts user_id from authentication headers and stores it in request.state
for use by the rate limiter.
"""

import logging
from fastapi import Request
from starlette.middleware.base import BaseHTTPMiddleware
from typing import Callable, Optional

logger = logging.getLogger(__name__)


class UserIdentificationMiddleware(BaseHTTPMiddleware):
    """
    Middleware to extract user identification for rate limiting.

    Checks for authentication tokens and sets request.state.user_id
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

        # Check for authentication token in headers
        auth_header = request.headers.get("Authorization", "")

        if auth_header.startswith("Bearer "):
            token = auth_header.replace("Bearer ", "")

            # Extract user_id from token
            # Note: In production, this should validate the JWT token
            # and extract the user_id from the token payload
            user_id = self._extract_user_id_from_token(token)

            if user_id:
                request.state.user_id = user_id
                logger.debug(f"Authenticated user identified: {user_id}")

        # Continue processing request
        response = await call_next(request)

        return response

    def _extract_user_id_from_token(self, token: str) -> Optional[str]:
        """
        Extract user_id from JWT token.

        In production, this should:
        1. Validate token signature
        2. Check token expiration
        3. Extract user_id from payload

        For now, returns None (anonymous user) as authentication is not implemented.

        Args:
            token: JWT token string

        Returns:
            User ID if token is valid, None otherwise
        """
        # TODO: Implement JWT token validation when authentication is added
        # For now, treat all users as anonymous
        return None
