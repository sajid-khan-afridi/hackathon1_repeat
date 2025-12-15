"""Rate limiting middleware for the RAG chatbot API."""

import logging
import time
from typing import Callable

from fastapi import Request, Response
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse

from app.config import get_settings
from app.services.rate_limiter import (
    get_user_identifier,
    is_authenticated,
    check_rate_limit,
    format_retry_after,
)

logger = logging.getLogger(__name__)


class RateLimitMiddleware(BaseHTTPMiddleware):
    """
    Middleware to add rate limit headers to all responses.

    This middleware adds X-RateLimit-* headers to inform clients
    about their current rate limit status.
    """

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """Process request and add rate limit headers to response."""
        settings = get_settings()

        # Skip rate limit headers for non-API routes
        if not request.url.path.startswith("/api/"):
            return await call_next(request)

        # Get user identifier for logging
        identifier = get_user_identifier(request)
        is_auth = is_authenticated(request)

        # Determine rate limit based on user type
        if is_auth:
            limit = settings.rate_limit_authenticated
        else:
            limit = settings.rate_limit_anonymous

        # Store rate limit info in request state for use by endpoints
        request.state.rate_limit_limit = limit
        request.state.rate_limit_identifier = identifier
        request.state.rate_limit_authenticated = is_auth

        # Process the request
        response = await call_next(request)

        # Add rate limit headers to response
        # Note: The actual remaining count would be tracked by SlowAPI
        # These are informational headers
        response.headers["X-RateLimit-Limit"] = str(limit)
        response.headers["X-RateLimit-Policy"] = f"{limit};w=3600"  # 1 hour window

        return response


def create_rate_limit_response(limit: int, reset_after: int, identifier: str) -> JSONResponse:
    """
    Create a 429 Too Many Requests response.

    Args:
        limit: The rate limit that was exceeded
        reset_after: Seconds until the limit resets
        identifier: The user identifier that was rate limited

    Returns:
        JSONResponse with 429 status and appropriate headers
    """
    retry_after_human = format_retry_after(reset_after)

    response = JSONResponse(
        status_code=429,
        content={
            "error": "Rate limit exceeded",
            "code": "RATE_LIMIT_EXCEEDED",
            "limit": limit,
            "resetAfter": reset_after,
            "retryAfter": retry_after_human,
            "message": f"You have exceeded the rate limit of {limit} requests per hour. "
            f"Please try again in {retry_after_human}.",
        },
    )

    # Add required headers
    response.headers["Retry-After"] = str(reset_after)
    response.headers["X-RateLimit-Limit"] = str(limit)
    response.headers["X-RateLimit-Remaining"] = "0"
    response.headers["X-RateLimit-Reset"] = str(int(time.time()) + reset_after)

    logger.warning(
        f"Rate limit exceeded for {identifier}: {limit}/hour, " f"reset in {retry_after_human}"
    )

    return response
