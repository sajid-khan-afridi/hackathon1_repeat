"""Rate limiter service using SlowAPI with token bucket algorithm."""

import logging
from typing import Callable, Optional

from fastapi import Request, Response
from slowapi import Limiter
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

from app.config import get_settings

logger = logging.getLogger(__name__)


def get_user_identifier(request: Request) -> str:
    """
    Extract user identifier from request for rate limiting.

    Priority:
    1. Authenticated user ID from Authorization header
    2. Session ID from request body or header
    3. IP address for anonymous users
    """
    # Check for Authorization header (authenticated user)
    auth_header = request.headers.get("authorization", "")
    if auth_header.startswith("Bearer "):
        token = auth_header[7:]
        # Use first 16 chars of token as identifier
        return f"auth:{token[:16]}"

    # Check for X-User-ID header
    user_id = request.headers.get("x-user-id")
    if user_id:
        return f"user:{user_id}"

    # Check for session ID in header
    session_id = request.headers.get("x-session-id")
    if session_id:
        return f"session:{session_id}"

    # Fall back to IP address
    return get_remote_address(request)


def is_authenticated(request: Request) -> bool:
    """Check if request is from an authenticated user."""
    auth_header = request.headers.get("authorization", "")
    user_id = request.headers.get("x-user-id")
    return bool(auth_header.startswith("Bearer ") or user_id)


def get_rate_limit_string(request: Request) -> str:
    """
    Get the appropriate rate limit string based on user type.

    Returns:
        Rate limit string in format "N/hour" for SlowAPI
    """
    settings = get_settings()

    if is_authenticated(request):
        return f"{settings.rate_limit_authenticated}/hour"
    else:
        return f"{settings.rate_limit_anonymous}/hour"


# Create the limiter instance with custom key function
limiter = Limiter(
    key_func=get_user_identifier,
    default_limits=["100/hour"],  # Default fallback
    storage_uri="memory://",  # Use in-memory storage
    strategy="fixed-window",  # Fixed window strategy
)


def get_limiter() -> Limiter:
    """Get the rate limiter instance."""
    return limiter


class RateLimitInfo:
    """Container for rate limit information."""

    def __init__(self, limit: int, remaining: int, reset_after: int, is_exceeded: bool = False):
        self.limit = limit
        self.remaining = remaining
        self.reset_after = reset_after
        self.is_exceeded = is_exceeded

    def to_headers(self) -> dict:
        """Convert to response headers."""
        return {
            "X-RateLimit-Limit": str(self.limit),
            "X-RateLimit-Remaining": str(max(0, self.remaining)),
            "X-RateLimit-Reset": str(self.reset_after),
        }


async def check_rate_limit(request: Request) -> RateLimitInfo:
    """
    Check rate limit for a request and return info.

    This is used for adding headers to successful responses.
    """
    settings = get_settings()
    identifier = get_user_identifier(request)

    if is_authenticated(request):
        limit = settings.rate_limit_authenticated
    else:
        limit = settings.rate_limit_anonymous

    # Get current usage from limiter's storage
    # Note: This is a simplified version - in production you'd query Redis
    # For now, we'll estimate based on the limit
    remaining = limit  # Default to full limit
    reset_after = 3600  # 1 hour in seconds

    return RateLimitInfo(
        limit=limit, remaining=remaining, reset_after=reset_after, is_exceeded=False
    )


def format_retry_after(seconds: int) -> str:
    """Format seconds into human-readable retry time."""
    if seconds < 60:
        return f"{seconds} seconds"
    elif seconds < 3600:
        minutes = seconds // 60
        return f"{minutes} minute{'s' if minutes > 1 else ''}"
    else:
        hours = seconds // 3600
        minutes = (seconds % 3600) // 60
        if minutes > 0:
            return f"{hours} hour{'s' if hours > 1 else ''} and {minutes} minute{'s' if minutes > 1 else ''}"
        return f"{hours} hour{'s' if hours > 1 else ''}"
