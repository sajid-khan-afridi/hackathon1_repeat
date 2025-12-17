"""
Rate limiting service using SlowAPI with token bucket algorithm.

Implements rate limiting for both anonymous and authenticated users:
- Anonymous users (identified by IP): 10 queries/hour
- Authenticated users (identified by user_id): 50 queries/hour

Uses SlowAPI's built-in token bucket algorithm for smooth rate limiting.
"""

import logging
from slowapi import Limiter
from slowapi.util import get_remote_address
from fastapi import Request
from typing import Optional

logger = logging.getLogger(__name__)

# Rate limit configuration
ANONYMOUS_RATE_LIMIT = "10/hour"  # 10 queries per hour for anonymous users
AUTHENTICATED_RATE_LIMIT = "50/hour"  # 50 queries per hour for authenticated users


def get_user_identifier(request: Request) -> str:
    """
    Extract user identifier from request for rate limiting.

    Priority:
    1. user_id from authentication (if available)
    2. IP address (for anonymous users)

    Args:
        request: FastAPI Request object

    Returns:
        User identifier string (user_id or IP address)
    """
    # Check if user is authenticated (user_id in request state)
    user_id = getattr(request.state, "user_id", None)

    if user_id:
        logger.debug(f"Rate limiting for authenticated user: {user_id}")
        return f"user:{user_id}"

    # Fallback to IP address for anonymous users
    ip_address = get_remote_address(request)
    logger.debug(f"Rate limiting for anonymous IP: {ip_address}")
    return f"ip:{ip_address}"


def get_rate_limit_for_request(request: Request) -> str:
    """
    Determine rate limit based on user type (anonymous vs authenticated).

    Args:
        request: FastAPI Request object

    Returns:
        Rate limit string (e.g., "10/hour" or "50/hour")
    """
    user_id = getattr(request.state, "user_id", None)

    if user_id:
        return AUTHENTICATED_RATE_LIMIT

    return ANONYMOUS_RATE_LIMIT


# Initialize SlowAPI limiter with custom key function
limiter = Limiter(
    key_func=get_user_identifier,
    default_limits=[],  # No default limits; we'll apply per-endpoint
    storage_uri="memory://",  # In-memory storage (use Redis in production)
    headers_enabled=True,  # Add rate limit headers to responses
)
