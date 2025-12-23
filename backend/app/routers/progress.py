"""
Chapter progress tracking API router for Phase 4B.
Endpoints for tracking started/completed/bookmarked chapters.
"""

from fastapi import APIRouter, HTTPException, status, Depends, Query, Request
from typing import Dict, List, Optional, Tuple
from datetime import datetime, timedelta, timezone
from collections import defaultdict
import logging
import asyncpg
import re

from app.config import settings
from app.middleware.auth import AuthenticatedUser, get_current_user
from app.services.progress_tracking_service import ProgressTrackingService
from app.services.recommendation_service import get_recommendation_service
from app.models.chapter_progress import (
    ChapterProgressResponse,
    ChapterProgressMarkStarted,
    ChapterProgressMarkCompleted,
    ChapterProgressToggleBookmark,
    ProgressStatus,
)

# Initialize router
router = APIRouter(
    prefix="/api/v1/progress",
    tags=["progress"],
)

logger = logging.getLogger(__name__)

# ============================================
# Rate Limiting
# ============================================

# In-memory rate limit store: {identifier: [(timestamp, count), ...]}
rate_limit_store: Dict[str, list[Tuple[datetime, int]]] = defaultdict(list)

# Rate limit configuration for progress endpoints
PROGRESS_RATE_LIMIT_AUTHENTICATED = 100  # 100 requests per hour for authenticated users
PROGRESS_RATE_LIMIT_WINDOW_HOURS = 1  # 1 hour window

# Chapter ID validation pattern (alphanumeric + hyphens + slashes)
CHAPTER_ID_PATTERN = re.compile(r'^[a-zA-Z0-9\-\/]+$')


def validate_chapter_id(chapter_id: str) -> None:
    """
    Validate chapter_id format (alphanumeric + hyphens + slashes only).

    Args:
        chapter_id: Chapter ID to validate

    Raises:
        HTTPException 400: If chapter_id format is invalid
    """
    if not chapter_id or not CHAPTER_ID_PATTERN.match(chapter_id):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid chapter ID format: '{chapter_id}'. Expected alphanumeric + hyphens + slashes only"
        )


def check_rate_limit(request: Request, user_id: str) -> None:
    """
    Check if the request exceeds rate limits for progress endpoints.

    Args:
        request: FastAPI Request object
        user_id: Authenticated user ID

    Raises:
        HTTPException 429: If rate limit is exceeded
    """
    identifier = f"user:{user_id}"
    limit = PROGRESS_RATE_LIMIT_AUTHENTICATED

    # Clean up old entries (older than window)
    now = datetime.now(timezone.utc)
    cutoff = now - timedelta(hours=PROGRESS_RATE_LIMIT_WINDOW_HOURS)
    rate_limit_store[identifier] = [
        (ts, count) for ts, count in rate_limit_store[identifier] if ts > cutoff
    ]

    # Count requests in the window
    used = sum(count for ts, count in rate_limit_store[identifier])

    if used >= limit:
        # Rate limit exceeded
        logger.warning(f"Rate limit exceeded for user {user_id}: {used}/{limit} requests")
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail=f"Rate limit exceeded. Maximum {limit} requests per hour allowed.",
            headers={
                "X-RateLimit-Limit": str(limit),
                "X-RateLimit-Remaining": "0",
                "X-RateLimit-Reset": str(int((now + timedelta(hours=PROGRESS_RATE_LIMIT_WINDOW_HOURS)).timestamp())),
            }
        )

    # Record this request
    rate_limit_store[identifier].append((now, 1))

    # Add rate limit headers to response (will be added by middleware)
    remaining = limit - used - 1
    logger.debug(f"Rate limit check passed for user {user_id}: {used + 1}/{limit} requests")


@router.get("/health")
async def health_check() -> Dict[str, str]:
    """
    Health check endpoint for progress tracking service.

    Returns:
        Dict with status "healthy"
    """
    return {"status": "healthy", "service": "progress-tracking"}


@router.post("/start", response_model=ChapterProgressResponse)
async def mark_chapter_started(
    body: ChapterProgressMarkStarted,
    http_request: Request,
    current_user: AuthenticatedUser = Depends(get_current_user),
):
    """
    Mark a chapter as started for the authenticated user.

    Args:
        body: ChapterProgressMarkStarted with chapter_id
        http_request: FastAPI Request object for rate limiting

    Returns:
        ChapterProgressResponse with progress record

    Raises:
        400: Invalid chapter_id format
        429: Rate limit exceeded
        500: Database error
    """
    # Check rate limit (100 requests/hour for authenticated users)
    check_rate_limit(http_request, str(current_user.user_id))

    # Validate chapter_id format
    validate_chapter_id(body.chapter_id)

    conn = await asyncpg.connect(settings.database_url)
    try:
        progress = await ProgressTrackingService.mark_started(
            current_user.user_id, body.chapter_id, conn
        )

        logger.info(
            f"User {current_user.user_id} started chapter '{body.chapter_id}'"
        )

        return progress

    except asyncpg.PostgresError as e:
        logger.error(
            f"Database error marking chapter started for user {current_user.user_id}: {str(e)}"
        )
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Database error during progress tracking",
        )
    finally:
        await conn.close()


@router.post("/complete", response_model=ChapterProgressResponse)
async def mark_chapter_completed(
    body: ChapterProgressMarkCompleted,
    http_request: Request,
    current_user: AuthenticatedUser = Depends(get_current_user),
):
    """
    Mark a chapter as completed for the authenticated user.

    Args:
        body: ChapterProgressMarkCompleted with chapter_id
        http_request: FastAPI Request object for rate limiting

    Returns:
        ChapterProgressResponse with updated progress record

    Raises:
        400: Invalid chapter_id format
        429: Rate limit exceeded
        500: Database error
    """
    # Check rate limit (100 requests/hour for authenticated users)
    check_rate_limit(http_request, str(current_user.user_id))

    # Validate chapter_id format
    validate_chapter_id(body.chapter_id)

    conn = await asyncpg.connect(settings.database_url)
    try:
        progress = await ProgressTrackingService.mark_completed(
            current_user.user_id, body.chapter_id, conn
        )

        logger.info(
            f"User {current_user.user_id} completed chapter '{body.chapter_id}'"
        )

        # T053: Invalidate recommendation cache when chapter is completed
        try:
            recommendation_service = get_recommendation_service()
            recommendation_service.invalidate_cache(str(current_user.user_id))
            logger.debug(f"Invalidated recommendation cache for user {current_user.user_id}")
        except Exception as cache_error:
            # Log error but don't fail the request if cache invalidation fails
            logger.warning(
                f"Failed to invalidate recommendation cache for user {current_user.user_id}: {cache_error}"
            )

        return progress

    except asyncpg.PostgresError as e:
        logger.error(
            f"Database error marking chapter completed for user {current_user.user_id}: {str(e)}"
        )
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Database error during progress tracking",
        )
    finally:
        await conn.close()


@router.post("/bookmark", response_model=ChapterProgressResponse)
async def toggle_chapter_bookmark(
    body: ChapterProgressToggleBookmark,
    http_request: Request,
    current_user: AuthenticatedUser = Depends(get_current_user),
):
    """
    Toggle bookmark status for a chapter.

    Args:
        body: ChapterProgressToggleBookmark with chapter_id
        http_request: FastAPI Request object for rate limiting

    Returns:
        ChapterProgressResponse with updated bookmark status

    Raises:
        400: Invalid chapter_id format
        429: Rate limit exceeded
        500: Database error
    """
    # Check rate limit (100 requests/hour for authenticated users)
    check_rate_limit(http_request, str(current_user.user_id))

    # Validate chapter_id format
    validate_chapter_id(body.chapter_id)

    conn = await asyncpg.connect(settings.database_url)
    try:
        progress = await ProgressTrackingService.toggle_bookmark(
            current_user.user_id, body.chapter_id, conn
        )

        logger.info(
            f"User {current_user.user_id} toggled bookmark for chapter '{body.chapter_id}': "
            f"is_bookmarked={progress.is_bookmarked}"
        )

        return progress

    except asyncpg.PostgresError as e:
        logger.error(
            f"Database error toggling bookmark for user {current_user.user_id}: {str(e)}"
        )
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Database error during bookmark toggle",
        )
    finally:
        await conn.close()


@router.get("", response_model=List[ChapterProgressResponse])
async def get_user_progress(
    current_user: AuthenticatedUser = Depends(get_current_user),
    status_filter: Optional[ProgressStatus] = Query(None, alias="status", description="Filter by status: 'started' or 'completed'"),
    bookmarked_only: bool = Query(False, description="Only return bookmarked chapters"),
):
    """
    Get all progress records for the authenticated user with optional filtering.

    Args:
        status_filter: Optional filter by status ('started' or 'completed')
        bookmarked_only: If True, only return bookmarked chapters

    Returns:
        List of ChapterProgressResponse records

    Raises:
        500: Database error
    """
    conn = await asyncpg.connect(settings.database_url)
    try:
        progress_list = await ProgressTrackingService.get_user_progress(
            current_user.user_id, conn, status_filter, bookmarked_only
        )

        logger.info(
            f"Retrieved {len(progress_list)} progress records for user {current_user.user_id} "
            f"(status={status_filter}, bookmarked_only={bookmarked_only})"
        )

        return progress_list

    except asyncpg.PostgresError as e:
        logger.error(
            f"Database error fetching progress for user {current_user.user_id}: {str(e)}"
        )
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Database error during progress retrieval",
        )
    finally:
        await conn.close()
