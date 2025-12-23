"""
Personalization API router for Phase 4B.
Endpoints for skill level classification and chapter recommendations.
"""

from fastapi import APIRouter, HTTPException, status, Depends, Query, Request
from fastapi.responses import JSONResponse
from typing import Dict, Any, List
from datetime import datetime, timedelta, timezone
from collections import defaultdict
from uuid import UUID
import logging
import asyncpg

from app.config import settings
from app.middleware.auth import AuthenticatedUser, get_current_user
from app.services.classification_service import ClassificationService
from app.services.recommendation_service import get_recommendation_service
from app.models.skill_level_classification import SkillLevelClassificationResponse
from app.models.chapter_recommendation import ChapterRecommendationResponse, RecommendationsResponse

# Initialize router
router = APIRouter(
    prefix="/api/v1",
    tags=["personalization"],
)

logger = logging.getLogger(__name__)


# ============================================
# Rate Limiting (T056)
# ============================================

# In-memory rate limit store: {identifier: [(timestamp, count), ...]}
rate_limit_store: Dict[str, list[tuple[datetime, int]]] = defaultdict(list)

# Rate limit configuration for recommendations endpoint
RECOMMENDATIONS_RATE_LIMIT = 50  # 50 requests per hour for authenticated users
RECOMMENDATIONS_RATE_LIMIT_WINDOW_HOURS = 1  # 1 hour window


def check_rate_limit(request: Request, user_id: str) -> None:
    """
    Check if the request exceeds rate limits for recommendations endpoint.

    Args:
        request: FastAPI Request object
        user_id: Authenticated user ID

    Raises:
        HTTPException 429: If rate limit is exceeded
    """
    identifier = f"recommendations:user:{user_id}"
    limit = RECOMMENDATIONS_RATE_LIMIT

    # Clean up old entries (older than window)
    now = datetime.now(timezone.utc)
    cutoff = now - timedelta(hours=RECOMMENDATIONS_RATE_LIMIT_WINDOW_HOURS)
    rate_limit_store[identifier] = [
        (ts, count) for ts, count in rate_limit_store[identifier] if ts > cutoff
    ]

    # Count requests in the window
    used = sum(count for ts, count in rate_limit_store[identifier])

    if used >= limit:
        # Rate limit exceeded
        logger.warning(f"Rate limit exceeded for user {user_id} on recommendations endpoint: {used}/{limit} requests")
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail=f"Rate limit exceeded. Maximum {limit} requests per hour allowed for recommendations.",
            headers={
                "X-RateLimit-Limit": str(limit),
                "X-RateLimit-Remaining": "0",
                "X-RateLimit-Reset": str(int((now + timedelta(hours=RECOMMENDATIONS_RATE_LIMIT_WINDOW_HOURS)).timestamp())),
            }
        )

    # Record this request
    rate_limit_store[identifier].append((now, 1))

    # Log rate limit check passed
    remaining = limit - used - 1
    logger.debug(f"Rate limit check passed for user {user_id} on recommendations: {used + 1}/{limit} requests")


# Error response models
class PersonalizationError:
    """Standard error responses for personalization endpoints."""

    @staticmethod
    def user_not_found(user_id: UUID) -> HTTPException:
        """Return 404 error when user is not found."""
        return HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"User with ID {user_id} not found"
        )

    @staticmethod
    def classification_not_found(user_id: UUID) -> HTTPException:
        """Return 404 error when skill level classification is not found."""
        return HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Skill level classification not found for user {user_id}"
        )

    @staticmethod
    def chapter_not_found(chapter_id: str) -> HTTPException:
        """Return 404 error when chapter is not found."""
        return HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Chapter '{chapter_id}' not found in metadata"
        )

    @staticmethod
    def progress_not_found(user_id: UUID, chapter_id: str) -> HTTPException:
        """Return 404 error when progress record is not found."""
        return HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Progress record not found for user {user_id} and chapter '{chapter_id}'"
        )

    @staticmethod
    def invalid_chapter_id(chapter_id: str) -> HTTPException:
        """Return 400 error for invalid chapter ID format."""
        return HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid chapter ID format: '{chapter_id}'. Expected alphanumeric + hyphens + slashes only"
        )

    @staticmethod
    def database_error(operation: str, error: str) -> HTTPException:
        """Return 500 error for database errors."""
        logger.error(f"Database error during {operation}: {error}")
        return HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Database error during {operation}"
        )

    @staticmethod
    def unauthorized() -> HTTPException:
        """Return 401 error when user is not authenticated."""
        return HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required"
        )

    @staticmethod
    def rate_limit_exceeded() -> HTTPException:
        """Return 429 error when rate limit is exceeded."""
        return HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail="Rate limit exceeded. Please try again later."
        )


@router.get("/health/personalization")
async def health_check() -> Dict[str, str]:
    """
    Health check endpoint for personalization service.

    Returns:
        Dict with status "healthy"
    """
    return {"status": "healthy", "service": "personalization"}


@router.get("/skill-level", response_model=SkillLevelClassificationResponse)
async def get_skill_level(
    current_user: AuthenticatedUser = Depends(get_current_user),
):
    """
    Get user's skill level classification.

    Returns existing classification or computes new one if doesn't exist.

    Returns:
        SkillLevelClassificationResponse with skill level tier

    Raises:
        404: User profile not found or incomplete
        500: Database error
    """
    conn = await asyncpg.connect(settings.database_url)
    try:
        # Try to get existing classification
        classification = await ClassificationService.get_classification(
            current_user.user_id, conn
        )

        if classification:
            logger.info(
                f"Retrieved existing classification for user {current_user.user_id}: "
                f"{classification.skill_level}"
            )
            return classification

        # No classification exists - compute new one
        logger.info(
            f"No classification found for user {current_user.user_id}, computing new one"
        )
        classification = await ClassificationService.classify_user(
            current_user.user_id, conn
        )
        return classification

    except ValueError as e:
        logger.error(f"Classification error for user {current_user.user_id}: {str(e)}")
        raise PersonalizationError.user_not_found(current_user.user_id)
    except asyncpg.PostgresError as e:
        logger.error(
            f"Database error during classification for user {current_user.user_id}: {str(e)}"
        )
        raise PersonalizationError.database_error("skill classification", str(e))
    finally:
        await conn.close()


@router.post("/skill-level", response_model=SkillLevelClassificationResponse)
async def recalculate_skill_level(
    current_user: AuthenticatedUser = Depends(get_current_user),
):
    """
    Force recalculation of user's skill level classification.

    Useful when user updates their profile and wants immediate reclassification.

    Returns:
        SkillLevelClassificationResponse with updated skill level tier

    Raises:
        404: User profile not found or incomplete
        500: Database error
    """
    conn = await asyncpg.connect(settings.database_url)
    try:
        classification = await ClassificationService.recalculate_classification(
            current_user.user_id, conn
        )

        logger.info(
            f"Recalculated classification for user {current_user.user_id}: "
            f"{classification.skill_level}"
        )

        return classification

    except ValueError as e:
        logger.error(
            f"Recalculation error for user {current_user.user_id}: {str(e)}"
        )
        raise PersonalizationError.user_not_found(current_user.user_id)
    except asyncpg.PostgresError as e:
        logger.error(
            f"Database error during recalculation for user {current_user.user_id}: {str(e)}"
        )
        raise PersonalizationError.database_error("skill recalculation", str(e))
    finally:
        await conn.close()


@router.get("/recommendations", response_model=RecommendationsResponse)
async def get_recommendations(
    http_request: Request,
    force_refresh: bool = Query(False, description="Force recompute instead of using cache"),
    current_user: AuthenticatedUser = Depends(get_current_user),
):
    """
    Get personalized chapter recommendations for authenticated user.

    Returns up to 3 recommended chapters based on:
    - User's skill level classification
    - Learning goals and hardware access
    - Completed chapters and prerequisites
    - Multi-factor relevance scoring

    Recommendations are cached for 1 hour unless force_refresh=true.

    Args:
        http_request: FastAPI Request object for rate limiting
        force_refresh: If true, bypass cache and recompute recommendations
        current_user: Authenticated user from JWT token

    Returns:
        RecommendationsResponse with up to 3 recommended chapters

    Raises:
        401: User not authenticated
        404: User skill level classification not found
        429: Rate limit exceeded (50 requests/hour)
        500: Database or service error

    Implementation: T055 (endpoint) + T056 (rate limiting)
    """
    # T056: Check rate limit (50 requests/hour for authenticated users)
    check_rate_limit(http_request, str(current_user.user_id))

    # T055: Get recommendations using RecommendationService
    try:
        recommendation_service = get_recommendation_service()

        # Get recommendations (with caching unless force_refresh)
        recommendations, from_cache = await recommendation_service.get_recommendations(
            user_id=str(current_user.user_id),
            force_refresh=force_refresh
        )

        logger.info(
            f"Retrieved {len(recommendations)} recommendations for user {current_user.user_id}, "
            f"from_cache={from_cache}, force_refresh={force_refresh}"
        )

        # Fetch chapter metadata for response enrichment
        conn = await asyncpg.connect(settings.database_url)
        try:
            chapter_ids = [rec.chapter_id for rec in recommendations]

            # Fetch metadata for all recommended chapters
            metadata_rows = await conn.fetch(
                """
                SELECT chapter_id, title, difficulty_level, module_number
                FROM chapter_metadata
                WHERE chapter_id = ANY($1)
                """,
                chapter_ids
            )

            # Create a map of chapter_id -> metadata
            metadata_map = {row['chapter_id']: row for row in metadata_rows}

            # Build enriched response
            enriched_recommendations = [
                ChapterRecommendationResponse(
                    chapter_id=rec.chapter_id,
                    relevance_score=rec.relevance_score,
                    recommended_at=rec.recommended_at,
                    reason=rec.reason,
                    title=metadata_map[rec.chapter_id]['title'] if rec.chapter_id in metadata_map else None,
                    difficulty_level=metadata_map[rec.chapter_id]['difficulty_level'] if rec.chapter_id in metadata_map else None,
                    module_number=metadata_map[rec.chapter_id]['module_number'] if rec.chapter_id in metadata_map else None,
                )
                for rec in recommendations
            ]

            return RecommendationsResponse(
                user_id=current_user.user_id,
                recommendations=enriched_recommendations,
                generated_at=datetime.utcnow(),
                from_cache=from_cache
            )

        finally:
            await conn.close()

    except ValueError as e:
        logger.error(f"Recommendation error for user {current_user.user_id}: {str(e)}")
        raise PersonalizationError.classification_not_found(current_user.user_id)
    except asyncpg.PostgresError as e:
        logger.error(
            f"Database error during recommendations for user {current_user.user_id}: {str(e)}"
        )
        raise PersonalizationError.database_error("recommendations", str(e))
    except Exception as e:
        logger.error(
            f"Unexpected error during recommendations for user {current_user.user_id}: {str(e)}"
        )
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error while generating recommendations"
        )
