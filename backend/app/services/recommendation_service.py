"""
Recommendation Service

Provides smart chapter recommendations based on user skill level, learning goals,
completed chapters, and prerequisites. Uses in-memory TTL caching to optimize performance.

Related Tasks: T045-T052
Related: research.md Section 3 - Recommendation Algorithm Design
"""

import asyncio
import json
from datetime import datetime
from typing import List, Dict, Any, Optional
import logging
from cachetools import TTLCache
import asyncpg

from app.models.chapter_recommendation import ChapterRecommendation
from app.config import settings

logger = logging.getLogger(__name__)


class RecommendationService:
    """
    Service for generating personalized chapter recommendations.

    Implements multi-factor scoring algorithm with caching for performance.
    Cache TTL: 1 hour (configurable via RECOMMENDATION_CACHE_TTL env var)
    Cache Max Size: 1000 users (configurable via RECOMMENDATION_CACHE_MAX_SIZE env var)
    """

    def __init__(self, db_pool: asyncpg.Pool):
        """
        Initialize recommendation service with database pool and cache.

        Args:
            db_pool: AsyncPG connection pool
        """
        self.db_pool = db_pool

        # T045: Initialize TTLCache for recommendations
        cache_ttl = getattr(settings, 'RECOMMENDATION_CACHE_TTL', 3600)  # 1 hour default
        cache_max_size = getattr(settings, 'RECOMMENDATION_CACHE_MAX_SIZE', 1000)  # 1000 users default

        self.recommendation_cache: TTLCache = TTLCache(
            maxsize=cache_max_size,
            ttl=cache_ttl
        )

        logger.info(
            f"RecommendationService initialized with cache TTL={cache_ttl}s, "
            f"max_size={cache_max_size}"
        )

    # T052: Invalidate cache method
    def invalidate_cache(self, user_id: str) -> None:
        """
        Invalidate recommendation cache for a specific user.

        Called when:
        - User updates profile (changes skill level, learning goals, hardware access)
        - User records progress (completes chapter, affects recommendations)

        Args:
            user_id: User UUID string

        Returns:
            None
        """
        cache_key = f"recommendations:{user_id}"

        if cache_key in self.recommendation_cache:
            del self.recommendation_cache[cache_key]
            logger.info(f"Invalidated recommendation cache for user_id={user_id}")
        else:
            logger.debug(f"No cache entry to invalidate for user_id={user_id}")

    # T047: Skill match score calculation
    def _calculate_skill_match_score(
        self,
        user_skill_level: str,
        chapter_difficulty: str
    ) -> float:
        """
        Calculate how well a chapter's difficulty matches user's skill level.

        Scoring:
        - Exact match (beginner→beginner): 1.0
        - One level off: 0.5
        - Two levels off: 0.2

        Args:
            user_skill_level: User's skill level ('beginner', 'intermediate', 'advanced')
            chapter_difficulty: Chapter difficulty ('beginner', 'intermediate', 'advanced')

        Returns:
            float: Skill match score (0.0-1.0)
        """
        skill_levels = ['beginner', 'intermediate', 'advanced']

        try:
            user_index = skill_levels.index(user_skill_level)
            chapter_index = skill_levels.index(chapter_difficulty)

            diff = abs(user_index - chapter_index)

            if diff == 0:
                return 1.0  # Exact match
            elif diff == 1:
                return 0.5  # One level off
            else:
                return 0.2  # Two levels off
        except ValueError:
            logger.warning(
                f"Invalid skill level or difficulty: user={user_skill_level}, "
                f"chapter={chapter_difficulty}"
            )
            return 0.3  # Default fallback score

    # T048: Learning goal match calculation
    def _calculate_learning_goal_match(
        self,
        user_learning_goal: str,
        chapter_tags: List[str]
    ) -> float:
        """
        Calculate how well chapter's learning goal tags match user's learning goal.

        Mapping:
        - career_transition → "practical"
        - academic_research → "research"
        - hobby_exploration → "theoretical" or "practical"
        - skill_upgrade → "practical"

        Scoring:
        - Chapter tags contain exact match: 1.0
        - Chapter tags contain related goal: 0.6
        - No match: 0.3

        Args:
            user_learning_goal: User's learning goal
            chapter_tags: List of chapter learning goal tags

        Returns:
            float: Learning goal match score (0.0-1.0)
        """
        # Map user learning goals to chapter tags
        goal_mapping = {
            'career_transition': ['practical'],
            'academic_research': ['research'],
            'hobby_exploration': ['theoretical', 'practical'],
            'skill_upgrade': ['practical']
        }

        user_preferred_tags = goal_mapping.get(user_learning_goal, [])

        if not user_preferred_tags or not chapter_tags:
            return 0.5  # Neutral score if no mapping

        # Check for exact matches
        exact_matches = set(user_preferred_tags) & set(chapter_tags)
        if exact_matches:
            return 1.0

        # Check for any overlap
        if any(tag in chapter_tags for tag in user_preferred_tags):
            return 0.6

        return 0.3  # No match

    # T049: Hardware match calculation
    def _calculate_hardware_match(
        self,
        user_hardware_access: str,
        chapter_requires_hardware: bool
    ) -> float:
        """
        Calculate how well chapter's hardware requirements match user's hardware access.

        Scoring:
        - User has hardware OR chapter doesn't require hardware: 1.0
        - User has simulation_only + chapter requires hardware: 0.3

        Args:
            user_hardware_access: User's hardware access ('physical', 'simulation_only')
            chapter_requires_hardware: Whether chapter requires physical hardware

        Returns:
            float: Hardware match score (0.0-1.0)
        """
        if not chapter_requires_hardware:
            # Chapter works for all users
            return 1.0

        if user_hardware_access == 'physical':
            # User has hardware, chapter requires it - perfect match
            return 1.0

        # User has simulation_only, chapter requires hardware - poor match
        return 0.3

    # T050: Prerequisite readiness calculation
    async def _calculate_prerequisite_readiness(
        self,
        user_id: str,
        chapter_prerequisites: List[str],
        user_completed_chapters: set
    ) -> float:
        """
        Calculate how ready user is for a chapter based on prerequisite completion.

        Scoring:
        - No prerequisites: 1.0
        - All prerequisites completed: 1.0
        - Most prerequisites completed (>= 75%): 0.7
        - Less than 75% completed: 0.0 (filters out chapter completely)

        Args:
            user_id: User UUID
            chapter_prerequisites: List of prerequisite chapter IDs
            user_completed_chapters: Set of chapter IDs user has completed

        Returns:
            float: Prerequisite readiness score (0.0-1.0)
        """
        if not chapter_prerequisites:
            # No prerequisites required
            return 1.0

        completed_prereqs = sum(
            1 for prereq in chapter_prerequisites if prereq in user_completed_chapters
        )

        total_prereqs = len(chapter_prerequisites)
        completion_ratio = completed_prereqs / total_prereqs

        if completion_ratio == 1.0:
            return 1.0  # All prerequisites met
        elif completion_ratio >= 0.75:
            return 0.7  # Most prerequisites met
        else:
            return 0.0  # Filter out - not ready

    # T046: Compute recommendations with multi-factor scoring
    async def compute_recommendations(
        self,
        user_id: str
    ) -> List[ChapterRecommendation]:
        """
        Compute personalized chapter recommendations using multi-factor scoring.

        Algorithm:
        1. Filter Phase: Exclude completed chapters and chapters with unmet prerequisites
        2. Scoring Phase: Calculate relevance score for each eligible chapter
        3. Ranking Phase: Sort by relevance_score DESC, return top 3

        Relevance Score Formula:
            relevance_score = (skill_match * 0.35) + (learning_goal_match * 0.30) +
                             (hardware_match * 0.20) + (prerequisite_readiness * 0.15)

        Args:
            user_id: User UUID

        Returns:
            List[ChapterRecommendation]: Top 3 recommended chapters (max)

        Raises:
            ValueError: If user not found or skill level not classified
        """
        async with self.db_pool.acquire() as conn:
            # Fetch user skill level classification
            skill_classification_row = await conn.fetchrow(
                """
                SELECT skill_level, based_on_profile
                FROM skill_level_classifications
                WHERE user_id = $1
                """,
                user_id
            )

            if not skill_classification_row:
                raise ValueError(f"No skill classification found for user_id={user_id}")

            user_skill_level = skill_classification_row['skill_level']
            based_on_profile_raw = skill_classification_row['based_on_profile']
            # Parse JSON string if needed (asyncpg returns string for JSONB when inserted as string)
            if isinstance(based_on_profile_raw, str):
                based_on_profile = json.loads(based_on_profile_raw)
            else:
                based_on_profile = based_on_profile_raw

            # Extract profile attributes
            user_learning_goal = based_on_profile.get('learning_goal', 'hobby_exploration')
            user_hardware_access = based_on_profile.get('hardware_access', 'simulation_only')

            # Fetch user's completed chapters
            completed_chapters_rows = await conn.fetch(
                """
                SELECT chapter_id
                FROM chapter_progress
                WHERE user_id = $1 AND status = 'completed'
                """,
                user_id
            )

            user_completed_chapters = {row['chapter_id'] for row in completed_chapters_rows}

            # Fetch all chapter metadata
            chapters_rows = await conn.fetch(
                """
                SELECT chapter_id, module_number, title, difficulty_level,
                       prerequisites, requires_hardware, learning_goal_tags
                FROM chapter_metadata
                ORDER BY module_number ASC, chapter_id ASC
                """
            )

            if not chapters_rows:
                logger.warning("No chapter metadata found in database")
                return []

            # Scoring phase
            scored_chapters = []

            for chapter_row in chapters_rows:
                chapter_id = chapter_row['chapter_id']

                # Filter: Skip completed chapters
                if chapter_id in user_completed_chapters:
                    continue

                # Extract chapter attributes
                difficulty_level = chapter_row['difficulty_level']
                prerequisites = chapter_row['prerequisites'] or []
                requires_hardware = chapter_row['requires_hardware']
                learning_goal_tags = chapter_row['learning_goal_tags'] or []

                # Calculate prerequisite readiness
                prerequisite_readiness = await self._calculate_prerequisite_readiness(
                    user_id,
                    prerequisites,
                    user_completed_chapters
                )

                # Filter: Skip if prerequisites not met (< 75%)
                if prerequisite_readiness == 0.0:
                    continue

                # Calculate other scoring factors
                skill_match = self._calculate_skill_match_score(user_skill_level, difficulty_level)
                goal_match = self._calculate_learning_goal_match(user_learning_goal, learning_goal_tags)
                hardware_match = self._calculate_hardware_match(user_hardware_access, requires_hardware)

                # Compute weighted relevance score
                relevance_score = (
                    (skill_match * 0.35) +
                    (goal_match * 0.30) +
                    (hardware_match * 0.20) +
                    (prerequisite_readiness * 0.15)
                )

                # Generate reason explanation
                reason_parts = []
                if skill_match >= 0.8:
                    reason_parts.append(f"Matches your {user_skill_level} skill level")
                if goal_match >= 0.8:
                    reason_parts.append(f"Aligns with your {user_learning_goal.replace('_', ' ')} goal")
                if hardware_match >= 0.8 and requires_hardware:
                    reason_parts.append("Suitable for your hardware setup")
                if prerequisite_readiness >= 0.9:
                    reason_parts.append("All prerequisites completed")

                if not reason_parts:
                    reason_parts.append("Good fit for your learning path")

                reason = " and ".join(reason_parts)

                scored_chapters.append({
                    'user_id': user_id,
                    'chapter_id': chapter_id,
                    'relevance_score': relevance_score,
                    'recommended_at': datetime.utcnow(),
                    'reason': reason,
                    'module_number': chapter_row['module_number']  # For tie-breaking
                })

            # Ranking phase: Sort by relevance_score DESC, then by module_number ASC (tie-breaker)
            scored_chapters.sort(
                key=lambda x: (-x['relevance_score'], x['module_number'])
            )

            # Return top 3 recommendations
            top_recommendations = scored_chapters[:3]

            # Convert to ChapterRecommendation models
            recommendations = [
                ChapterRecommendation(
                    user_id=rec['user_id'],
                    chapter_id=rec['chapter_id'],
                    relevance_score=rec['relevance_score'],
                    recommended_at=rec['recommended_at'],
                    reason=rec['reason']
                )
                for rec in top_recommendations
            ]

            logger.info(
                f"Computed {len(recommendations)} recommendations for user_id={user_id}, "
                f"scores={[r.relevance_score for r in recommendations]}"
            )

            return recommendations

    # T051: Get recommendations with caching
    async def get_recommendations(
        self,
        user_id: str,
        force_refresh: bool = False
    ) -> tuple[List[ChapterRecommendation], bool]:
        """
        Get personalized recommendations for a user (with caching).

        Caching Strategy:
        - Check cache first (TTL = 1 hour)
        - If cache hit and not force_refresh: return cached recommendations
        - If cache miss or force_refresh: compute and cache recommendations

        Args:
            user_id: User UUID
            force_refresh: If True, bypass cache and recompute

        Returns:
            tuple: (List[ChapterRecommendation], bool)
                - List of recommendations (max 3)
                - Boolean indicating if from cache (True) or freshly computed (False)

        Raises:
            ValueError: If user not found or skill level not classified
        """
        cache_key = f"recommendations:{user_id}"

        # Check cache (unless force_refresh)
        if not force_refresh and cache_key in self.recommendation_cache:
            cached_recommendations = self.recommendation_cache[cache_key]
            logger.info(f"Cache HIT for user_id={user_id}")
            return cached_recommendations, True

        # Cache miss or force refresh - compute recommendations
        logger.info(f"Cache MISS for user_id={user_id}, computing recommendations")

        recommendations = await self.compute_recommendations(user_id)

        # Store in cache
        self.recommendation_cache[cache_key] = recommendations

        return recommendations, False


# Singleton instance (initialized in main.py)
_recommendation_service_instance: Optional[RecommendationService] = None


def get_recommendation_service() -> RecommendationService:
    """Get singleton RecommendationService instance."""
    global _recommendation_service_instance
    if _recommendation_service_instance is None:
        raise RuntimeError(
            "RecommendationService not initialized. "
            "Call initialize_recommendation_service() first."
        )
    return _recommendation_service_instance


def initialize_recommendation_service(db_pool: asyncpg.Pool) -> None:
    """Initialize singleton RecommendationService instance."""
    global _recommendation_service_instance
    _recommendation_service_instance = RecommendationService(db_pool)
    logger.info("RecommendationService singleton initialized")
