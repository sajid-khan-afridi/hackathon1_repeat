"""
Classification Service for Phase 4B Personalization Engine.
Handles skill level classification logic using weighted scoring algorithm.
"""

import logging
from uuid import UUID
from datetime import datetime
from typing import Optional, Dict, Any
import asyncpg
from app.config import settings
from app.models.skill_level_classification import (
    SkillLevelClassificationCreate,
    SkillLevelClassificationUpdate,
    SkillLevelClassificationResponse,
)
from app.models.profile import ExperienceLevel, ROSFamiliarity

logger = logging.getLogger(__name__)


class ClassificationService:
    """Service for skill level classification operations."""

    # Weighted scoring algorithm constants (from research.md)
    EXPERIENCE_WEIGHT = 0.6
    ROS_WEIGHT = 0.4

    EXPERIENCE_VALUES = {
        "beginner": 1,
        "intermediate": 2,
        "advanced": 3,
    }

    ROS_FAMILIARITY_VALUES = {
        "none": 1,
        "basic": 2,
        "proficient": 3,
    }

    # Classification thresholds
    BEGINNER_THRESHOLD = 1.4
    INTERMEDIATE_THRESHOLD = 2.2

    @staticmethod
    def _calculate_skill_score(
        experience_level: str, ros_familiarity: str
    ) -> float:
        """
        Calculate weighted skill score from profile attributes.

        Args:
            experience_level: User's programming experience level
            ros_familiarity: User's ROS familiarity level

        Returns:
            Weighted skill score (1.0 - 3.0)
        """
        exp_value = ClassificationService.EXPERIENCE_VALUES.get(
            experience_level, 1
        )
        ros_value = ClassificationService.ROS_FAMILIARITY_VALUES.get(
            ros_familiarity, 1
        )

        skill_score = (
            ClassificationService.EXPERIENCE_WEIGHT * exp_value
            + ClassificationService.ROS_WEIGHT * ros_value
        )

        logger.debug(
            f"Skill score calculation: exp={experience_level}({exp_value}), "
            f"ros={ros_familiarity}({ros_value}) -> score={skill_score:.2f}"
        )

        return skill_score

    @staticmethod
    def _determine_skill_level(skill_score: float) -> str:
        """
        Determine skill level tier from skill score.

        Args:
            skill_score: Computed weighted score

        Returns:
            Skill level: 'beginner', 'intermediate', or 'advanced'
        """
        if skill_score <= ClassificationService.BEGINNER_THRESHOLD:
            return "beginner"
        elif skill_score <= ClassificationService.INTERMEDIATE_THRESHOLD:
            return "intermediate"
        else:
            return "advanced"

    @staticmethod
    async def classify_user(
        user_id: UUID, conn: asyncpg.Connection
    ) -> SkillLevelClassificationResponse:
        """
        Classify user's skill level based on their profile attributes.
        Creates or updates classification record in database.

        Args:
            user_id: UUID of the user to classify
            conn: Database connection

        Returns:
            SkillLevelClassificationResponse with classification result

        Raises:
            ValueError: If user profile not found or incomplete
            asyncpg.PostgresError: If database operation fails
        """
        # Fetch user profile
        profile = await conn.fetchrow(
            """
            SELECT experience_level, ros_familiarity, hardware_access,
                   learning_goal, preferred_language
            FROM user_profiles
            WHERE user_id = $1
            """,
            user_id,
        )

        if not profile:
            raise ValueError(f"Profile not found for user {user_id}")

        if not profile["experience_level"] or not profile["ros_familiarity"]:
            raise ValueError(
                f"Profile incomplete for user {user_id}: "
                "experience_level and ros_familiarity required"
            )

        # Calculate skill score
        skill_score = ClassificationService._calculate_skill_score(
            profile["experience_level"], profile["ros_familiarity"]
        )

        # Determine skill level
        skill_level = ClassificationService._determine_skill_level(skill_score)

        # Create profile snapshot
        profile_snapshot = {
            "experience_level": profile["experience_level"],
            "ros_familiarity": profile["ros_familiarity"],
            "hardware_access": profile["hardware_access"],
            "learning_goal": profile["learning_goal"],
            "preferred_language": profile["preferred_language"],
        }

        # Insert or update classification
        result = await conn.fetchrow(
            """
            INSERT INTO skill_level_classifications
                (user_id, skill_level, based_on_profile)
            VALUES ($1, $2, $3)
            ON CONFLICT (user_id)
            DO UPDATE SET
                skill_level = EXCLUDED.skill_level,
                updated_at = NOW(),
                based_on_profile = EXCLUDED.based_on_profile
            RETURNING id, user_id, skill_level, calculated_at, updated_at, based_on_profile
            """,
            user_id,
            skill_level,
            profile_snapshot,
        )

        logger.info(
            f"Classified user {user_id} as {skill_level} "
            f"(score={skill_score:.2f})"
        )

        return SkillLevelClassificationResponse(
            id=result["id"],
            user_id=result["user_id"],
            skill_level=result["skill_level"],
            calculated_at=result["calculated_at"],
            updated_at=result["updated_at"],
            based_on_profile=result["based_on_profile"],
        )

    @staticmethod
    async def get_classification(
        user_id: UUID, conn: asyncpg.Connection
    ) -> Optional[SkillLevelClassificationResponse]:
        """
        Retrieve existing skill level classification for a user.

        Args:
            user_id: UUID of the user
            conn: Database connection

        Returns:
            SkillLevelClassificationResponse if exists, None otherwise
        """
        result = await conn.fetchrow(
            """
            SELECT id, user_id, skill_level, calculated_at, updated_at, based_on_profile
            FROM skill_level_classifications
            WHERE user_id = $1
            """,
            user_id,
        )

        if not result:
            return None

        return SkillLevelClassificationResponse(
            id=result["id"],
            user_id=result["user_id"],
            skill_level=result["skill_level"],
            calculated_at=result["calculated_at"],
            updated_at=result["updated_at"],
            based_on_profile=result["based_on_profile"],
        )

    @staticmethod
    async def recalculate_classification(
        user_id: UUID, conn: asyncpg.Connection
    ) -> SkillLevelClassificationResponse:
        """
        Force recalculation of user's skill level classification.
        Useful when user updates their profile.

        Args:
            user_id: UUID of the user
            conn: Database connection

        Returns:
            Updated SkillLevelClassificationResponse

        Raises:
            ValueError: If user profile not found or incomplete
            asyncpg.PostgresError: If database operation fails
        """
        logger.info(f"Recalculating skill level for user {user_id}")
        return await ClassificationService.classify_user(user_id, conn)
