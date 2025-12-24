"""
Unit tests for ClassificationService.
Tests the skill level classification algorithm logic.
"""

import pytest
from uuid import uuid4, UUID
from unittest.mock import AsyncMock, MagicMock, patch
from app.services.classification_service import ClassificationService
from app.models.skill_level_classification import SkillLevelClassificationResponse


class TestSkillScoreCalculation:
    """Test skill score calculation algorithm."""

    def test_beginner_beginner_none(self):
        """Test beginner experience + no ROS familiarity -> lowest score."""
        score = ClassificationService._calculate_skill_score("beginner", "none")
        # (0.6 * 1) + (0.4 * 1) = 1.0
        assert score == 1.0

    def test_beginner_basic(self):
        """Test beginner experience + basic ROS -> low-intermediate score."""
        score = ClassificationService._calculate_skill_score("beginner", "basic")
        # (0.6 * 1) + (0.4 * 2) = 1.4
        assert score == 1.4

    def test_intermediate_basic(self):
        """Test intermediate experience + basic ROS -> mid score."""
        score = ClassificationService._calculate_skill_score("intermediate", "basic")
        # (0.6 * 2) + (0.4 * 2) = 2.0
        assert score == 2.0

    def test_intermediate_proficient(self):
        """Test intermediate experience + proficient ROS -> high-intermediate score."""
        score = ClassificationService._calculate_skill_score(
            "intermediate", "proficient"
        )
        # (0.6 * 2) + (0.4 * 3) = 2.4
        assert abs(score - 2.4) < 0.001

    def test_advanced_proficient(self):
        """Test advanced experience + proficient ROS -> highest score."""
        score = ClassificationService._calculate_skill_score("advanced", "proficient")
        # (0.6 * 3) + (0.4 * 3) = 3.0
        assert score == 3.0

    def test_advanced_none(self):
        """Test advanced experience + no ROS -> mid-high score."""
        score = ClassificationService._calculate_skill_score("advanced", "none")
        # (0.6 * 3) + (0.4 * 1) = 2.2
        assert abs(score - 2.2) < 0.001

    def test_invalid_experience_defaults_to_beginner(self):
        """Test invalid experience level defaults to beginner value."""
        score = ClassificationService._calculate_skill_score("invalid", "basic")
        # (0.6 * 1) + (0.4 * 2) = 1.4 (defaults to 1 for invalid)
        assert score == 1.4

    def test_invalid_ros_defaults_to_none(self):
        """Test invalid ROS familiarity defaults to none value."""
        score = ClassificationService._calculate_skill_score("intermediate", "invalid")
        # (0.6 * 2) + (0.4 * 1) = 1.6 (defaults to 1 for invalid)
        assert score == 1.6


class TestSkillLevelDetermination:
    """Test skill level tier determination from scores."""

    def test_score_1_0_is_beginner(self):
        """Test score at lower bound -> beginner."""
        level = ClassificationService._determine_skill_level(1.0)
        assert level == "beginner"

    def test_score_1_4_is_beginner(self):
        """Test score at beginner threshold boundary -> beginner."""
        level = ClassificationService._determine_skill_level(1.4)
        assert level == "beginner"

    def test_score_1_5_is_intermediate(self):
        """Test score just above beginner threshold -> intermediate."""
        level = ClassificationService._determine_skill_level(1.5)
        assert level == "intermediate"

    def test_score_2_0_is_intermediate(self):
        """Test score in middle of intermediate range -> intermediate."""
        level = ClassificationService._determine_skill_level(2.0)
        assert level == "intermediate"

    def test_score_2_2_is_advanced(self):
        """Test score at intermediate threshold boundary -> advanced (advanced experience + no ROS)."""
        level = ClassificationService._determine_skill_level(2.2)
        assert level == "advanced"

    def test_score_2_3_is_advanced(self):
        """Test score just above intermediate threshold -> advanced."""
        level = ClassificationService._determine_skill_level(2.3)
        assert level == "advanced"

    def test_score_3_0_is_advanced(self):
        """Test score at upper bound -> advanced."""
        level = ClassificationService._determine_skill_level(3.0)
        assert level == "advanced"


class TestClassifyUser:
    """Test user classification with database operations."""

    @pytest.mark.asyncio
    async def test_classify_user_beginner_profile(self):
        """Test classification with beginner profile."""
        user_id = uuid4()
        mock_conn = AsyncMock()

        # Mock user profile query
        mock_conn.fetchrow = AsyncMock(
            side_effect=[
                # First call: fetch profile
                {
                    "experience_level": "beginner",
                    "ros_familiarity": "none",
                    "hardware_access": "simulation_only",
                    "learning_goal": "academic_research",
                    "preferred_language": "python",
                },
                # Second call: insert/update classification
                {
                    "id": uuid4(),
                    "user_id": user_id,
                    "skill_level": "beginner",
                    "calculated_at": "2025-12-23T00:00:00Z",
                    "updated_at": "2025-12-23T00:00:00Z",
                    "based_on_profile": {
                        "experience_level": "beginner",
                        "ros_familiarity": "none",
                        "hardware_access": "simulation_only",
                        "learning_goal": "academic_research",
                        "preferred_language": "python",
                    },
                },
            ]
        )

        result = await ClassificationService.classify_user(user_id, mock_conn)

        assert result.skill_level == "beginner"
        assert result.user_id == user_id
        assert result.based_on_profile["experience_level"] == "beginner"

    @pytest.mark.asyncio
    async def test_classify_user_intermediate_profile(self):
        """Test classification with intermediate profile."""
        user_id = uuid4()
        mock_conn = AsyncMock()

        mock_conn.fetchrow = AsyncMock(
            side_effect=[
                {
                    "experience_level": "intermediate",
                    "ros_familiarity": "basic",
                    "hardware_access": "physical_robot",
                    "learning_goal": "career_transition",
                    "preferred_language": "python",
                },
                {
                    "id": uuid4(),
                    "user_id": user_id,
                    "skill_level": "intermediate",
                    "calculated_at": "2025-12-23T00:00:00Z",
                    "updated_at": "2025-12-23T00:00:00Z",
                    "based_on_profile": {
                        "experience_level": "intermediate",
                        "ros_familiarity": "basic",
                        "hardware_access": "physical_robot",
                        "learning_goal": "career_transition",
                        "preferred_language": "python",
                    },
                },
            ]
        )

        result = await ClassificationService.classify_user(user_id, mock_conn)

        assert result.skill_level == "intermediate"
        assert result.user_id == user_id

    @pytest.mark.asyncio
    async def test_classify_user_advanced_profile(self):
        """Test classification with advanced profile."""
        user_id = uuid4()
        mock_conn = AsyncMock()

        mock_conn.fetchrow = AsyncMock(
            side_effect=[
                {
                    "experience_level": "advanced",
                    "ros_familiarity": "proficient",
                    "hardware_access": "physical_robot",
                    "learning_goal": "academic_research",
                    "preferred_language": "cpp",
                },
                {
                    "id": uuid4(),
                    "user_id": user_id,
                    "skill_level": "advanced",
                    "calculated_at": "2025-12-23T00:00:00Z",
                    "updated_at": "2025-12-23T00:00:00Z",
                    "based_on_profile": {
                        "experience_level": "advanced",
                        "ros_familiarity": "proficient",
                        "hardware_access": "physical_robot",
                        "learning_goal": "academic_research",
                        "preferred_language": "cpp",
                    },
                },
            ]
        )

        result = await ClassificationService.classify_user(user_id, mock_conn)

        assert result.skill_level == "advanced"
        assert result.user_id == user_id

    @pytest.mark.asyncio
    async def test_classify_user_profile_not_found(self):
        """Test classification fails when profile not found."""
        user_id = uuid4()
        mock_conn = AsyncMock()
        mock_conn.fetchrow = AsyncMock(return_value=None)

        with pytest.raises(ValueError, match="Profile not found"):
            await ClassificationService.classify_user(user_id, mock_conn)

    @pytest.mark.asyncio
    async def test_classify_user_incomplete_profile_missing_experience(self):
        """Test classification fails when experience_level missing."""
        user_id = uuid4()
        mock_conn = AsyncMock()
        mock_conn.fetchrow = AsyncMock(
            return_value={
                "experience_level": None,  # Missing
                "ros_familiarity": "basic",
                "hardware_access": "simulation_only",
                "learning_goal": "academic_research",
                "preferred_language": "python",
            }
        )

        with pytest.raises(ValueError, match="Profile incomplete"):
            await ClassificationService.classify_user(user_id, mock_conn)

    @pytest.mark.asyncio
    async def test_classify_user_incomplete_profile_missing_ros(self):
        """Test classification fails when ros_familiarity missing."""
        user_id = uuid4()
        mock_conn = AsyncMock()
        mock_conn.fetchrow = AsyncMock(
            return_value={
                "experience_level": "intermediate",
                "ros_familiarity": None,  # Missing
                "hardware_access": "simulation_only",
                "learning_goal": "academic_research",
                "preferred_language": "python",
            }
        )

        with pytest.raises(ValueError, match="Profile incomplete"):
            await ClassificationService.classify_user(user_id, mock_conn)


class TestGetClassification:
    """Test retrieval of existing classification."""

    @pytest.mark.asyncio
    async def test_get_classification_exists(self):
        """Test retrieving existing classification."""
        user_id = uuid4()
        classification_id = uuid4()
        mock_conn = AsyncMock()

        mock_conn.fetchrow = AsyncMock(
            return_value={
                "id": classification_id,
                "user_id": user_id,
                "skill_level": "intermediate",
                "calculated_at": "2025-12-23T00:00:00Z",
                "updated_at": "2025-12-23T00:00:00Z",
                "based_on_profile": {
                    "experience_level": "intermediate",
                    "ros_familiarity": "basic",
                },
            }
        )

        result = await ClassificationService.get_classification(user_id, mock_conn)

        assert result is not None
        assert result.id == classification_id
        assert result.user_id == user_id
        assert result.skill_level == "intermediate"

    @pytest.mark.asyncio
    async def test_get_classification_not_exists(self):
        """Test retrieving non-existent classification returns None."""
        user_id = uuid4()
        mock_conn = AsyncMock()
        mock_conn.fetchrow = AsyncMock(return_value=None)

        result = await ClassificationService.get_classification(user_id, mock_conn)

        assert result is None


class TestRecalculateClassification:
    """Test force recalculation of classification."""

    @pytest.mark.asyncio
    async def test_recalculate_classification(self):
        """Test recalculation delegates to classify_user."""
        user_id = uuid4()
        mock_conn = AsyncMock()

        mock_conn.fetchrow = AsyncMock(
            side_effect=[
                {
                    "experience_level": "advanced",
                    "ros_familiarity": "proficient",
                    "hardware_access": "physical_robot",
                    "learning_goal": "academic_research",
                    "preferred_language": "python",
                },
                {
                    "id": uuid4(),
                    "user_id": user_id,
                    "skill_level": "advanced",
                    "calculated_at": "2025-12-23T00:00:00Z",
                    "updated_at": "2025-12-23T00:00:00Z",
                    "based_on_profile": {
                        "experience_level": "advanced",
                        "ros_familiarity": "proficient",
                        "hardware_access": "physical_robot",
                        "learning_goal": "academic_research",
                        "preferred_language": "python",
                    },
                },
            ]
        )

        result = await ClassificationService.recalculate_classification(
            user_id, mock_conn
        )

        assert result.skill_level == "advanced"
        assert result.user_id == user_id


class TestClassificationConsistency:
    """Test that classification is deterministic and consistent."""

    def test_same_profile_gives_same_classification(self):
        """Test that identical profiles always get same classification."""
        # Test multiple times to ensure determinism
        for _ in range(10):
            score1 = ClassificationService._calculate_skill_score(
                "intermediate", "basic"
            )
            score2 = ClassificationService._calculate_skill_score(
                "intermediate", "basic"
            )
            assert score1 == score2

            level1 = ClassificationService._determine_skill_level(score1)
            level2 = ClassificationService._determine_skill_level(score2)
            assert level1 == level2

    def test_classification_thresholds_are_correct(self):
        """Test that threshold boundaries match spec (FR-003)."""
        # Updated thresholds:
        # - skill_score <= 1.4: beginner
        # - 1.4 < skill_score < 2.2: intermediate
        # - skill_score >= 2.2: advanced (so advanced experience + no ROS = advanced)

        assert ClassificationService.BEGINNER_THRESHOLD == 1.4
        assert ClassificationService.INTERMEDIATE_THRESHOLD == 2.2

        # Test boundary cases
        assert ClassificationService._determine_skill_level(1.4) == "beginner"
        assert ClassificationService._determine_skill_level(1.41) == "intermediate"
        assert ClassificationService._determine_skill_level(2.19) == "intermediate"
        assert ClassificationService._determine_skill_level(2.2) == "advanced"
