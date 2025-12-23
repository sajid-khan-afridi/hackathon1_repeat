"""
Unit Tests for Recommendation Service

Tests the scoring algorithms used for personalized chapter recommendations.
Covers:
- T041: Recommendation scoring algorithm
- T042: Recommendation caching and invalidation

Related: backend/app/services/recommendation_service.py
"""

import pytest
import asyncio
from unittest.mock import AsyncMock, Mock, patch, MagicMock
from datetime import datetime
from uuid import uuid4
import asyncpg

from app.services.recommendation_service import RecommendationService
from app.models.chapter_recommendation import ChapterRecommendation


@pytest.fixture
def mock_db_pool():
    """Create a mock database pool for testing."""
    pool = AsyncMock(spec=asyncpg.Pool)
    return pool


@pytest.fixture
def recommendation_service(mock_db_pool):
    """Create a RecommendationService instance with mocked database."""
    service = RecommendationService(mock_db_pool)
    return service


# ============================================
# T041: Unit Tests for Scoring Algorithm
# ============================================

class TestSkillMatchScoring:
    """Test skill level matching score calculation."""

    def test_exact_match_beginner(self, recommendation_service):
        """Test exact match: beginner user → beginner chapter."""
        score = recommendation_service._calculate_skill_match_score('beginner', 'beginner')
        assert score == 1.0, "Exact match should return 1.0"

    def test_exact_match_intermediate(self, recommendation_service):
        """Test exact match: intermediate user → intermediate chapter."""
        score = recommendation_service._calculate_skill_match_score('intermediate', 'intermediate')
        assert score == 1.0, "Exact match should return 1.0"

    def test_exact_match_advanced(self, recommendation_service):
        """Test exact match: advanced user → advanced chapter."""
        score = recommendation_service._calculate_skill_match_score('advanced', 'advanced')
        assert score == 1.0, "Exact match should return 1.0"

    def test_one_level_off_beginner_to_intermediate(self, recommendation_service):
        """Test one level off: beginner user → intermediate chapter."""
        score = recommendation_service._calculate_skill_match_score('beginner', 'intermediate')
        assert score == 0.5, "One level off should return 0.5"

    def test_one_level_off_intermediate_to_advanced(self, recommendation_service):
        """Test one level off: intermediate user → advanced chapter."""
        score = recommendation_service._calculate_skill_match_score('intermediate', 'advanced')
        assert score == 0.5, "One level off should return 0.5"

    def test_one_level_off_intermediate_to_beginner(self, recommendation_service):
        """Test one level off: intermediate user → beginner chapter."""
        score = recommendation_service._calculate_skill_match_score('intermediate', 'beginner')
        assert score == 0.5, "One level off should return 0.5"

    def test_two_levels_off_beginner_to_advanced(self, recommendation_service):
        """Test two levels off: beginner user → advanced chapter."""
        score = recommendation_service._calculate_skill_match_score('beginner', 'advanced')
        assert score == 0.2, "Two levels off should return 0.2"

    def test_two_levels_off_advanced_to_beginner(self, recommendation_service):
        """Test two levels off: advanced user → beginner chapter."""
        score = recommendation_service._calculate_skill_match_score('advanced', 'beginner')
        assert score == 0.2, "Two levels off should return 0.2"

    def test_invalid_skill_level(self, recommendation_service):
        """Test handling of invalid skill levels."""
        score = recommendation_service._calculate_skill_match_score('invalid', 'beginner')
        assert score == 0.3, "Invalid skill level should return default fallback score 0.3"

    def test_invalid_chapter_difficulty(self, recommendation_service):
        """Test handling of invalid chapter difficulty."""
        score = recommendation_service._calculate_skill_match_score('beginner', 'invalid')
        assert score == 0.3, "Invalid difficulty should return default fallback score 0.3"


class TestLearningGoalMatching:
    """Test learning goal matching score calculation."""

    def test_career_transition_with_practical_tag(self, recommendation_service):
        """Test career_transition goal matches practical tag."""
        score = recommendation_service._calculate_learning_goal_match(
            'career_transition', ['practical']
        )
        assert score == 1.0, "Career transition should match practical tag"

    def test_career_transition_with_research_tag(self, recommendation_service):
        """Test career_transition goal does not match research tag."""
        score = recommendation_service._calculate_learning_goal_match(
            'career_transition', ['research']
        )
        assert score == 0.3, "Career transition should not match research tag"

    def test_academic_research_with_research_tag(self, recommendation_service):
        """Test academic_research goal matches research tag."""
        score = recommendation_service._calculate_learning_goal_match(
            'academic_research', ['research']
        )
        assert score == 1.0, "Academic research should match research tag"

    def test_hobby_exploration_with_theoretical_tag(self, recommendation_service):
        """Test hobby_exploration goal matches theoretical tag."""
        score = recommendation_service._calculate_learning_goal_match(
            'hobby_exploration', ['theoretical']
        )
        assert score == 1.0, "Hobby exploration should match theoretical tag"

    def test_hobby_exploration_with_practical_tag(self, recommendation_service):
        """Test hobby_exploration goal matches practical tag."""
        score = recommendation_service._calculate_learning_goal_match(
            'hobby_exploration', ['practical']
        )
        assert score == 1.0, "Hobby exploration should match practical tag"

    def test_skill_upgrade_with_practical_tag(self, recommendation_service):
        """Test skill_upgrade goal matches practical tag."""
        score = recommendation_service._calculate_learning_goal_match(
            'skill_upgrade', ['practical']
        )
        assert score == 1.0, "Skill upgrade should match practical tag"

    def test_no_matching_tags(self, recommendation_service):
        """Test when chapter has no matching learning goal tags."""
        score = recommendation_service._calculate_learning_goal_match(
            'career_transition', ['research', 'theoretical']
        )
        assert score == 0.3, "No matching tags should return 0.3"

    def test_empty_chapter_tags(self, recommendation_service):
        """Test when chapter has empty learning goal tags."""
        score = recommendation_service._calculate_learning_goal_match(
            'career_transition', []
        )
        assert score == 0.5, "Empty chapter tags should return neutral score 0.5"

    def test_invalid_learning_goal(self, recommendation_service):
        """Test handling of invalid learning goal."""
        score = recommendation_service._calculate_learning_goal_match(
            'invalid_goal', ['practical']
        )
        assert score == 0.5, "Invalid learning goal should return neutral score 0.5"


class TestHardwareMatching:
    """Test hardware matching score calculation."""

    def test_no_hardware_required(self, recommendation_service):
        """Test chapter that doesn't require hardware (always matches)."""
        # User has physical hardware
        score1 = recommendation_service._calculate_hardware_match('physical', False)
        assert score1 == 1.0, "No hardware required should match all users"

        # User has simulation only
        score2 = recommendation_service._calculate_hardware_match('simulation_only', False)
        assert score2 == 1.0, "No hardware required should match all users"

    def test_physical_user_hardware_required(self, recommendation_service):
        """Test user with physical hardware accessing hardware-required chapter."""
        score = recommendation_service._calculate_hardware_match('physical', True)
        assert score == 1.0, "Physical hardware user should match hardware-required chapter"

    def test_simulation_only_user_hardware_required(self, recommendation_service):
        """Test user with simulation only accessing hardware-required chapter."""
        score = recommendation_service._calculate_hardware_match('simulation_only', True)
        assert score == 0.3, "Simulation-only user should poorly match hardware-required chapter"


class TestPrerequisiteReadiness:
    """Test prerequisite readiness score calculation."""

    @pytest.mark.asyncio
    async def test_no_prerequisites(self, recommendation_service):
        """Test chapter with no prerequisites."""
        score = await recommendation_service._calculate_prerequisite_readiness(
            user_id='test-user',
            chapter_prerequisites=[],
            user_completed_chapters=set()
        )
        assert score == 1.0, "No prerequisites should return 1.0"

    @pytest.mark.asyncio
    async def test_all_prerequisites_met(self, recommendation_service):
        """Test all prerequisites completed."""
        completed = {'chapter-1', 'chapter-2', 'chapter-3'}
        prerequisites = ['chapter-1', 'chapter-2', 'chapter-3']

        score = await recommendation_service._calculate_prerequisite_readiness(
            user_id='test-user',
            chapter_prerequisites=prerequisites,
            user_completed_chapters=completed
        )
        assert score == 1.0, "All prerequisites met should return 1.0"

    @pytest.mark.asyncio
    async def test_most_prerequisites_met_75_percent(self, recommendation_service):
        """Test 75% of prerequisites completed."""
        completed = {'chapter-1', 'chapter-2', 'chapter-3'}
        prerequisites = ['chapter-1', 'chapter-2', 'chapter-3', 'chapter-4']

        score = await recommendation_service._calculate_prerequisite_readiness(
            user_id='test-user',
            chapter_prerequisites=prerequisites,
            user_completed_chapters=completed
        )
        assert score == 0.7, "75% prerequisites met should return 0.7"

    @pytest.mark.asyncio
    async def test_most_prerequisites_met_80_percent(self, recommendation_service):
        """Test 80% of prerequisites completed (>= 75% threshold)."""
        completed = {'chapter-1', 'chapter-2', 'chapter-3', 'chapter-4'}
        prerequisites = ['chapter-1', 'chapter-2', 'chapter-3', 'chapter-4', 'chapter-5']

        score = await recommendation_service._calculate_prerequisite_readiness(
            user_id='test-user',
            chapter_prerequisites=prerequisites,
            user_completed_chapters=completed
        )
        assert score == 0.7, "80% prerequisites met should return 0.7"

    @pytest.mark.asyncio
    async def test_less_than_75_percent_prerequisites(self, recommendation_service):
        """Test less than 75% of prerequisites completed (filtered out)."""
        completed = {'chapter-1'}
        prerequisites = ['chapter-1', 'chapter-2', 'chapter-3', 'chapter-4']

        score = await recommendation_service._calculate_prerequisite_readiness(
            user_id='test-user',
            chapter_prerequisites=prerequisites,
            user_completed_chapters=completed
        )
        assert score == 0.0, "Less than 75% prerequisites should return 0.0 (filter out)"

    @pytest.mark.asyncio
    async def test_no_prerequisites_completed(self, recommendation_service):
        """Test zero prerequisites completed."""
        completed = set()
        prerequisites = ['chapter-1', 'chapter-2', 'chapter-3']

        score = await recommendation_service._calculate_prerequisite_readiness(
            user_id='test-user',
            chapter_prerequisites=prerequisites,
            user_completed_chapters=completed
        )
        assert score == 0.0, "No prerequisites met should return 0.0 (filter out)"


class TestMultiFactorScoring:
    """Test the integrated multi-factor relevance scoring."""

    @pytest.mark.asyncio
    async def test_compute_recommendations_perfect_match(self, recommendation_service, mock_db_pool):
        """Test recommendation computation for a perfect match scenario."""
        # Mock database responses
        mock_conn = AsyncMock()

        # Mock skill classification
        mock_conn.fetchrow.return_value = {
            'skill_level': 'beginner',
            'based_on_profile': {
                'learning_goal': 'career_transition',
                'hardware_access': 'physical'
            }
        }

        # Mock completed chapters (empty)
        mock_conn.fetch.side_effect = [
            [],  # No completed chapters
            [  # Chapter metadata
                {
                    'chapter_id': 'chapter-1',
                    'module_number': 1,
                    'title': 'Introduction to ROS',
                    'difficulty_level': 'beginner',
                    'prerequisites': [],
                    'requires_hardware': False,
                    'learning_goal_tags': ['practical']
                }
            ]
        ]

        mock_db_pool.acquire.return_value.__aenter__.return_value = mock_conn

        # Compute recommendations
        recommendations = await recommendation_service.compute_recommendations('test-user-id')

        # Verify we got recommendations
        assert len(recommendations) > 0, "Should return at least one recommendation"
        assert recommendations[0].chapter_id == 'chapter-1'

        # Verify relevance score is high (perfect match scenario)
        # Skill match: 1.0 (beginner→beginner) * 0.35 = 0.35
        # Goal match: 1.0 (career_transition→practical) * 0.30 = 0.30
        # Hardware match: 1.0 (no hardware required) * 0.20 = 0.20
        # Prereq readiness: 1.0 (no prerequisites) * 0.15 = 0.15
        # Total expected: 1.0
        assert recommendations[0].relevance_score == 1.0, "Perfect match should have relevance score of 1.0"

    @pytest.mark.asyncio
    async def test_compute_recommendations_filters_completed_chapters(self, recommendation_service, mock_db_pool):
        """Test that completed chapters are filtered out."""
        mock_conn = AsyncMock()

        # Mock skill classification
        mock_conn.fetchrow.return_value = {
            'skill_level': 'intermediate',
            'based_on_profile': {
                'learning_goal': 'skill_upgrade',
                'hardware_access': 'simulation_only'
            }
        }

        # Mock completed chapters
        mock_conn.fetch.side_effect = [
            [{'chapter_id': 'chapter-1'}],  # User completed chapter-1
            [  # All chapters
                {
                    'chapter_id': 'chapter-1',
                    'module_number': 1,
                    'title': 'Completed Chapter',
                    'difficulty_level': 'beginner',
                    'prerequisites': [],
                    'requires_hardware': False,
                    'learning_goal_tags': ['practical']
                },
                {
                    'chapter_id': 'chapter-2',
                    'module_number': 2,
                    'title': 'Next Chapter',
                    'difficulty_level': 'intermediate',
                    'prerequisites': [],
                    'requires_hardware': False,
                    'learning_goal_tags': ['practical']
                }
            ]
        ]

        mock_db_pool.acquire.return_value.__aenter__.return_value = mock_conn

        # Compute recommendations
        recommendations = await recommendation_service.compute_recommendations('test-user-id')

        # Verify chapter-1 is filtered out
        chapter_ids = [rec.chapter_id for rec in recommendations]
        assert 'chapter-1' not in chapter_ids, "Completed chapters should be filtered out"
        assert 'chapter-2' in chapter_ids, "Uncompleted chapters should be included"

    @pytest.mark.asyncio
    async def test_compute_recommendations_filters_unmet_prerequisites(self, recommendation_service, mock_db_pool):
        """Test that chapters with unmet prerequisites (<75%) are filtered out."""
        mock_conn = AsyncMock()

        # Mock skill classification
        mock_conn.fetchrow.return_value = {
            'skill_level': 'intermediate',
            'based_on_profile': {
                'learning_goal': 'academic_research',
                'hardware_access': 'physical'
            }
        }

        # Mock completed chapters (only completed chapter-1)
        mock_conn.fetch.side_effect = [
            [{'chapter_id': 'chapter-1'}],
            [  # All chapters
                {
                    'chapter_id': 'chapter-5',
                    'module_number': 5,
                    'title': 'Advanced Chapter',
                    'difficulty_level': 'intermediate',
                    'prerequisites': ['chapter-1', 'chapter-2', 'chapter-3', 'chapter-4'],  # 25% completed
                    'requires_hardware': False,
                    'learning_goal_tags': ['research']
                }
            ]
        ]

        mock_db_pool.acquire.return_value.__aenter__.return_value = mock_conn

        # Compute recommendations
        recommendations = await recommendation_service.compute_recommendations('test-user-id')

        # Verify chapter with unmet prerequisites is filtered out
        assert len(recommendations) == 0, "Chapters with <75% prerequisites should be filtered out"

    @pytest.mark.asyncio
    async def test_compute_recommendations_returns_top_3(self, recommendation_service, mock_db_pool):
        """Test that only top 3 recommendations are returned."""
        mock_conn = AsyncMock()

        # Mock skill classification
        mock_conn.fetchrow.return_value = {
            'skill_level': 'beginner',
            'based_on_profile': {
                'learning_goal': 'hobby_exploration',
                'hardware_access': 'physical'
            }
        }

        # Mock completed chapters (empty)
        mock_conn.fetch.side_effect = [
            [],
            [  # 5 chapters available
                {
                    'chapter_id': f'chapter-{i}',
                    'module_number': i,
                    'title': f'Chapter {i}',
                    'difficulty_level': 'beginner',
                    'prerequisites': [],
                    'requires_hardware': False,
                    'learning_goal_tags': ['theoretical']
                }
                for i in range(1, 6)
            ]
        ]

        mock_db_pool.acquire.return_value.__aenter__.return_value = mock_conn

        # Compute recommendations
        recommendations = await recommendation_service.compute_recommendations('test-user-id')

        # Verify only top 3 are returned
        assert len(recommendations) <= 3, "Should return maximum 3 recommendations"

    @pytest.mark.asyncio
    async def test_compute_recommendations_raises_on_no_classification(self, recommendation_service, mock_db_pool):
        """Test that ValueError is raised when no skill classification exists."""
        mock_conn = AsyncMock()

        # Mock no skill classification found
        mock_conn.fetchrow.return_value = None

        mock_db_pool.acquire.return_value.__aenter__.return_value = mock_conn

        # Verify ValueError is raised
        with pytest.raises(ValueError, match="No skill classification found"):
            await recommendation_service.compute_recommendations('test-user-id')


# ============================================
# T042: Unit Tests for Caching and Invalidation
# ============================================

class TestRecommendationCaching:
    """Test recommendation caching functionality."""

    @pytest.mark.asyncio
    async def test_cache_miss_computes_recommendations(self, recommendation_service, mock_db_pool):
        """Test that cache miss triggers computation."""
        mock_conn = AsyncMock()

        # Mock skill classification
        mock_conn.fetchrow.return_value = {
            'skill_level': 'beginner',
            'based_on_profile': {
                'learning_goal': 'career_transition',
                'hardware_access': 'physical'
            }
        }

        # Mock database responses
        mock_conn.fetch.side_effect = [
            [],  # No completed chapters
            [  # Chapter metadata
                {
                    'chapter_id': 'chapter-1',
                    'module_number': 1,
                    'title': 'Test Chapter',
                    'difficulty_level': 'beginner',
                    'prerequisites': [],
                    'requires_hardware': False,
                    'learning_goal_tags': ['practical']
                }
            ]
        ]

        mock_db_pool.acquire.return_value.__aenter__.return_value = mock_conn

        # Get recommendations (cache miss)
        recommendations, from_cache = await recommendation_service.get_recommendations('user-1', force_refresh=False)

        # Verify computation happened
        assert not from_cache, "First call should be cache miss"
        assert len(recommendations) > 0, "Should compute and return recommendations"

    @pytest.mark.asyncio
    async def test_cache_hit_returns_cached_recommendations(self, recommendation_service, mock_db_pool):
        """Test that cache hit returns cached recommendations without recomputation."""
        mock_conn = AsyncMock()

        # Mock skill classification
        mock_conn.fetchrow.return_value = {
            'skill_level': 'beginner',
            'based_on_profile': {
                'learning_goal': 'career_transition',
                'hardware_access': 'physical'
            }
        }

        # Mock database responses
        mock_conn.fetch.side_effect = [
            [],  # No completed chapters
            [  # Chapter metadata
                {
                    'chapter_id': 'chapter-1',
                    'module_number': 1,
                    'title': 'Test Chapter',
                    'difficulty_level': 'beginner',
                    'prerequisites': [],
                    'requires_hardware': False,
                    'learning_goal_tags': ['practical']
                }
            ]
        ]

        mock_db_pool.acquire.return_value.__aenter__.return_value = mock_conn

        # First call - cache miss
        recs1, from_cache1 = await recommendation_service.get_recommendations('user-1', force_refresh=False)
        assert not from_cache1, "First call should be cache miss"

        # Reset mock to verify no DB calls on second request
        mock_conn.reset_mock()

        # Second call - should be cache hit
        recs2, from_cache2 = await recommendation_service.get_recommendations('user-1', force_refresh=False)

        # Verify cache hit
        assert from_cache2, "Second call should be cache hit"
        assert recs1 == recs2, "Cached recommendations should match original"

        # Verify no database calls were made
        mock_conn.fetchrow.assert_not_called()
        mock_conn.fetch.assert_not_called()

    @pytest.mark.asyncio
    async def test_force_refresh_bypasses_cache(self, recommendation_service, mock_db_pool):
        """Test that force_refresh=True bypasses cache and recomputes."""
        mock_conn = AsyncMock()

        # Mock skill classification
        mock_conn.fetchrow.return_value = {
            'skill_level': 'intermediate',
            'based_on_profile': {
                'learning_goal': 'skill_upgrade',
                'hardware_access': 'simulation_only'
            }
        }

        # Mock database responses for both calls
        mock_conn.fetch.side_effect = [
            [],  # No completed chapters - first call
            [  # Chapter metadata - first call
                {
                    'chapter_id': 'chapter-1',
                    'module_number': 1,
                    'title': 'Test Chapter',
                    'difficulty_level': 'intermediate',
                    'prerequisites': [],
                    'requires_hardware': False,
                    'learning_goal_tags': ['practical']
                }
            ],
            [],  # No completed chapters - second call
            [  # Chapter metadata - second call
                {
                    'chapter_id': 'chapter-2',
                    'module_number': 2,
                    'title': 'Updated Chapter',
                    'difficulty_level': 'intermediate',
                    'prerequisites': [],
                    'requires_hardware': False,
                    'learning_goal_tags': ['practical']
                }
            ]
        ]

        mock_db_pool.acquire.return_value.__aenter__.return_value = mock_conn

        # First call - populates cache
        recs1, from_cache1 = await recommendation_service.get_recommendations('user-1', force_refresh=False)
        assert not from_cache1, "First call should compute"

        # Second call with force_refresh - should bypass cache
        recs2, from_cache2 = await recommendation_service.get_recommendations('user-1', force_refresh=True)

        # Verify force refresh worked
        assert not from_cache2, "Force refresh should bypass cache"

        # Verify database was called twice (once for each get_recommendations call)
        assert mock_conn.fetchrow.call_count == 2, "Should make DB calls for both requests"


class TestCacheInvalidation:
    """Test cache invalidation functionality."""

    def test_invalidate_cache_removes_user_entry(self, recommendation_service):
        """Test that invalidate_cache removes user's cached recommendations."""
        # Manually add cache entry
        cache_key = "recommendations:user-123"
        mock_recommendations = [
            ChapterRecommendation(
                user_id='user-123',
                chapter_id='chapter-1',
                relevance_score=0.95,
                recommended_at=datetime.utcnow(),
                reason='Test recommendation'
            )
        ]
        recommendation_service.recommendation_cache[cache_key] = mock_recommendations

        # Verify cache entry exists
        assert cache_key in recommendation_service.recommendation_cache

        # Invalidate cache
        recommendation_service.invalidate_cache('user-123')

        # Verify cache entry removed
        assert cache_key not in recommendation_service.recommendation_cache

    def test_invalidate_cache_no_entry(self, recommendation_service):
        """Test that invalidate_cache handles non-existent entries gracefully."""
        # Invalidate non-existent cache entry (should not raise error)
        recommendation_service.invalidate_cache('non-existent-user')

        # Verify no errors occurred (test passes if no exception)
        assert True

    @pytest.mark.asyncio
    async def test_cache_invalidation_forces_recomputation(self, recommendation_service, mock_db_pool):
        """Test that invalidating cache forces recomputation on next request."""
        mock_conn = AsyncMock()

        # Mock skill classification
        mock_conn.fetchrow.return_value = {
            'skill_level': 'beginner',
            'based_on_profile': {
                'learning_goal': 'hobby_exploration',
                'hardware_access': 'physical'
            }
        }

        # Mock database responses
        mock_conn.fetch.side_effect = [
            [],  # No completed chapters - first call
            [  # Chapter metadata - first call
                {
                    'chapter_id': 'chapter-1',
                    'module_number': 1,
                    'title': 'Original Chapter',
                    'difficulty_level': 'beginner',
                    'prerequisites': [],
                    'requires_hardware': False,
                    'learning_goal_tags': ['theoretical']
                }
            ],
            [],  # No completed chapters - second call after invalidation
            [  # Chapter metadata - second call
                {
                    'chapter_id': 'chapter-2',
                    'module_number': 2,
                    'title': 'Updated Chapter',
                    'difficulty_level': 'beginner',
                    'prerequisites': [],
                    'requires_hardware': False,
                    'learning_goal_tags': ['theoretical']
                }
            ]
        ]

        mock_db_pool.acquire.return_value.__aenter__.return_value = mock_conn

        # First call - cache miss
        recs1, from_cache1 = await recommendation_service.get_recommendations('user-1', force_refresh=False)
        assert not from_cache1, "First call should be cache miss"

        # Invalidate cache
        recommendation_service.invalidate_cache('user-1')

        # Third call after invalidation - should recompute
        recs3, from_cache3 = await recommendation_service.get_recommendations('user-1', force_refresh=False)

        # Verify recomputation happened
        assert not from_cache3, "After invalidation, should recompute"
        assert mock_conn.fetchrow.call_count == 2, "Should make DB calls for both get_recommendations"


if __name__ == "__main__":
    # Run tests with pytest
    pytest.main([__file__, "-v"])
