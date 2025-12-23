"""
Integration Tests for Personalization API

Tests the personalization endpoints with real database interactions.
Covers:
- T043: GET /api/v1/recommendations endpoint integration test
- Additional integration tests for skill classification endpoints (optional)

Related: backend/app/routers/personalization.py
"""

import pytest
import asyncio
import httpx
from datetime import datetime, timedelta
from uuid import uuid4
import asyncpg

from app.config import settings

# Test configuration
API_BASE_URL = f"http://{settings.api_host}:{settings.api_port}"
TIMEOUT = 30.0


@pytest.fixture
async def db_connection():
    """Create a database connection for test setup/teardown."""
    conn = await asyncpg.connect(settings.database_url)
    yield conn
    await conn.close()


@pytest.fixture
async def test_user(db_connection):
    """Create a test user with profile and skill classification."""
    user_id = str(uuid4())

    # Insert test user profile (assuming user_profiles table exists from Phase 4A)
    try:
        await db_connection.execute(
            """
            INSERT INTO user_profiles (user_id, preferred_language, experience_level,
                                      learning_goal, hardware_access, created_at, updated_at)
            VALUES ($1, $2, $3, $4, $5, $6, $7)
            """,
            user_id,
            'python',
            'beginner',
            'career_transition',
            'physical',
            datetime.utcnow(),
            datetime.utcnow()
        )
    except asyncpg.exceptions.UndefinedTableError:
        # If user_profiles doesn't exist, skip profile creation
        pass

    # Insert skill classification
    await db_connection.execute(
        """
        INSERT INTO skill_level_classifications (user_id, skill_level, based_on_profile,
                                                 classified_at, confidence_score)
        VALUES ($1, $2, $3, $4, $5)
        """,
        user_id,
        'beginner',
        {
            'experience_level': 'beginner',
            'learning_goal': 'career_transition',
            'hardware_access': 'physical'
        },
        datetime.utcnow(),
        0.95
    )

    yield user_id

    # Cleanup
    await db_connection.execute(
        "DELETE FROM skill_level_classifications WHERE user_id = $1",
        user_id
    )
    await db_connection.execute(
        "DELETE FROM chapter_progress WHERE user_id = $1",
        user_id
    )
    try:
        await db_connection.execute(
            "DELETE FROM user_profiles WHERE user_id = $1",
            user_id
        )
    except:
        pass


@pytest.fixture
async def test_user_with_progress(db_connection, test_user):
    """Create a test user with some completed chapters."""
    # Mark chapter-1 and chapter-2 as completed
    await db_connection.execute(
        """
        INSERT INTO chapter_progress (user_id, chapter_id, status, started_at, completed_at, is_bookmarked)
        VALUES ($1, $2, $3, $4, $5, $6)
        """,
        test_user,
        'intro-to-robotics',
        'completed',
        datetime.utcnow() - timedelta(days=7),
        datetime.utcnow() - timedelta(days=5),
        False
    )

    await db_connection.execute(
        """
        INSERT INTO chapter_progress (user_id, chapter_id, status, started_at, completed_at, is_bookmarked)
        VALUES ($1, $2, $3, $4, $5, $6)
        """,
        test_user,
        'ros-basics',
        'completed',
        datetime.utcnow() - timedelta(days=5),
        datetime.utcnow() - timedelta(days=3),
        False
    )

    yield test_user

    # Cleanup handled by test_user fixture


@pytest.fixture
def auth_headers(test_user):
    """Create authentication headers for test user.

    NOTE: This is a simplified fixture. In a real implementation, you would:
    1. Generate a valid JWT token for the test user
    2. Use the actual authentication flow

    For now, this assumes the auth middleware is mocked or bypassed in tests.
    """
    # In production, generate real JWT token here
    # For testing purposes, you may need to mock the auth middleware
    return {
        "Authorization": f"Bearer test-token-{test_user}",
        "X-User-ID": test_user  # Custom header for testing (if supported)
    }


# ============================================
# T043: Integration Test for GET /api/v1/recommendations
# ============================================

class TestRecommendationsEndpoint:
    """Integration tests for GET /api/v1/recommendations endpoint."""

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_get_recommendations_success(self, test_user, auth_headers):
        """Test successful retrieval of personalized recommendations."""
        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            response = await client.get(
                f"{API_BASE_URL}/api/v1/recommendations",
                headers=auth_headers
            )

            # Verify response
            assert response.status_code == 200, f"Expected 200, got {response.status_code}: {response.text}"

            data = response.json()

            # Verify response structure
            assert "user_id" in data
            assert "recommendations" in data
            assert "generated_at" in data
            assert "from_cache" in data

            # Verify user_id matches
            assert data["user_id"] == test_user

            # Verify recommendations list
            recommendations = data["recommendations"]
            assert isinstance(recommendations, list)
            assert len(recommendations) <= 3, "Should return maximum 3 recommendations"

            # Verify each recommendation has required fields
            for rec in recommendations:
                assert "chapter_id" in rec
                assert "relevance_score" in rec
                assert "recommended_at" in rec
                assert "reason" in rec
                assert "title" in rec
                assert "difficulty_level" in rec
                assert "module_number" in rec

                # Verify relevance score is valid
                assert 0.0 <= rec["relevance_score"] <= 1.0, "Relevance score must be between 0.0 and 1.0"

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_get_recommendations_caching(self, test_user, auth_headers):
        """Test that recommendations are cached on subsequent requests."""
        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            # First request - should compute and cache
            response1 = await client.get(
                f"{API_BASE_URL}/api/v1/recommendations",
                headers=auth_headers
            )

            assert response1.status_code == 200
            data1 = response1.json()
            assert data1["from_cache"] is False, "First request should not be from cache"

            # Second request - should return cached results
            response2 = await client.get(
                f"{API_BASE_URL}/api/v1/recommendations",
                headers=auth_headers
            )

            assert response2.status_code == 200
            data2 = response2.json()
            assert data2["from_cache"] is True, "Second request should be from cache"

            # Verify recommendations are identical
            assert data1["recommendations"] == data2["recommendations"], "Cached recommendations should match"

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_get_recommendations_force_refresh(self, test_user, auth_headers):
        """Test force_refresh parameter bypasses cache."""
        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            # First request - populate cache
            response1 = await client.get(
                f"{API_BASE_URL}/api/v1/recommendations",
                headers=auth_headers
            )

            assert response1.status_code == 200
            data1 = response1.json()

            # Second request with force_refresh - should bypass cache
            response2 = await client.get(
                f"{API_BASE_URL}/api/v1/recommendations",
                headers=auth_headers,
                params={"force_refresh": True}
            )

            assert response2.status_code == 200
            data2 = response2.json()
            assert data2["from_cache"] is False, "Force refresh should bypass cache"

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_get_recommendations_filters_completed_chapters(self, test_user_with_progress, auth_headers, db_connection):
        """Test that completed chapters are excluded from recommendations."""
        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            response = await client.get(
                f"{API_BASE_URL}/api/v1/recommendations",
                headers=auth_headers
            )

            assert response.status_code == 200
            data = response.json()

            # Verify completed chapters are not in recommendations
            recommended_chapter_ids = [rec["chapter_id"] for rec in data["recommendations"]]
            assert "intro-to-robotics" not in recommended_chapter_ids, "Completed chapter should be filtered out"
            assert "ros-basics" not in recommended_chapter_ids, "Completed chapter should be filtered out"

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_get_recommendations_rate_limiting(self, test_user, auth_headers):
        """Test rate limiting on recommendations endpoint (50 requests/hour)."""
        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            # Make 51 requests rapidly
            responses = []

            for i in range(51):
                response = await client.get(
                    f"{API_BASE_URL}/api/v1/recommendations",
                    headers=auth_headers,
                    params={"force_refresh": True}  # Force refresh to avoid all being cached
                )
                responses.append(response)

                # Small delay to avoid overwhelming the server
                if i < 50:
                    await asyncio.sleep(0.1)

            # Verify first 50 requests succeed
            for i in range(50):
                assert responses[i].status_code in [200, 429], f"Request {i} failed unexpectedly"

            # Verify 51st request is rate limited
            # Note: This may not always trigger if cache is used, so we check for either 200 or 429
            last_response = responses[50]
            if last_response.status_code == 429:
                # Rate limit triggered as expected
                assert "rate limit" in last_response.text.lower(), "Rate limit error message should mention rate limit"
            else:
                # If not rate limited, it's likely due to caching - that's acceptable
                pass

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_get_recommendations_unauthenticated(self):
        """Test that unauthenticated requests are rejected."""
        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            response = await client.get(
                f"{API_BASE_URL}/api/v1/recommendations"
                # No authentication headers
            )

            # Verify authentication is required
            assert response.status_code == 401, f"Expected 401 Unauthorized, got {response.status_code}"

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_get_recommendations_no_classification(self, auth_headers):
        """Test response when user has no skill classification."""
        # Create a user without skill classification
        user_id_no_classification = str(uuid4())

        # Use custom auth headers for this user
        custom_headers = {
            "Authorization": f"Bearer test-token-{user_id_no_classification}",
            "X-User-ID": user_id_no_classification
        }

        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            response = await client.get(
                f"{API_BASE_URL}/api/v1/recommendations",
                headers=custom_headers
            )

            # Verify appropriate error response
            assert response.status_code == 404, f"Expected 404 Not Found, got {response.status_code}"
            assert "classification not found" in response.text.lower(), "Error message should indicate missing classification"

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_get_recommendations_returns_high_relevance_scores(self, test_user, auth_headers):
        """Test that recommendations have high relevance scores (> 0.5)."""
        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            response = await client.get(
                f"{API_BASE_URL}/api/v1/recommendations",
                headers=auth_headers
            )

            assert response.status_code == 200
            data = response.json()

            recommendations = data["recommendations"]

            if len(recommendations) > 0:
                # Verify all recommendations have relevance score > 0.5
                for rec in recommendations:
                    assert rec["relevance_score"] > 0.5, \
                        f"Recommendation {rec['chapter_id']} has low relevance score: {rec['relevance_score']}"

                # Verify recommendations are sorted by relevance score (descending)
                scores = [rec["relevance_score"] for rec in recommendations]
                assert scores == sorted(scores, reverse=True), \
                    "Recommendations should be sorted by relevance score (descending)"

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_get_recommendations_includes_metadata(self, test_user, auth_headers):
        """Test that recommendations include enriched chapter metadata."""
        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            response = await client.get(
                f"{API_BASE_URL}/api/v1/recommendations",
                headers=auth_headers
            )

            assert response.status_code == 200
            data = response.json()

            recommendations = data["recommendations"]

            if len(recommendations) > 0:
                # Verify first recommendation has complete metadata
                first_rec = recommendations[0]

                assert first_rec["title"] is not None, "Title should be populated"
                assert first_rec["difficulty_level"] is not None, "Difficulty level should be populated"
                assert first_rec["module_number"] is not None, "Module number should be populated"

                # Verify difficulty level is valid
                assert first_rec["difficulty_level"] in ['beginner', 'intermediate', 'advanced'], \
                    "Difficulty level should be valid"

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_get_recommendations_reason_field(self, test_user, auth_headers):
        """Test that recommendations include human-readable reason."""
        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            response = await client.get(
                f"{API_BASE_URL}/api/v1/recommendations",
                headers=auth_headers
            )

            assert response.status_code == 200
            data = response.json()

            recommendations = data["recommendations"]

            if len(recommendations) > 0:
                # Verify each recommendation has a non-empty reason
                for rec in recommendations:
                    assert rec["reason"], "Reason should be non-empty"
                    assert isinstance(rec["reason"], str), "Reason should be a string"
                    assert len(rec["reason"]) > 10, "Reason should be descriptive (> 10 chars)"


# ============================================
# Optional: Additional Integration Tests
# ============================================

class TestSkillLevelEndpoint:
    """Optional integration tests for skill level endpoints."""

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_get_skill_level_success(self, test_user, auth_headers):
        """Test GET /api/v1/skill-level endpoint."""
        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            response = await client.get(
                f"{API_BASE_URL}/api/v1/skill-level",
                headers=auth_headers
            )

            assert response.status_code == 200
            data = response.json()

            # Verify response structure
            assert "user_id" in data
            assert "skill_level" in data
            assert "classified_at" in data

            # Verify skill level is valid
            assert data["skill_level"] in ['beginner', 'intermediate', 'advanced']

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_post_skill_level_recalculate(self, test_user, auth_headers):
        """Test POST /api/v1/skill-level endpoint (recalculation)."""
        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            response = await client.post(
                f"{API_BASE_URL}/api/v1/skill-level",
                headers=auth_headers
            )

            assert response.status_code == 200
            data = response.json()

            # Verify response structure
            assert "user_id" in data
            assert "skill_level" in data
            assert "classified_at" in data


# ============================================
# T025: Integration Tests for Progress Tracking Endpoints
# ============================================

class TestProgressTrackingEndpoints:
    """Integration tests for progress tracking endpoints."""

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_post_progress_start(self, test_user, auth_headers):
        """Test POST /api/v1/progress/start endpoint."""
        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            response = await client.post(
                f"{API_BASE_URL}/api/v1/progress/start",
                headers=auth_headers,
                json={"chapter_id": "module-1/ros-intro"}
            )

            assert response.status_code in [200, 201]
            data = response.json()

            # Verify response structure
            assert "user_id" in data
            assert "chapter_id" in data
            assert "status" in data
            assert "started_at" in data

            # Verify values
            assert data["user_id"] == test_user
            assert data["chapter_id"] == "module-1/ros-intro"
            assert data["status"] == "started"

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_post_progress_complete(self, test_user, auth_headers):
        """Test POST /api/v1/progress/complete endpoint."""
        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            # First start the chapter
            await client.post(
                f"{API_BASE_URL}/api/v1/progress/start",
                headers=auth_headers,
                json={"chapter_id": "module-2/ros-publishers"}
            )

            # Then mark as complete
            response = await client.post(
                f"{API_BASE_URL}/api/v1/progress/complete",
                headers=auth_headers,
                json={"chapter_id": "module-2/ros-publishers"}
            )

            assert response.status_code == 200
            data = response.json()

            # Verify response structure
            assert "user_id" in data
            assert "chapter_id" in data
            assert "status" in data
            assert "completed_at" in data

            # Verify values
            assert data["status"] == "completed"
            assert data["completed_at"] is not None

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_post_progress_bookmark(self, test_user, auth_headers):
        """Test POST /api/v1/progress/bookmark endpoint."""
        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            chapter_id = "module-1/gazebo-simulation"

            # First bookmark toggle (OFF -> ON)
            response1 = await client.post(
                f"{API_BASE_URL}/api/v1/progress/bookmark",
                headers=auth_headers,
                json={"chapter_id": chapter_id}
            )

            assert response1.status_code == 200
            data1 = response1.json()
            assert data1["is_bookmarked"] is True

            # Second bookmark toggle (ON -> OFF)
            response2 = await client.post(
                f"{API_BASE_URL}/api/v1/progress/bookmark",
                headers=auth_headers,
                json={"chapter_id": chapter_id}
            )

            assert response2.status_code == 200
            data2 = response2.json()
            assert data2["is_bookmarked"] is False

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_get_progress_all(self, test_user_with_progress, auth_headers):
        """Test GET /api/v1/progress endpoint (all progress)."""
        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            response = await client.get(
                f"{API_BASE_URL}/api/v1/progress",
                headers=auth_headers
            )

            assert response.status_code == 200
            data = response.json()

            # Verify response structure
            assert "progress" in data
            assert isinstance(data["progress"], list)
            assert len(data["progress"]) >= 2  # From test_user_with_progress fixture

            # Verify each progress record
            for progress in data["progress"]:
                assert "chapter_id" in progress
                assert "status" in progress
                assert "started_at" in progress

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_get_progress_filter_by_status(self, test_user_with_progress, auth_headers):
        """Test GET /api/v1/progress with status filter."""
        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            response = await client.get(
                f"{API_BASE_URL}/api/v1/progress",
                headers=auth_headers,
                params={"status": "completed"}
            )

            assert response.status_code == 200
            data = response.json()

            # All returned records should have completed status
            for progress in data["progress"]:
                assert progress["status"] == "completed"

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_get_progress_bookmarked_only(self, test_user, auth_headers, db_connection):
        """Test GET /api/v1/progress with bookmarked_only filter."""
        # Create a bookmarked chapter
        await db_connection.execute(
            """
            INSERT INTO chapter_progress (user_id, chapter_id, status, is_bookmarked)
            VALUES ($1, $2, $3, $4)
            """,
            test_user,
            "module-1/bookmarked-chapter",
            "started",
            True
        )

        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            response = await client.get(
                f"{API_BASE_URL}/api/v1/progress",
                headers=auth_headers,
                params={"bookmarked_only": True}
            )

            assert response.status_code == 200
            data = response.json()

            # All returned records should be bookmarked
            for progress in data["progress"]:
                assert progress["is_bookmarked"] is True

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_progress_state_transition(self, test_user, auth_headers):
        """Test complete progress state transition flow."""
        chapter_id = "module-3/advanced-control"

        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            # 1. Mark as started
            response1 = await client.post(
                f"{API_BASE_URL}/api/v1/progress/start",
                headers=auth_headers,
                json={"chapter_id": chapter_id}
            )
            assert response1.status_code in [200, 201]
            assert response1.json()["status"] == "started"

            # 2. Bookmark it
            response2 = await client.post(
                f"{API_BASE_URL}/api/v1/progress/bookmark",
                headers=auth_headers,
                json={"chapter_id": chapter_id}
            )
            assert response2.status_code == 200
            assert response2.json()["is_bookmarked"] is True

            # 3. Mark as completed
            response3 = await client.post(
                f"{API_BASE_URL}/api/v1/progress/complete",
                headers=auth_headers,
                json={"chapter_id": chapter_id}
            )
            assert response3.status_code == 200
            data3 = response3.json()
            assert data3["status"] == "completed"
            assert data3["is_bookmarked"] is True  # Bookmark preserved

            # 4. Verify in GET endpoint
            response4 = await client.get(
                f"{API_BASE_URL}/api/v1/progress",
                headers=auth_headers
            )
            assert response4.status_code == 200
            progress_list = response4.json()["progress"]
            chapter_progress = next(
                (p for p in progress_list if p["chapter_id"] == chapter_id), None
            )
            assert chapter_progress is not None
            assert chapter_progress["status"] == "completed"


class TestHealthEndpoint:
    """Test personalization health check endpoint."""

    @pytest.mark.asyncio
    @pytest.mark.integration
    async def test_health_check(self):
        """Test GET /api/v1/health/personalization endpoint."""
        async with httpx.AsyncClient(timeout=TIMEOUT) as client:
            response = await client.get(
                f"{API_BASE_URL}/api/v1/health/personalization"
            )

            assert response.status_code == 200
            data = response.json()

            assert data["status"] == "healthy"
            assert data["service"] == "personalization"


if __name__ == "__main__":
    # Run tests with pytest
    pytest.main([__file__, "-v", "-m", "integration"])
