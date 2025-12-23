"""
Unit tests for ProgressTrackingService.
Tests chapter progress state transitions and CRUD operations.
"""

import pytest
from uuid import uuid4
from unittest.mock import AsyncMock
from datetime import datetime
from app.services.progress_tracking_service import ProgressTrackingService
from app.models.chapter_progress import ChapterProgressResponse


class TestMarkStarted:
    """Test marking chapters as started."""

    @pytest.mark.asyncio
    async def test_mark_started_new_chapter(self):
        """Test marking a new chapter as started creates progress record."""
        user_id = uuid4()
        chapter_id = "module-1/ros-intro"
        mock_conn = AsyncMock()

        # Mock database response for INSERT
        record_id = uuid4()
        mock_conn.fetchrow = AsyncMock(
            return_value={
                "id": record_id,
                "user_id": user_id,
                "chapter_id": chapter_id,
                "status": "started",
                "is_bookmarked": False,
                "started_at": datetime.now(),
                "completed_at": None,
                "updated_at": datetime.now(),
            }
        )

        result = await ProgressTrackingService.mark_started(
            user_id, chapter_id, mock_conn
        )

        assert result.id == record_id
        assert result.user_id == user_id
        assert result.chapter_id == chapter_id
        assert result.status == "started"
        assert result.is_bookmarked is False
        assert result.completed_at is None

    @pytest.mark.asyncio
    async def test_mark_started_idempotent(self):
        """Test marking same chapter as started again is idempotent."""
        user_id = uuid4()
        chapter_id = "module-1/ros-intro"
        mock_conn = AsyncMock()

        record_id = uuid4()
        started_time = datetime.now()

        # Mock ON CONFLICT DO UPDATE scenario
        mock_conn.fetchrow = AsyncMock(
            return_value={
                "id": record_id,
                "user_id": user_id,
                "chapter_id": chapter_id,
                "status": "started",
                "is_bookmarked": False,
                "started_at": started_time,
                "completed_at": None,
                "updated_at": datetime.now(),
            }
        )

        result = await ProgressTrackingService.mark_started(
            user_id, chapter_id, mock_conn
        )

        assert result.id == record_id
        assert result.status == "started"
        assert result.started_at == started_time


class TestMarkCompleted:
    """Test marking chapters as completed."""

    @pytest.mark.asyncio
    async def test_mark_completed_from_started(self):
        """Test marking started chapter as completed updates status."""
        user_id = uuid4()
        chapter_id = "module-1/ros-intro"
        mock_conn = AsyncMock()

        record_id = uuid4()
        started_time = datetime.now()
        completed_time = datetime.now()

        # Mock database response for UPDATE
        mock_conn.fetchrow = AsyncMock(
            return_value={
                "id": record_id,
                "user_id": user_id,
                "chapter_id": chapter_id,
                "status": "completed",
                "is_bookmarked": False,
                "started_at": started_time,
                "completed_at": completed_time,
                "updated_at": datetime.now(),
            }
        )

        result = await ProgressTrackingService.mark_completed(
            user_id, chapter_id, mock_conn
        )

        assert result.status == "completed"
        assert result.completed_at == completed_time
        assert result.started_at == started_time

    @pytest.mark.asyncio
    async def test_mark_completed_directly(self):
        """Test marking chapter as completed without prior started state."""
        user_id = uuid4()
        chapter_id = "module-2/ros-publishers"
        mock_conn = AsyncMock()

        record_id = uuid4()
        now = datetime.now()

        # Mock INSERT with completed status
        mock_conn.fetchrow = AsyncMock(
            return_value={
                "id": record_id,
                "user_id": user_id,
                "chapter_id": chapter_id,
                "status": "completed",
                "is_bookmarked": False,
                "started_at": now,
                "completed_at": now,
                "updated_at": now,
            }
        )

        result = await ProgressTrackingService.mark_completed(
            user_id, chapter_id, mock_conn
        )

        assert result.status == "completed"
        assert result.completed_at is not None


class TestToggleBookmark:
    """Test bookmark toggling functionality."""

    @pytest.mark.asyncio
    async def test_toggle_bookmark_on_existing_unbookmarked(self):
        """Test toggling bookmark ON for existing unbookmarked chapter."""
        user_id = uuid4()
        chapter_id = "module-1/ros-intro"
        mock_conn = AsyncMock()

        record_id = uuid4()

        # Mock sequence: fetchrow (check exists) -> fetchrow (update)
        mock_conn.fetchrow = AsyncMock(
            side_effect=[
                # First call: check existing
                {"is_bookmarked": False},
                # Second call: update result
                {
                    "id": record_id,
                    "user_id": user_id,
                    "chapter_id": chapter_id,
                    "status": "started",
                    "is_bookmarked": True,  # Toggled ON
                    "started_at": datetime.now(),
                    "completed_at": None,
                    "updated_at": datetime.now(),
                },
            ]
        )

        result = await ProgressTrackingService.toggle_bookmark(
            user_id, chapter_id, mock_conn
        )

        assert result.is_bookmarked is True

    @pytest.mark.asyncio
    async def test_toggle_bookmark_off_existing_bookmarked(self):
        """Test toggling bookmark OFF for existing bookmarked chapter."""
        user_id = uuid4()
        chapter_id = "module-1/ros-intro"
        mock_conn = AsyncMock()

        record_id = uuid4()

        # Mock sequence: fetchrow (check exists) -> fetchrow (update)
        mock_conn.fetchrow = AsyncMock(
            side_effect=[
                # First call: check existing
                {"is_bookmarked": True},
                # Second call: update result
                {
                    "id": record_id,
                    "user_id": user_id,
                    "chapter_id": chapter_id,
                    "status": "started",
                    "is_bookmarked": False,  # Toggled OFF
                    "started_at": datetime.now(),
                    "completed_at": None,
                    "updated_at": datetime.now(),
                },
            ]
        )

        result = await ProgressTrackingService.toggle_bookmark(
            user_id, chapter_id, mock_conn
        )

        assert result.is_bookmarked is False

    @pytest.mark.asyncio
    async def test_toggle_bookmark_new_chapter_creates_with_bookmark(self):
        """Test toggling bookmark on non-existent chapter creates it with bookmark."""
        user_id = uuid4()
        chapter_id = "module-2/ros-subscribers"
        mock_conn = AsyncMock()

        record_id = uuid4()

        # Mock sequence: fetchrow (not exists) -> fetchrow (insert)
        mock_conn.fetchrow = AsyncMock(
            side_effect=[
                # First call: check existing (returns None)
                None,
                # Second call: insert new record with bookmark
                {
                    "id": record_id,
                    "user_id": user_id,
                    "chapter_id": chapter_id,
                    "status": "started",
                    "is_bookmarked": True,  # Created with bookmark
                    "started_at": datetime.now(),
                    "completed_at": None,
                    "updated_at": datetime.now(),
                },
            ]
        )

        result = await ProgressTrackingService.toggle_bookmark(
            user_id, chapter_id, mock_conn
        )

        assert result.is_bookmarked is True
        assert result.status == "started"


class TestGetUserProgress:
    """Test retrieving user progress records."""

    @pytest.mark.asyncio
    async def test_get_user_progress_all(self):
        """Test retrieving all progress records for a user."""
        user_id = uuid4()
        mock_conn = AsyncMock()

        # Mock multiple progress records
        mock_conn.fetch = AsyncMock(
            return_value=[
                {
                    "id": uuid4(),
                    "user_id": user_id,
                    "chapter_id": "module-1/ros-intro",
                    "status": "completed",
                    "is_bookmarked": True,
                    "started_at": datetime.now(),
                    "completed_at": datetime.now(),
                    "updated_at": datetime.now(),
                },
                {
                    "id": uuid4(),
                    "user_id": user_id,
                    "chapter_id": "module-2/ros-publishers",
                    "status": "started",
                    "is_bookmarked": False,
                    "started_at": datetime.now(),
                    "completed_at": None,
                    "updated_at": datetime.now(),
                },
            ]
        )

        results = await ProgressTrackingService.get_user_progress(user_id, mock_conn)

        assert len(results) == 2
        assert results[0].status == "completed"
        assert results[1].status == "started"

    @pytest.mark.asyncio
    async def test_get_user_progress_filter_by_status(self):
        """Test filtering progress by status."""
        user_id = uuid4()
        mock_conn = AsyncMock()

        # Mock only completed chapters
        mock_conn.fetch = AsyncMock(
            return_value=[
                {
                    "id": uuid4(),
                    "user_id": user_id,
                    "chapter_id": "module-1/ros-intro",
                    "status": "completed",
                    "is_bookmarked": False,
                    "started_at": datetime.now(),
                    "completed_at": datetime.now(),
                    "updated_at": datetime.now(),
                }
            ]
        )

        results = await ProgressTrackingService.get_user_progress(
            user_id, mock_conn, status="completed"
        )

        assert len(results) == 1
        assert results[0].status == "completed"

    @pytest.mark.asyncio
    async def test_get_user_progress_bookmarked_only(self):
        """Test filtering progress for bookmarked chapters only."""
        user_id = uuid4()
        mock_conn = AsyncMock()

        # Mock only bookmarked chapters
        mock_conn.fetch = AsyncMock(
            return_value=[
                {
                    "id": uuid4(),
                    "user_id": user_id,
                    "chapter_id": "module-1/gazebo",
                    "status": "started",
                    "is_bookmarked": True,
                    "started_at": datetime.now(),
                    "completed_at": None,
                    "updated_at": datetime.now(),
                }
            ]
        )

        results = await ProgressTrackingService.get_user_progress(
            user_id, mock_conn, bookmarked_only=True
        )

        assert len(results) == 1
        assert results[0].is_bookmarked is True

    @pytest.mark.asyncio
    async def test_get_user_progress_empty(self):
        """Test retrieving progress when user has no progress records."""
        user_id = uuid4()
        mock_conn = AsyncMock()

        mock_conn.fetch = AsyncMock(return_value=[])

        results = await ProgressTrackingService.get_user_progress(user_id, mock_conn)

        assert len(results) == 0


class TestGetProgressByChapter:
    """Test retrieving progress for specific chapter."""

    @pytest.mark.asyncio
    async def test_get_progress_by_chapter_exists(self):
        """Test retrieving progress for specific chapter that exists."""
        user_id = uuid4()
        chapter_id = "module-1/ros-intro"
        mock_conn = AsyncMock()

        record_id = uuid4()

        mock_conn.fetchrow = AsyncMock(
            return_value={
                "id": record_id,
                "user_id": user_id,
                "chapter_id": chapter_id,
                "status": "completed",
                "is_bookmarked": True,
                "started_at": datetime.now(),
                "completed_at": datetime.now(),
                "updated_at": datetime.now(),
            }
        )

        result = await ProgressTrackingService.get_progress_by_chapter(
            user_id, chapter_id, mock_conn
        )

        assert result is not None
        assert result.chapter_id == chapter_id
        assert result.status == "completed"

    @pytest.mark.asyncio
    async def test_get_progress_by_chapter_not_exists(self):
        """Test retrieving progress for chapter with no progress record."""
        user_id = uuid4()
        chapter_id = "module-3/advanced-control"
        mock_conn = AsyncMock()

        mock_conn.fetchrow = AsyncMock(return_value=None)

        result = await ProgressTrackingService.get_progress_by_chapter(
            user_id, chapter_id, mock_conn
        )

        assert result is None


class TestStateTransitions:
    """Test valid state transitions for chapter progress."""

    @pytest.mark.asyncio
    async def test_state_transition_started_to_completed(self):
        """Test valid transition from started to completed."""
        user_id = uuid4()
        chapter_id = "module-1/ros-intro"
        mock_conn = AsyncMock()

        record_id = uuid4()
        started_time = datetime.now()

        # First mark as started
        mock_conn.fetchrow = AsyncMock(
            return_value={
                "id": record_id,
                "user_id": user_id,
                "chapter_id": chapter_id,
                "status": "started",
                "is_bookmarked": False,
                "started_at": started_time,
                "completed_at": None,
                "updated_at": datetime.now(),
            }
        )

        result_started = await ProgressTrackingService.mark_started(
            user_id, chapter_id, mock_conn
        )
        assert result_started.status == "started"

        # Then mark as completed
        completed_time = datetime.now()
        mock_conn.fetchrow = AsyncMock(
            return_value={
                "id": record_id,
                "user_id": user_id,
                "chapter_id": chapter_id,
                "status": "completed",
                "is_bookmarked": False,
                "started_at": started_time,
                "completed_at": completed_time,
                "updated_at": datetime.now(),
            }
        )

        result_completed = await ProgressTrackingService.mark_completed(
            user_id, chapter_id, mock_conn
        )
        assert result_completed.status == "completed"
        assert result_completed.completed_at is not None

    @pytest.mark.asyncio
    async def test_bookmark_independent_of_status(self):
        """Test that bookmark can be toggled regardless of completion status."""
        user_id = uuid4()
        chapter_id = "module-1/ros-intro"
        mock_conn = AsyncMock()

        record_id = uuid4()

        # Test bookmarking a completed chapter
        mock_conn.fetchrow = AsyncMock(
            side_effect=[
                # Check existing (completed, not bookmarked)
                {"is_bookmarked": False},
                # Update result (completed, now bookmarked)
                {
                    "id": record_id,
                    "user_id": user_id,
                    "chapter_id": chapter_id,
                    "status": "completed",
                    "is_bookmarked": True,
                    "started_at": datetime.now(),
                    "completed_at": datetime.now(),
                    "updated_at": datetime.now(),
                },
            ]
        )

        result = await ProgressTrackingService.toggle_bookmark(
            user_id, chapter_id, mock_conn
        )

        assert result.status == "completed"
        assert result.is_bookmarked is True
