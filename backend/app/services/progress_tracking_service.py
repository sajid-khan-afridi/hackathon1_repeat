"""
Progress Tracking Service for Phase 4B Personalization Engine.
Handles chapter progress CRUD operations (started/completed/bookmarked).
"""

import logging
from uuid import UUID
from datetime import datetime
from typing import List, Optional
import asyncpg
from app.models.chapter_progress import (
    ChapterProgressResponse,
    ProgressStatus,
)

logger = logging.getLogger(__name__)


class ProgressTrackingService:
    """Service for chapter progress tracking operations."""

    @staticmethod
    async def mark_started(
        user_id: UUID, chapter_id: str, conn: asyncpg.Connection
    ) -> ChapterProgressResponse:
        """
        Mark a chapter as started for a user.
        Creates progress record if doesn't exist, updates if already exists.

        Args:
            user_id: UUID of the user
            chapter_id: Identifier for the chapter
            conn: Database connection

        Returns:
            ChapterProgressResponse with progress record

        Raises:
            asyncpg.PostgresError: If database operation fails
        """
        # Insert or do nothing if already exists (idempotent)
        result = await conn.fetchrow(
            """
            INSERT INTO chapter_progress
                (user_id, chapter_id, status, started_at)
            VALUES ($1, $2, 'started', NOW())
            ON CONFLICT (user_id, chapter_id)
            DO UPDATE SET updated_at = NOW()
            RETURNING id, user_id, chapter_id, status, is_bookmarked,
                      started_at, completed_at, updated_at
            """,
            user_id,
            chapter_id,
        )

        logger.info(
            f"Marked chapter '{chapter_id}' as started for user {user_id}"
        )

        return ChapterProgressResponse(
            id=result["id"],
            user_id=result["user_id"],
            chapter_id=result["chapter_id"],
            status=result["status"],
            is_bookmarked=result["is_bookmarked"],
            started_at=result["started_at"],
            completed_at=result["completed_at"],
            updated_at=result["updated_at"],
        )

    @staticmethod
    async def mark_completed(
        user_id: UUID, chapter_id: str, conn: asyncpg.Connection
    ) -> ChapterProgressResponse:
        """
        Mark a chapter as completed for a user.
        Updates existing progress record or creates new one with completed status.

        Args:
            user_id: UUID of the user
            chapter_id: Identifier for the chapter
            conn: Database connection

        Returns:
            ChapterProgressResponse with updated progress record

        Raises:
            asyncpg.PostgresError: If database operation fails
        """
        # Insert or update to completed status
        result = await conn.fetchrow(
            """
            INSERT INTO chapter_progress
                (user_id, chapter_id, status, started_at, completed_at)
            VALUES ($1, $2, 'completed', NOW(), NOW())
            ON CONFLICT (user_id, chapter_id)
            DO UPDATE SET
                status = 'completed',
                completed_at = NOW(),
                updated_at = NOW()
            RETURNING id, user_id, chapter_id, status, is_bookmarked,
                      started_at, completed_at, updated_at
            """,
            user_id,
            chapter_id,
        )

        logger.info(
            f"Marked chapter '{chapter_id}' as completed for user {user_id}"
        )

        return ChapterProgressResponse(
            id=result["id"],
            user_id=result["user_id"],
            chapter_id=result["chapter_id"],
            status=result["status"],
            is_bookmarked=result["is_bookmarked"],
            started_at=result["started_at"],
            completed_at=result["completed_at"],
            updated_at=result["updated_at"],
        )

    @staticmethod
    async def toggle_bookmark(
        user_id: UUID, chapter_id: str, conn: asyncpg.Connection
    ) -> ChapterProgressResponse:
        """
        Toggle bookmark status for a chapter.
        Creates progress record if doesn't exist.

        Args:
            user_id: UUID of the user
            chapter_id: Identifier for the chapter
            conn: Database connection

        Returns:
            ChapterProgressResponse with updated bookmark status

        Raises:
            asyncpg.PostgresError: If database operation fails
        """
        # Check if progress record exists
        existing = await conn.fetchrow(
            """
            SELECT is_bookmarked FROM chapter_progress
            WHERE user_id = $1 AND chapter_id = $2
            """,
            user_id,
            chapter_id,
        )

        if existing:
            # Toggle existing bookmark
            new_bookmark_status = not existing["is_bookmarked"]
            result = await conn.fetchrow(
                """
                UPDATE chapter_progress
                SET is_bookmarked = $3, updated_at = NOW()
                WHERE user_id = $1 AND chapter_id = $2
                RETURNING id, user_id, chapter_id, status, is_bookmarked,
                          started_at, completed_at, updated_at
                """,
                user_id,
                chapter_id,
                new_bookmark_status,
            )
        else:
            # Create new record with bookmark enabled
            result = await conn.fetchrow(
                """
                INSERT INTO chapter_progress
                    (user_id, chapter_id, status, is_bookmarked)
                VALUES ($1, $2, 'started', TRUE)
                RETURNING id, user_id, chapter_id, status, is_bookmarked,
                          started_at, completed_at, updated_at
                """,
                user_id,
                chapter_id,
            )

        logger.info(
            f"Toggled bookmark for chapter '{chapter_id}' "
            f"(user {user_id}): is_bookmarked={result['is_bookmarked']}"
        )

        return ChapterProgressResponse(
            id=result["id"],
            user_id=result["user_id"],
            chapter_id=result["chapter_id"],
            status=result["status"],
            is_bookmarked=result["is_bookmarked"],
            started_at=result["started_at"],
            completed_at=result["completed_at"],
            updated_at=result["updated_at"],
        )

    @staticmethod
    async def get_user_progress(
        user_id: UUID,
        conn: asyncpg.Connection,
        status: Optional[ProgressStatus] = None,
        bookmarked_only: bool = False,
    ) -> List[ChapterProgressResponse]:
        """
        Get all progress records for a user with optional filtering.

        Args:
            user_id: UUID of the user
            conn: Database connection
            status: Optional filter by status ('started' or 'completed')
            bookmarked_only: If True, only return bookmarked chapters

        Returns:
            List of ChapterProgressResponse records
        """
        # Build query with optional filters
        query = """
            SELECT id, user_id, chapter_id, status, is_bookmarked,
                   started_at, completed_at, updated_at
            FROM chapter_progress
            WHERE user_id = $1
        """
        params = [user_id]

        if status:
            query += " AND status = $2"
            params.append(status)

        if bookmarked_only:
            query += f" AND is_bookmarked = ${len(params) + 1}"
            params.append(True)

        query += " ORDER BY updated_at DESC"

        results = await conn.fetch(query, *params)

        logger.debug(
            f"Retrieved {len(results)} progress records for user {user_id} "
            f"(status={status}, bookmarked_only={bookmarked_only})"
        )

        return [
            ChapterProgressResponse(
                id=row["id"],
                user_id=row["user_id"],
                chapter_id=row["chapter_id"],
                status=row["status"],
                is_bookmarked=row["is_bookmarked"],
                started_at=row["started_at"],
                completed_at=row["completed_at"],
                updated_at=row["updated_at"],
            )
            for row in results
        ]

    @staticmethod
    async def get_progress_by_chapter(
        user_id: UUID, chapter_id: str, conn: asyncpg.Connection
    ) -> Optional[ChapterProgressResponse]:
        """
        Get progress record for a specific chapter and user.

        Args:
            user_id: UUID of the user
            chapter_id: Identifier for the chapter
            conn: Database connection

        Returns:
            ChapterProgressResponse if exists, None otherwise
        """
        result = await conn.fetchrow(
            """
            SELECT id, user_id, chapter_id, status, is_bookmarked,
                   started_at, completed_at, updated_at
            FROM chapter_progress
            WHERE user_id = $1 AND chapter_id = $2
            """,
            user_id,
            chapter_id,
        )

        if not result:
            return None

        return ChapterProgressResponse(
            id=result["id"],
            user_id=result["user_id"],
            chapter_id=result["chapter_id"],
            status=result["status"],
            is_bookmarked=result["is_bookmarked"],
            started_at=result["started_at"],
            completed_at=result["completed_at"],
            updated_at=result["updated_at"],
        )
