"""Chat service for managing sessions and messages."""

import json
import logging
from datetime import datetime, timedelta
from typing import List, Optional, Dict, Any
from uuid import UUID
import secrets
import asyncpg
from asyncpg import Connection, Pool

from app.config import get_settings
from app.models.chat import ChatSession, ChatMessage, MessageRole, SessionStats
from app.models.query import SourceCitation

logger = logging.getLogger(__name__)


class ChatService:
    """Service for managing chat sessions and messages."""

    def __init__(self):
        """Initialize the chat service."""
        self.settings = get_settings()
        self._pool: Optional[Pool] = None

    async def initialize(self) -> None:
        """Initialize the database connection pool."""
        if self._pool is None:
            self._pool = await asyncpg.create_pool(
                self.settings.database_url,
                min_size=2,
                max_size=self.settings.db_pool_size,
                command_timeout=60,
            )
            logger.info("Initialized database connection pool")

    @property
    def pool(self) -> Pool:
        """Get or create the database connection pool."""
        if self._pool is None:
            raise RuntimeError("ChatService not initialized. Call initialize() first.")
        return self._pool

    async def create_session(
        self, user_id: Optional[str] = None, metadata: Optional[Dict[str, Any]] = None
    ) -> ChatSession:
        """Create a new chat session."""
        async with self.pool.acquire() as conn:
            session_id = str(await conn.fetchval("SELECT gen_random_uuid()"))
            session_token = secrets.token_urlsafe(48)

            # Insert session
            await conn.execute(
                """
                INSERT INTO chat_sessions
                (id, user_id, session_token, metadata)
                VALUES ($1, $2, $3, $4)
                """,
                session_id,
                user_id,
                session_token,
                json.dumps(metadata or {}),
            )

            # Fetch and return the created session
            row = await conn.fetchrow(
                """
                SELECT id, user_id, session_token, created_at, last_activity_at,
                       metadata, is_active, COALESCE(message_count, 0) as message_count
                FROM chat_sessions
                WHERE id = $1
                """,
                session_id,
            )

            logger.info(f"Created new session {session_id}")
            return self._row_to_session(row)

    async def get_session(
        self, session_id: Optional[str] = None, session_token: Optional[str] = None
    ) -> Optional[ChatSession]:
        """Get a session by ID or token."""
        if not session_id and not session_token:
            return None

        async with self.pool.acquire() as conn:
            if session_id:
                row = await conn.fetchrow(
                    """
                    SELECT id, user_id, session_token, created_at, last_activity_at,
                           metadata, is_active, COALESCE(message_count, 0) as message_count
                    FROM chat_sessions
                    WHERE id = $1
                    """,
                    session_id,
                )
            else:
                row = await conn.fetchrow(
                    """
                    SELECT id, user_id, session_token, created_at, last_activity_at,
                           metadata, is_active, COALESCE(message_count, 0) as message_count
                    FROM chat_sessions
                    WHERE session_token = $1
                    """,
                    session_token,
                )

            return self._row_to_session(row) if row else None

    async def update_session_activity(self, session_id: str) -> None:
        """Update the last activity timestamp for a session."""
        async with self.pool.acquire() as conn:
            await conn.execute(
                """
                UPDATE chat_sessions
                SET last_activity_at = NOW()
                WHERE id = $1
                """,
                session_id,
            )

    async def add_message(
        self,
        session_id: str,
        role: str | MessageRole,
        content: str,
        token_usage: Optional[Dict[str, int]] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> ChatMessage:
        """Add a message to a session."""
        async with self.pool.acquire() as conn:
            message_id = str(await conn.fetchval("SELECT gen_random_uuid()"))

            # Handle role as string or enum
            role_value = role.value if isinstance(role, MessageRole) else role

            # Insert message
            await conn.execute(
                """
                INSERT INTO chat_messages
                (id, session_id, role, content, token_usage, metadata)
                VALUES ($1, $2, $3, $4, $5, $6)
                """,
                message_id,
                session_id,
                role_value,
                content,
                json.dumps(token_usage) if token_usage else None,
                json.dumps(metadata or {}),
            )

            # Fetch and return the created message
            row = await conn.fetchrow(
                """
                SELECT id, session_id, role, content, timestamp,
                       token_usage, metadata
                FROM chat_messages
                WHERE id = $1
                """,
                message_id,
            )

            logger.debug(f"Added {role_value} message to session {session_id}")
            return self._row_to_message(row)

    async def get_messages(self, session_id: str, limit: Optional[int] = None) -> List[ChatMessage]:
        """Get messages for a session."""
        async with self.pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT id, session_id, role, content, timestamp,
                       token_usage, metadata
                FROM chat_messages
                WHERE session_id = $1
                ORDER BY timestamp ASC
                LIMIT $2
                """,
                session_id,
                limit
                or self.settings.chat_history_limit * 2
                + 1,  # Include both user and assistant messages
            )

            return [self._row_to_message(row) for row in rows]

    async def get_conversation_context(
        self, session_id: str, limit: int = 5
    ) -> List[Dict[str, str]]:
        """Get conversation history for context in RAG."""
        async with self.pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT role, content
                FROM chat_messages
                WHERE session_id = $1
                ORDER BY timestamp DESC
                LIMIT $2
                """,
                session_id,
                limit * 2,  # Get pairs of messages
            )

            # Convert to list and reverse to get chronological order
            context = [{"role": row["role"], "content": row["content"]} for row in reversed(rows)]

            return context

    async def add_citations(self, message_id: str, sources: List[SourceCitation]) -> None:
        """Add source citations for a message."""
        async with self.pool.acquire() as conn:
            for source in sources:
                await conn.execute(
                    """
                    INSERT INTO source_citations
                    (message_id, chapter_id, chapter_title, section, module,
                     module_title, content, relevance_score, page_url, tags,
                     difficulty, position)
                    VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12)
                    """,
                    message_id,
                    source.chapter,
                    getattr(source, "chapter_title", ""),
                    getattr(source, "section", ""),
                    source.module,
                    getattr(source, "module_title", ""),
                    source.content,
                    source.relevance_score,
                    source.page_url,
                    [],
                    None,  # difficulty
                    None,  # position
                )

    async def delete_session(self, session_id: str) -> bool:
        """Delete a session and all its messages."""
        async with self.pool.acquire() as conn:
            # Delete session (messages will be cascade deleted)
            result = await conn.execute("DELETE FROM chat_sessions WHERE id = $1", session_id)

            deleted = int(result.split()[-1]) if result else 0
            logger.info(f"Deleted session {session_id}: {deleted} rows affected")
            return deleted > 0

    async def get_session_stats(self, session_id: str) -> Optional[SessionStats]:
        """Get statistics for a session."""
        async with self.pool.acquire() as conn:
            row = await conn.fetchrow(
                """
                SELECT
                    s.id as session_id,
                    COUNT(m.id) as message_count,
                    COUNT(CASE WHEN m.role = 'user' THEN 1 END) as user_messages,
                    COUNT(CASE WHEN m.role = 'assistant' THEN 1 END) as assistant_messages,
                    COALESCE(SUM(
                        COALESCE((m.token_usage->>'total_tokens')::int, 0)
                    ), 0) as total_tokens,
                    s.created_at as first_message_at,
                    MAX(m.timestamp) as last_message_at
                FROM chat_sessions s
                LEFT JOIN chat_messages m ON s.id = m.session_id
                WHERE s.id = $1
                GROUP BY s.id
                """,
                session_id,
            )

            if not row:
                return None

            # Calculate session duration
            duration = None
            if row["first_message_at"] and row["last_message_at"]:
                duration = (row["last_message_at"] - row["first_message_at"]).total_seconds()

            return SessionStats(
                sessionId=row["session_id"],
                messageCount=row["message_count"],
                userMessages=row["user_messages"],
                assistantMessages=row["assistant_messages"],
                totalTokens=row["total_tokens"],
                sessionDuration=duration,
                firstMessageAt=row["first_message_at"],
                lastMessageAt=row["last_message_at"],
            )

    def _row_to_session(self, row) -> ChatSession:
        """Convert database row to ChatSession."""
        # Parse metadata if it's a string
        metadata = row["metadata"]
        if isinstance(metadata, str):
            metadata = json.loads(metadata) if metadata else {}
        return ChatSession(
            id=str(row["id"]),
            sessionToken=row["session_token"],
            userId=row["user_id"],
            createdAt=row["created_at"],
            lastActivityAt=row["last_activity_at"],
            messageCount=row["message_count"],
            isActive=row["is_active"],
            metadata=metadata,
        )

    def _row_to_message(self, row) -> ChatMessage:
        """Convert database row to ChatMessage."""
        # Parse token_usage if it's a string
        token_usage = row["token_usage"]
        if isinstance(token_usage, str):
            token_usage = json.loads(token_usage) if token_usage else None
        # Parse metadata if it's a string
        metadata = row["metadata"]
        if isinstance(metadata, str):
            metadata = json.loads(metadata) if metadata else {}
        return ChatMessage(
            id=str(row["id"]),
            sessionId=str(row["session_id"]),
            role=MessageRole(row["role"]),
            content=row["content"],
            timestamp=row["timestamp"],
            tokenUsage=token_usage,
            metadata=metadata,
        )

    async def cleanup_old_sessions(self, days: int = 30) -> int:
        """Clean up sessions older than specified days."""
        async with self.pool.acquire() as conn:
            result = await conn.fetchval("SELECT cleanup_old_sessions($1)", days)
            logger.info(f"Cleaned up {result} old sessions")
            return result

    async def close(self) -> None:
        """Close the database connection pool."""
        if self._pool:
            await self._pool.close()
            self._pool = None
            logger.info("Closed database connection pool")


# Global chat service instance
chat_service = ChatService()
