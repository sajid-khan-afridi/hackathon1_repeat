"""
Chat history management service using asyncpg for PostgreSQL operations.
"""
from typing import List, Dict, Optional, Any
from uuid import UUID
import asyncpg
from datetime import datetime
import secrets
import json
from app.config import settings
from app.models.query import SourceCitation
import logging

logger = logging.getLogger(__name__)


class ChatService:
    """Service for managing chat sessions, messages, and citations."""

    def __init__(self):
        """Initialize with database connection pool."""
        self.pool: Optional[asyncpg.Pool] = None
        logger.info("ChatService initialized (pool will be created on connect)")

    async def connect(self):
        """Create database connection pool."""
        if self.pool is None:
            try:
                self.pool = await asyncpg.create_pool(
                    dsn=settings.database_url,
                    min_size=2,
                    max_size=10,
                    command_timeout=60,
                )
                logger.info("Database connection pool created")
            except Exception as e:
                logger.error(f"Failed to create database pool: {str(e)}", exc_info=True)
                raise Exception(f"Database connection failed: {str(e)}")

    async def disconnect(self):
        """Close database connection pool."""
        if self.pool:
            await self.pool.close()
            self.pool = None
            logger.info("Database connection pool closed")

    async def create_session(self, user_id: Optional[UUID] = None) -> Dict[str, Any]:
        """
        Create a new chat session.

        Args:
            user_id: Optional authenticated user ID

        Returns:
            Dictionary with session data:
                - id: Session UUID
                - user_id: User UUID (if authenticated)
                - session_token: Secure random token for anonymous sessions
                - created_at: Timestamp
                - last_activity_at: Timestamp

        Raises:
            Exception: If database operation fails
        """
        if not self.pool:
            await self.connect()

        try:
            # Generate secure session token
            session_token = secrets.token_urlsafe(48)

            # Insert new session
            async with self.pool.acquire() as conn:
                row = await conn.fetchrow(
                    """
                    INSERT INTO chat_sessions (user_id, session_token, created_at, last_activity_at)
                    VALUES ($1, $2, NOW(), NOW())
                    RETURNING id, user_id, session_token, created_at, last_activity_at, metadata
                    """,
                    user_id,
                    session_token,
                )

            session = {
                "id": row["id"],
                "user_id": row["user_id"],
                "session_token": row["session_token"],
                "created_at": row["created_at"],
                "last_activity_at": row["last_activity_at"],
                "metadata": row["metadata"],
            }

            logger.info(
                f"Created session {session['id']} "
                f"(user_id={user_id}, token={session_token[:10]}...)"
            )
            return session

        except Exception as e:
            logger.error(f"Failed to create session: {str(e)}", exc_info=True)
            raise Exception(f"Session creation failed: {str(e)}")

    async def get_session(self, session_id: UUID) -> Optional[Dict[str, Any]]:
        """
        Retrieve session by ID.

        Args:
            session_id: Session UUID

        Returns:
            Session dictionary or None if not found

        Raises:
            Exception: If database operation fails
        """
        if not self.pool:
            await self.connect()

        try:
            async with self.pool.acquire() as conn:
                row = await conn.fetchrow(
                    """
                    SELECT id, user_id, session_token, created_at, last_activity_at, metadata
                    FROM chat_sessions
                    WHERE id = $1
                    """,
                    session_id,
                )

            if row is None:
                logger.warning(f"Session {session_id} not found")
                return None

            session = {
                "id": row["id"],
                "user_id": row["user_id"],
                "session_token": row["session_token"],
                "created_at": row["created_at"],
                "last_activity_at": row["last_activity_at"],
                "metadata": row["metadata"],
            }

            logger.info(f"Retrieved session {session_id}")
            return session

        except Exception as e:
            logger.error(f"Failed to get session: {str(e)}", exc_info=True)
            raise Exception(f"Session retrieval failed: {str(e)}")

    async def save_message(
        self,
        session_id: UUID,
        role: str,
        content: str,
        confidence: Optional[float] = None,
        tokens_used: Optional[Dict[str, int]] = None,
    ) -> UUID:
        """
        Save a chat message to the database.

        Args:
            session_id: Parent session UUID
            role: Message role ('user' or 'assistant')
            content: Message text content
            confidence: Optional confidence score (for assistant messages)
            tokens_used: Optional token usage dict {"input": N, "output": M, "total": T}

        Returns:
            UUID of created message

        Raises:
            Exception: If database operation fails
        """
        if not self.pool:
            await self.connect()

        try:
            tokens_dict = tokens_used or {}

            async with self.pool.acquire() as conn:
                # Insert message
                # Convert tokens_dict to JSON string for JSONB column
                tokens_json = json.dumps(tokens_dict)

                message_id = await conn.fetchval(
                    """
                    INSERT INTO chat_messages (session_id, role, content, confidence, tokens_used, created_at)
                    VALUES ($1, $2, $3, $4, $5::jsonb, NOW())
                    RETURNING id
                    """,
                    session_id,
                    role,
                    content,
                    confidence,
                    tokens_json,
                )

                # Update session last_activity_at
                await conn.execute(
                    """
                    UPDATE chat_sessions
                    SET last_activity_at = NOW()
                    WHERE id = $1
                    """,
                    session_id,
                )

            logger.info(
                f"Saved message {message_id} "
                f"(session={session_id}, role={role}, confidence={confidence})"
            )
            return message_id

        except Exception as e:
            logger.error(f"Failed to save message: {str(e)}", exc_info=True)
            raise Exception(f"Message save failed: {str(e)}")

    async def save_citations(
        self, message_id: UUID, citations: List[SourceCitation]
    ) -> None:
        """
        Save source citations for a message.

        Args:
            message_id: Parent message UUID
            citations: List of SourceCitation objects

        Raises:
            Exception: If database operation fails
        """
        if not self.pool:
            await self.connect()

        if not citations:
            logger.info(f"No citations to save for message {message_id}")
            return

        try:
            async with self.pool.acquire() as conn:
                # Prepare batch insert data
                values = [
                    (
                        message_id,
                        citation.chapter_id,
                        citation.chapter_title,
                        citation.relevance_score,
                        citation.excerpt,
                        citation.position,
                    )
                    for citation in citations
                ]

                # Batch insert citations
                await conn.executemany(
                    """
                    INSERT INTO source_citations (message_id, chapter_id, chapter_title, relevance_score, excerpt, position)
                    VALUES ($1, $2, $3, $4, $5, $6)
                    """,
                    values,
                )

            logger.info(f"Saved {len(citations)} citations for message {message_id}")

        except Exception as e:
            logger.error(f"Failed to save citations: {str(e)}", exc_info=True)
            raise Exception(f"Citation save failed: {str(e)}")

    async def get_conversation_history(
        self, session_id: UUID, limit: int = 5
    ) -> List[Dict[str, str]]:
        """
        Retrieve recent conversation history for a session.

        Args:
            session_id: Session UUID
            limit: Maximum number of message pairs to return (default 5)

        Returns:
            List of messages in format [{"role": "user|assistant", "content": "..."}]
            Ordered chronologically (oldest first)

        Raises:
            Exception: If database operation fails
        """
        if not self.pool:
            await self.connect()

        try:
            async with self.pool.acquire() as conn:
                rows = await conn.fetch(
                    """
                    SELECT role, content
                    FROM chat_messages
                    WHERE session_id = $1
                    ORDER BY created_at DESC
                    LIMIT $2
                    """,
                    session_id,
                    limit * 2,  # Get last N exchanges (user + assistant pairs)
                )

            # Convert to list of dicts and reverse to chronological order
            messages = [{"role": row["role"], "content": row["content"]} for row in rows]
            messages.reverse()

            logger.info(
                f"Retrieved {len(messages)} messages from session {session_id}"
            )
            return messages

        except Exception as e:
            logger.error(
                f"Failed to get conversation history: {str(e)}", exc_info=True
            )
            raise Exception(f"Conversation history retrieval failed: {str(e)}")

    async def delete_session(self, session_id: UUID) -> bool:
        """
        Delete a chat session and all associated messages/citations.

        Args:
            session_id: Session UUID to delete

        Returns:
            True if session was deleted, False if not found

        Raises:
            Exception: If database operation fails
        """
        if not self.pool:
            await self.connect()

        try:
            async with self.pool.acquire() as conn:
                result = await conn.execute(
                    """
                    DELETE FROM chat_sessions
                    WHERE id = $1
                    """,
                    session_id,
                )

            # Check if any rows were deleted
            deleted = result.split()[-1] == "1"

            if deleted:
                logger.info(f"Deleted session {session_id}")
            else:
                logger.warning(f"Session {session_id} not found for deletion")

            return deleted

        except Exception as e:
            logger.error(f"Failed to delete session: {str(e)}", exc_info=True)
            raise Exception(f"Session deletion failed: {str(e)}")

    async def health_check(self) -> bool:
        """
        Check if database connection is healthy.

        Returns:
            True if healthy, False otherwise
        """
        if not self.pool:
            try:
                await self.connect()
            except Exception:
                return False

        try:
            async with self.pool.acquire() as conn:
                await conn.fetchval("SELECT 1")
            logger.info("Chat service health check passed")
            return True

        except Exception as e:
            logger.error(f"Chat service health check failed: {str(e)}")
            return False


# Global instance
chat_service = ChatService()
