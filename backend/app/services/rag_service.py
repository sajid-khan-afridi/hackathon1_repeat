"""
RAG orchestration service that coordinates vector search, LLM generation, and chat history.
"""
import json
import logging
from typing import List, Dict, Any, Optional
from uuid import UUID
from pathlib import Path

from app.models.query import QueryRequest, QueryResponse, SourceCitation, TokenUsage
from app.services.vector_service import vector_service
from app.services.llm_service import llm_service
from app.services.chat_service import chat_service
from app.config import settings

logger = logging.getLogger(__name__)


class RAGService:
    """Service for orchestrating the complete RAG pipeline."""

    # Path to FAQ fallback data
    FAQ_FALLBACK_PATH = Path("static/data/faq-fallback.json")

    # Confidence thresholds
    CONFIDENCE_OFF_TOPIC = 0.2  # Below this = off-topic
    CONFIDENCE_LOW = 0.3  # Between 0.2-0.3 = low confidence warning

    def __init__(self):
        """Initialize RAG service with FAQ fallback data."""
        self.faq_data = self._load_faq_fallback()
        logger.info("RAGService initialized")

    def _load_faq_fallback(self) -> Dict[str, Any]:
        """
        Load FAQ fallback responses and suggested topics.

        Returns:
            Dictionary with fallback_responses, suggested_topics, off_topic_message

        Raises:
            Exception: If FAQ file cannot be loaded
        """
        try:
            with open(self.FAQ_FALLBACK_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
            logger.info(f"Loaded FAQ fallback data from {self.FAQ_FALLBACK_PATH}")
            return data
        except Exception as e:
            logger.error(f"Failed to load FAQ fallback: {str(e)}", exc_info=True)
            # Return minimal fallback if file is missing
            return {
                "fallback_responses": [],
                "suggested_topics": [
                    "ROS 2",
                    "kinematics",
                    "navigation",
                    "Isaac Sim",
                ],
                "off_topic_message": "I can only answer questions about the robotics textbook.",
            }

    async def process_query(self, request: QueryRequest) -> QueryResponse:
        """
        Process a user query through the complete RAG pipeline.

        Pipeline steps:
        1. Load FAQ fallback data
        2. Load conversation history if session_id provided
        3. Generate query embedding
        4. Search vector store with filters
        5. Check if results are relevant (off-topic detection)
        6. Build context from top chunks
        7. Generate answer using LLM with context and history
        8. Calculate confidence score
        9. Create or get session
        10. Save user message and assistant response with citations
        11. Add warning if low confidence
        12. Return response with sources, confidence, session_id, tokens

        Args:
            request: QueryRequest with query, filters, session_id, etc.

        Returns:
            QueryResponse with answer, sources, confidence, session_id, tokens_used

        Raises:
            Exception: If any step in the pipeline fails
        """
        logger.info(f"Processing query: '{request.query[:50]}...'")

        # Step 1: FAQ fallback already loaded in __init__

        # Step 2: Load conversation history if session exists
        conversation_history: List[Dict[str, str]] = []
        if request.session_id:
            try:
                # Verify session exists
                session = await chat_service.get_session(request.session_id)
                if session:
                    conversation_history = await chat_service.get_conversation_history(
                        request.session_id, limit=settings.max_conversation_history
                    )
                    logger.info(
                        f"Loaded {len(conversation_history)} messages from session {request.session_id}"
                    )
                else:
                    logger.warning(
                        f"Session {request.session_id} not found, creating new session"
                    )
                    request.session_id = None  # Will create new session later
            except Exception as e:
                logger.error(f"Failed to load conversation history: {str(e)}")
                # Continue without history
                conversation_history = []

        # Step 3: Generate query embedding
        try:
            query_embedding = await llm_service.generate_embedding(request.query)
        except Exception as e:
            logger.error(f"Embedding generation failed: {str(e)}", exc_info=True)
            raise Exception(f"Failed to generate query embedding: {str(e)}")

        # Step 4: Search vector store with filters
        try:
            # Build filters dict from request
            filters = None
            if request.filters:
                filters = {}
                if request.filters.module is not None:
                    filters["module"] = request.filters.module
                if request.filters.difficulty:
                    filters["difficulty"] = request.filters.difficulty
                if request.filters.tags:
                    filters["tags"] = request.filters.tags

            # Perform vector search
            search_results = await vector_service.search_context(
                query_embedding=query_embedding,
                filters=filters,
                top_k=request.top_k or settings.top_k_chunks,
            )

            logger.info(f"Vector search returned {len(search_results)} results")

        except Exception as e:
            logger.error(f"Vector search failed: {str(e)}", exc_info=True)
            # Return FAQ fallback for vector store errors
            return await self._fallback_response(
                request, error_type="vector_store_unavailable"
            )

        # Step 5: Check if results are relevant (off-topic detection)
        if not search_results or all(
            result.relevance_score < self.CONFIDENCE_OFF_TOPIC
            for result in search_results
        ):
            logger.warning("No relevant results found - query appears off-topic")
            return await self._off_topic_response(request)

        # Step 6: Build context from top chunks
        context_parts = []
        for idx, result in enumerate(search_results, start=1):
            context_parts.append(
                f"[Source {idx}: {result.chapter_title}]\n{result.excerpt}\n"
            )

        context = "\n".join(context_parts)
        logger.info(f"Built context from {len(search_results)} chunks ({len(context)} chars)")

        # Step 7: Generate answer using LLM with context and history
        try:
            llm_result = await llm_service.generate_response(
                prompt=request.query, context=context, conversation_history=conversation_history
            )

            answer = llm_result["response"]
            input_tokens = llm_result["input_tokens"]
            output_tokens = llm_result["output_tokens"]
            total_tokens = llm_result["total_tokens"]

        except Exception as e:
            logger.error(f"LLM generation failed: {str(e)}", exc_info=True)
            raise Exception(f"Failed to generate answer: {str(e)}")

        # Step 8: Calculate confidence score (weighted average of relevance scores)
        if search_results:
            # Weight scores by position (earlier results have higher weight)
            total_weight = 0
            weighted_sum = 0
            for idx, result in enumerate(search_results):
                weight = 1.0 / (idx + 1)  # Position weighting: 1, 0.5, 0.33, 0.25, 0.2
                weighted_sum += result.relevance_score * weight
                total_weight += weight

            confidence = weighted_sum / total_weight if total_weight > 0 else 0.0
            confidence = round(confidence, 2)
        else:
            confidence = 0.0

        logger.info(f"Calculated confidence: {confidence}")

        # Step 9: Create or get session
        if request.session_id:
            # Session already verified in step 2
            session_id = request.session_id
        else:
            # Create new session
            try:
                session = await chat_service.create_session(user_id=request.user_id)
                session_id = session["id"]
                logger.info(f"Created new session: {session_id}")
            except Exception as e:
                logger.error(f"Failed to create session: {str(e)}", exc_info=True)
                raise Exception(f"Session creation failed: {str(e)}")

        # Step 10: Save user message and assistant response with citations
        try:
            # Save user message
            user_message_id = await chat_service.save_message(
                session_id=session_id, role="user", content=request.query
            )

            # Save assistant message with confidence and tokens
            assistant_message_id = await chat_service.save_message(
                session_id=session_id,
                role="assistant",
                content=answer,
                confidence=confidence,
                tokens_used={
                    "input": input_tokens,
                    "output": output_tokens,
                    "total": total_tokens,
                },
            )

            # Build citations from search results
            citations = [
                SourceCitation(
                    chapter_id=result.chapter_id,
                    chapter_title=result.chapter_title,
                    relevance_score=result.relevance_score,
                    excerpt=result.excerpt[:500],  # Limit excerpt length
                    position=idx + 1,
                )
                for idx, result in enumerate(search_results)
            ]

            # Save citations
            await chat_service.save_citations(
                message_id=assistant_message_id, citations=citations
            )

            logger.info(
                f"Saved messages and {len(citations)} citations to session {session_id}"
            )

        except Exception as e:
            logger.error(f"Failed to save conversation: {str(e)}", exc_info=True)
            # Don't fail the request if saving fails
            logger.warning("Continuing despite conversation save failure")

        # Step 11: Add warning if low confidence (0.2-0.3)
        filter_message = None
        if self.CONFIDENCE_OFF_TOPIC <= confidence < self.CONFIDENCE_LOW:
            filter_message = (
                "Note: This answer has low confidence. The textbook may not contain "
                "detailed information on this specific topic. Try asking about: "
                + ", ".join(self.faq_data["suggested_topics"][:5])
            )
            logger.info(f"Added low confidence warning (confidence={confidence})")

        # Step 12: Return QueryResponse
        return QueryResponse(
            answer=answer,
            sources=citations,
            confidence=confidence,
            session_id=session_id,
            tokens_used=TokenUsage(
                input_tokens=input_tokens,
                output_tokens=output_tokens,
                total_tokens=total_tokens,
            ),
            filter_message=filter_message,
            suggested_terms=None,  # Only set for off-topic queries
        )

    async def _off_topic_response(self, request: QueryRequest) -> QueryResponse:
        """
        Generate response for off-topic queries with suggested terms.

        Args:
            request: Original QueryRequest

        Returns:
            QueryResponse with off-topic message and suggested terms
        """
        logger.info("Generating off-topic response")

        # Get or create session
        if request.session_id:
            session_id = request.session_id
        else:
            session = await chat_service.create_session(user_id=request.user_id)
            session_id = session["id"]

        # Build off-topic message
        answer = self.faq_data["off_topic_message"]

        # Save user message
        await chat_service.save_message(
            session_id=session_id, role="user", content=request.query
        )

        # Save assistant message
        await chat_service.save_message(
            session_id=session_id,
            role="assistant",
            content=answer,
            confidence=0.0,
        )

        return QueryResponse(
            answer=answer,
            sources=[],
            confidence=0.0,
            session_id=session_id,
            tokens_used=TokenUsage(input_tokens=0, output_tokens=0, total_tokens=0),
            filter_message=None,
            suggested_terms=self.faq_data["suggested_topics"],
        )

    async def _fallback_response(
        self, request: QueryRequest, error_type: str = "general"
    ) -> QueryResponse:
        """
        Generate fallback response when services are unavailable.

        Args:
            request: Original QueryRequest
            error_type: Type of error (vector_store_unavailable, llm_unavailable, etc.)

        Returns:
            QueryResponse with fallback answer
        """
        logger.info(f"Generating fallback response for error: {error_type}")

        # Get or create session
        if request.session_id:
            session_id = request.session_id
        else:
            try:
                session = await chat_service.create_session(user_id=request.user_id)
                session_id = session["id"]
            except Exception:
                # If can't create session, use a placeholder UUID
                from uuid import uuid4

                session_id = uuid4()
                logger.warning("Could not create session, using temporary ID")

        # Build fallback message
        if error_type == "vector_store_unavailable":
            answer = (
                "I'm currently unable to search the textbook content. "
                "Please try again in a few moments. In the meantime, here are some "
                f"topics I can help with: {', '.join(self.faq_data['suggested_topics'][:5])}"
            )
        else:
            answer = (
                "I'm experiencing technical difficulties. Please try again in a few moments."
            )

        # Try to save messages (may fail if database is down)
        try:
            await chat_service.save_message(
                session_id=session_id, role="user", content=request.query
            )
            await chat_service.save_message(
                session_id=session_id, role="assistant", content=answer, confidence=0.0
            )
        except Exception as e:
            logger.error(f"Failed to save fallback messages: {str(e)}")

        return QueryResponse(
            answer=answer,
            sources=[],
            confidence=0.0,
            session_id=session_id,
            tokens_used=TokenUsage(input_tokens=0, output_tokens=0, total_tokens=0),
            filter_message="Service temporarily unavailable",
            suggested_terms=self.faq_data["suggested_topics"],
        )


# Global instance
rag_service = RAGService()
