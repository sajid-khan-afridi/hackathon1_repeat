"""
RAG orchestration service that coordinates vector search, LLM generation, and chat history.
"""

import json
import logging
from typing import List, Dict, Any, Optional, AsyncGenerator
from uuid import UUID
from pathlib import Path

from app.models.query import (
    QueryRequest,
    QueryResponse,
    SourceCitation,
    TokenUsage,
    StreamChunk,
)
from app.services.vector_service import vector_service
from app.services.llm_service import llm_service
from app.services.chat_service import chat_service
from app.services.personalization_service import PersonalizationService
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
                    logger.warning(f"Session {request.session_id} not found, creating new session")
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

        # Step 4: Search vector store with adaptive filtering
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

            # Apply adaptive filtering if module filter is present
            search_results, adaptive_filter_message = await self._apply_adaptive_filtering(
                query_embedding=query_embedding,
                filters=filters,
                top_k=request.top_k or settings.top_k_chunks,
            )

            logger.info(
                f"Vector search returned {len(search_results)} results "
                f"(adaptive_filtering={'applied' if adaptive_filter_message else 'none'})"
            )

        except Exception as e:
            logger.error(f"Vector search failed: {str(e)}", exc_info=True)
            # Return FAQ fallback for vector store errors
            return await self._fallback_response(request, error_type="vector_store_unavailable")

        # Step 5: Check if results are relevant (off-topic detection)
        if not search_results or all(
            result.relevance_score < self.CONFIDENCE_OFF_TOPIC for result in search_results
        ):
            logger.warning("No relevant results found - query appears off-topic")
            return await self._off_topic_response(request)

        # Step 6: Build context from top chunks
        context_parts = []
        for idx, result in enumerate(search_results, start=1):
            context_parts.append(f"[Source {idx}: {result.chapter_title}]\n{result.excerpt}\n")

        context = "\n".join(context_parts)
        logger.info(f"Built context from {len(search_results)} chunks ({len(context)} chars)")

        # Step 6.5: Fetch user profile and build profile context (Phase 4B)
        profile_context = None
        if request.user_id:
            try:
                # Use chat_service pool to get database connection
                if chat_service.pool:
                    async with chat_service.pool.acquire() as conn:
                        user_profile_data = await PersonalizationService.get_user_profile_with_classification(
                            request.user_id, conn
                        )
                        if user_profile_data:
                            skill_level = user_profile_data.get("skill_level")
                            profile_context = PersonalizationService.build_profile_context(
                                user_profile_data, skill_level
                            )
                            logger.info(
                                f"Built profile context for user {request.user_id} "
                                f"(skill_level={skill_level})"
                            )
                        else:
                            logger.warning(f"No profile found for user {request.user_id}, using default")
                else:
                    logger.warning("Database pool not available, using default profile context")
            except Exception as e:
                logger.error(f"Failed to fetch user profile: {str(e)}", exc_info=True)
                # Continue with default profile context (None will use LLM's default)

        # If no profile context (unauthenticated or profile fetch failed), use default
        if not profile_context:
            profile_context = PersonalizationService.build_profile_context(None, None)
            logger.debug("Using default intermediate profile context")

        # Step 7: Generate answer using LLM with context, history, and profile (Phase 4B enhanced)
        try:
            llm_result = await llm_service.generate_response(
                prompt=request.query,
                context=context,
                conversation_history=conversation_history,
                profile_context=profile_context,
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
            await chat_service.save_citations(message_id=assistant_message_id, citations=citations)

            logger.info(f"Saved messages and {len(citations)} citations to session {session_id}")

        except Exception as e:
            logger.error(f"Failed to save conversation: {str(e)}", exc_info=True)
            # Don't fail the request if saving fails
            logger.warning("Continuing despite conversation save failure")

        # Step 11: Determine final filter_message (adaptive filtering or low confidence warning)
        filter_message = None

        # Priority 1: Adaptive filtering message (if set from Step 4)
        if adaptive_filter_message:
            filter_message = adaptive_filter_message
            logger.info(f"Using adaptive filter message: {adaptive_filter_message}")
        # Priority 2: Low confidence warning (0.2-0.3)
        elif self.CONFIDENCE_OFF_TOPIC <= confidence < self.CONFIDENCE_LOW:
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

    async def process_query_stream(
        self, request: QueryRequest
    ) -> AsyncGenerator[StreamChunk, None]:
        """
        Process a user query through the RAG pipeline with streaming response.

        This method performs the same pipeline as process_query but yields
        text chunks as they arrive from the LLM, improving perceived performance.

        Pipeline steps:
        1-6: Same as process_query (prepare context)
        7: Stream answer chunks from LLM
        8-10: After streaming complete, calculate confidence and save to DB
        11: Yield final chunk with metadata

        Args:
            request: QueryRequest with query, filters, session_id, etc.

        Yields:
            StreamChunk objects containing:
            - Text chunks during streaming (done=False)
            - Final metadata chunk (done=True) with sources, confidence, session_id, tokens

        Raises:
            Exception: If any step in the pipeline fails
        """
        logger.info(f"Processing streaming query: '{request.query[:50]}...'")

        # Steps 1-6: Prepare context (same as non-streaming)
        conversation_history: List[Dict[str, str]] = []
        if request.session_id:
            try:
                session = await chat_service.get_session(request.session_id)
                if session:
                    conversation_history = await chat_service.get_conversation_history(
                        request.session_id, limit=settings.max_conversation_history
                    )
                else:
                    request.session_id = None
            except Exception as e:
                logger.error(f"Failed to load conversation history: {str(e)}")

        # Generate query embedding
        try:
            query_embedding = await llm_service.generate_embedding(request.query)
        except Exception as e:
            logger.error(f"Embedding generation failed: {str(e)}", exc_info=True)
            raise Exception(f"Failed to generate query embedding: {str(e)}")

        # Search vector store
        try:
            filters = None
            if request.filters:
                filters = {}
                if request.filters.module is not None:
                    filters["module"] = request.filters.module
                if request.filters.difficulty:
                    filters["difficulty"] = request.filters.difficulty
                if request.filters.tags:
                    filters["tags"] = request.filters.tags

            search_results, adaptive_filter_message = await self._apply_adaptive_filtering(
                query_embedding=query_embedding,
                filters=filters,
                top_k=request.top_k or settings.top_k_chunks,
            )
        except Exception as e:
            logger.error(f"Vector search failed: {str(e)}", exc_info=True)
            # Yield error chunk with diagnostic info
            error_msg = f"I'm currently unable to search the textbook content. Error: {str(e)[:100]}"
            logger.error(f"Vector search error details: {str(e)}")
            yield StreamChunk(
                chunk=error_msg,
                done=True,
                sources=[],
                confidence=0.0,
                tokens_used=TokenUsage(
                    input_tokens=0,
                    output_tokens=0,
                    total_tokens=0,
                ),
            )
            return

        # Check if off-topic
        if not search_results or all(
            result.relevance_score < self.CONFIDENCE_OFF_TOPIC for result in search_results
        ):
            logger.warning("No relevant results found - query appears off-topic")
            off_topic_response = await self._off_topic_response(request)
            yield StreamChunk(
                chunk=off_topic_response.answer,
                done=True,
                sources=[],
                confidence=0.0,
                session_id=off_topic_response.session_id,
                tokens_used=TokenUsage(
                    input_tokens=0,
                    output_tokens=0,
                    total_tokens=0,
                ),
            )
            return

        # Build context
        context_parts = []
        for idx, result in enumerate(search_results, start=1):
            context_parts.append(f"[Source {idx}: {result.chapter_title}]\n{result.excerpt}\n")
        context = "\n".join(context_parts)

        # Step 6.5: Fetch user profile and build profile context (Phase 4B)
        profile_context = None
        if request.user_id:
            try:
                if chat_service.pool:
                    async with chat_service.pool.acquire() as conn:
                        user_profile_data = await PersonalizationService.get_user_profile_with_classification(
                            request.user_id, conn
                        )
                        if user_profile_data:
                            skill_level = user_profile_data.get("skill_level")
                            profile_context = PersonalizationService.build_profile_context(
                                user_profile_data, skill_level
                            )
                            logger.info(
                                f"Built profile context for streaming (user={request.user_id}, skill={skill_level})"
                            )
            except Exception as e:
                logger.error(f"Failed to fetch user profile for streaming: {str(e)}", exc_info=True)

        # If no profile context, use default
        if not profile_context:
            profile_context = PersonalizationService.build_profile_context(None, None)
            logger.debug("Using default profile context for streaming")

        # Step 7: Stream answer from LLM with profile context (Phase 4B enhanced)
        full_answer = ""
        try:
            async for chunk_text in llm_service.generate_response_stream(
                prompt=request.query,
                context=context,
                conversation_history=conversation_history,
                profile_context=profile_context,
            ):
                full_answer += chunk_text
                # Yield text chunk
                yield StreamChunk(chunk=chunk_text, done=False)

        except Exception as e:
            logger.error(f"LLM streaming failed: {str(e)}", exc_info=True)
            yield StreamChunk(
                chunk="\n\n[Error: Failed to generate complete response]",
                done=True,
                sources=[],
                confidence=0.0,
                tokens_used=TokenUsage(
                    input_tokens=0,
                    output_tokens=0,
                    total_tokens=0,
                ),
            )
            return

        # Steps 8-10: Calculate confidence and save to DB
        if search_results:
            total_weight = 0
            weighted_sum = 0
            for idx, result in enumerate(search_results):
                weight = 1.0 / (idx + 1)
                weighted_sum += result.relevance_score * weight
                total_weight += weight
            confidence = round(weighted_sum / total_weight, 2) if total_weight > 0 else 0.0
        else:
            confidence = 0.0

        # Create or get session
        if request.session_id:
            session_id = request.session_id
        else:
            try:
                session = await chat_service.create_session(user_id=request.user_id)
                session_id = session["id"]
            except Exception as e:
                logger.error(f"Failed to create session: {str(e)}")
                session_id = None

        # Save conversation (estimate tokens since streaming doesn't return usage)
        estimated_input_tokens = len(context) // 4 + len(request.query) // 4
        estimated_output_tokens = len(full_answer) // 4
        estimated_total_tokens = estimated_input_tokens + estimated_output_tokens

        if session_id:
            try:
                await chat_service.save_message(
                    session_id=session_id, role="user", content=request.query
                )
                assistant_message_id = await chat_service.save_message(
                    session_id=session_id,
                    role="assistant",
                    content=full_answer,
                    confidence=confidence,
                    tokens_used={
                        "input": estimated_input_tokens,
                        "output": estimated_output_tokens,
                        "total": estimated_total_tokens,
                    },
                )

                # Build and save citations
                citations = [
                    SourceCitation(
                        chapter_id=result.chapter_id,
                        chapter_title=result.chapter_title,
                        relevance_score=result.relevance_score,
                        excerpt=result.excerpt[:500],
                        position=idx + 1,
                    )
                    for idx, result in enumerate(search_results)
                ]
                await chat_service.save_citations(
                    message_id=assistant_message_id, citations=citations
                )
            except Exception as e:
                logger.error(f"Failed to save conversation: {str(e)}")

        # Step 11: Yield final chunk with metadata
        citations = [
            SourceCitation(
                chapter_id=result.chapter_id,
                chapter_title=result.chapter_title,
                relevance_score=result.relevance_score,
                excerpt=result.excerpt[:500],
                position=idx + 1,
            )
            for idx, result in enumerate(search_results)
        ]

        yield StreamChunk(
            chunk="",
            done=True,
            sources=citations,
            confidence=confidence,
            session_id=session_id,
            tokens_used=TokenUsage(
                input_tokens=estimated_input_tokens,
                output_tokens=estimated_output_tokens,
                total_tokens=estimated_total_tokens,
            ),
        )

        logger.info(f"Completed streaming query (confidence={confidence})")

    async def _apply_adaptive_filtering(
        self,
        query_embedding: List[float],
        filters: Optional[Dict[str, Any]],
        top_k: int,
    ) -> tuple[List[Any], Optional[str]]:
        """
        Apply confidence-based adaptive filtering for module searches.

        Adaptive filtering logic (from research.md):
        - Confidence >= 0.7 with filter: Strict filter, use filtered results
        - Confidence 0.4-0.7 with filter: Use filtered results + suggest other modules
        - Confidence < 0.4 with filter: Ignore filter, search all, explain to user
        - No filter: Normal search

        Args:
            query_embedding: Query vector embedding
            filters: Optional filters dict (module, difficulty, tags)
            top_k: Number of results to return

        Returns:
            Tuple of (search_results, filter_message)
            - search_results: List of VectorSearchResult objects
            - filter_message: Optional message explaining adaptive filtering behavior
        """
        # No filter or no module filter? Normal search
        if not filters or "module" not in filters or filters["module"] is None:
            results = await vector_service.search_context(
                query_embedding=query_embedding,
                filters=filters,
                top_k=top_k,
            )
            logger.info(f"No module filter applied - normal search")
            return results, None

        module_num = filters["module"]
        logger.info(f"Applying adaptive filtering for Module {module_num}")

        # Step 1: Try with filter first
        filtered_results = await vector_service.search_context(
            query_embedding=query_embedding,
            filters=filters,
            top_k=top_k,
        )

        # Calculate confidence from filtered results
        filtered_confidence = self._calculate_confidence_from_results(filtered_results)
        logger.info(
            f"Filtered search (Module {module_num}): {len(filtered_results)} results, "
            f"confidence={filtered_confidence}"
        )

        # Step 2: High confidence (>= 0.7) - use filtered results, no message
        if filtered_confidence >= 0.7:
            logger.info(f"High confidence with filter - using Module {module_num} results")
            return filtered_results, None

        # Step 3: Get unfiltered results for comparison
        unfiltered_results = await vector_service.search_context(
            query_embedding=query_embedding,
            filters=None,  # Remove all filters
            top_k=top_k,
        )
        unfiltered_confidence = self._calculate_confidence_from_results(unfiltered_results)
        logger.info(
            f"Unfiltered search: {len(unfiltered_results)} results, "
            f"confidence={unfiltered_confidence}"
        )

        # Step 4: Medium confidence (0.4-0.7) - use filtered, suggest other modules
        if filtered_confidence >= 0.4:
            message = (
                f"Found relevant content in Module {module_num}. "
                f"Other modules may have additional information."
            )
            logger.info(f"Medium confidence - suggesting other modules")
            return filtered_results, message

        # Step 5: Low confidence (< 0.4) and unfiltered is significantly better
        if unfiltered_confidence > filtered_confidence + 0.2:
            message = (
                f"I couldn't find relevant information in Module {module_num}, "
                f"so I searched all modules."
            )
            logger.info(f"Low confidence - using unfiltered results")
            return unfiltered_results, message

        # Step 6: Low confidence in both - keep filter but warn
        message = (
            f"Limited results found in Module {module_num}. "
            f"Try removing the filter for broader search."
        )
        logger.info(f"Low confidence in both - keeping filter with warning")
        return filtered_results, message

    def _calculate_confidence_from_results(self, results: List[Any]) -> float:
        """
        Calculate confidence score from search results using weighted average.

        Args:
            results: List of VectorSearchResult objects

        Returns:
            Confidence score (0.0-1.0)
        """
        if not results:
            return 0.0

        # Weight scores by position (earlier results have higher weight)
        total_weight = 0
        weighted_sum = 0
        for idx, result in enumerate(results):
            weight = 1.0 / (idx + 1)  # Position weighting: 1, 0.5, 0.33, 0.25, 0.2
            weighted_sum += result.relevance_score * weight
            total_weight += weight

        confidence = weighted_sum / total_weight if total_weight > 0 else 0.0
        return round(confidence, 2)

    async def _off_topic_response(self, request: QueryRequest) -> QueryResponse:
        """
        Generate response for off-topic queries with suggested terms.

        Enhanced to:
        - Provide helpful topic guidance message
        - Return 3-5 randomly selected suggested terms for variety
        - Guide users back to relevant content

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

        # Build enhanced off-topic message with topic guidance
        answer = (
            "I can only answer questions about the Physical AI & Humanoid Robotics textbook. "
            "The textbook covers topics like ROS 2, Isaac Sim simulation, kinematics, "
            "navigation, path planning, perception, and humanoid robot control.\n\n"
            "Try asking about one of the suggested topics below!"
        )

        # Select 5 random suggested terms for variety (avoids overwhelming users)
        import random

        all_topics = self.faq_data["suggested_topics"]
        suggested_terms = random.sample(all_topics, min(5, len(all_topics)))

        logger.info(f"Selected {len(suggested_terms)} suggested terms: {suggested_terms}")

        # Save user message
        await chat_service.save_message(session_id=session_id, role="user", content=request.query)

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
            suggested_terms=suggested_terms,
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
            answer = "I'm experiencing technical difficulties. Please try again in a few moments."

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
