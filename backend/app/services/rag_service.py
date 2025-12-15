"""RAG service for orchestrating question answering."""

import json
import logging
import time
from typing import List, Dict, Optional, Tuple, Any
import math

from app.config import get_settings
from app.models.query import (
    QueryRequest,
    QueryResponse,
    ErrorResponse,
    OffTopicResponse,
    LowConfidenceResponse,
    SourceCitation,
    TokenUsage,
)
from app.services.vector_service import vector_service
from app.services.llm_service import llm_service
from app.services.chat_service import chat_service

logger = logging.getLogger(__name__)


class RAGService:
    """Service for Retrieval-Augmented Generation question answering."""

    def __init__(self):
        """Initialize the RAG service."""
        self.settings = get_settings()
        self._faq_data: Optional[Dict] = None

    async def initialize(self) -> None:
        """Initialize the RAG service and load FAQ data."""
        # Initialize dependent services
        await vector_service.initialize()
        await llm_service.initialize()
        await chat_service.initialize()

        # Load FAQ fallback data
        await self._load_faq_data()

        logger.info("RAG service initialized")

    async def process_query(
        self, request: QueryRequest, user_identifier: Optional[str] = None
    ) -> Dict[str, Any]:
        """Process a user query and return a response."""
        start_time = time.time()

        try:
            # Get or create session
            session_id = await self._get_or_create_session(request.sessionId, user_identifier)

            # Create embedding for the query
            query_embedding = await llm_service.create_embedding(request.query)

            # Prepare filters
            filters = self._prepare_filters(request.filters)

            # Search for relevant context
            context_chunks = await vector_service.search_context(
                query_embedding=query_embedding,
                limit=self.settings.rag_max_context_chunks,
                score_threshold=self.settings.rag_min_confidence,
                filters=filters,
            )

            # Get conversation history if we have a session
            conversation_history = None
            if session_id:
                conversation_history = await chat_service.get_conversation_context(
                    session_id, limit=self.settings.chat_history_limit
                )

            # Determine if we have enough relevant context
            if not context_chunks:
                logger.info(f"No relevant context found for query: {request.query}")
                return await self._handle_off_topic_query(request.query, session_id)

            # Calculate weighted confidence score
            confidence = self._calculate_confidence(context_chunks)

            # Generate response
            response = await llm_service.generate_answer(
                query=request.query,
                context_chunks=context_chunks,
                conversation_history=conversation_history,
            )

            # Create citations
            citations = self._create_citations(context_chunks)

            # Save user message to session
            if session_id:
                await chat_service.add_message(
                    session_id=session_id, role="user", content=request.query
                )

                # Save assistant response
                await chat_service.add_message(
                    session_id=session_id,
                    role="assistant",
                    content=response["answer"],
                    token_usage=response["token_usage"],
                )

                # Save citations
                assistant_message = await chat_service.get_messages(session_id, limit=1)
                if assistant_message:
                    await chat_service.add_citations(assistant_message[-1].id, citations)

            # Calculate response time
            response_time = time.time() - start_time

            # Check confidence level and format response accordingly
            if confidence < self.settings.rag_min_confidence:
                # Off-topic - no answer
                return await self._handle_off_topic_query(request.query, session_id)
            elif confidence < self.settings.rag_warning_confidence:
                # Low confidence - answer with warning
                return self._format_low_confidence_response(
                    answer=response["answer"],
                    confidence=confidence,
                    citations=citations,
                    session_id=session_id,
                    token_usage=response["token_usage"],
                    response_time=response_time,
                    query=request.query,
                )
            else:
                # High confidence - normal response
                return self._format_normal_response(
                    answer=response["answer"],
                    confidence=confidence,
                    citations=citations,
                    session_id=session_id,
                    token_usage=response["token_usage"],
                    response_time=response_time,
                    filter_message=self._get_filter_message(filters, confidence),
                )

        except Exception as e:
            logger.error(f"Error processing query: {e}", exc_info=True)
            return ErrorResponse(
                error="Failed to process query",
                code="PROCESSING_ERROR",
                details={"error": str(e)} if self.settings.is_development else None,
            ).dict()

    async def _get_or_create_session(
        self, session_id: Optional[str], user_identifier: Optional[str]
    ) -> Optional[str]:
        """Get existing session or create new one."""
        if session_id:
            # Check if session exists
            session = await chat_service.get_session(session_id=session_id)
            if session:
                await chat_service.update_session_activity(session_id)
                return session_id

        # Create new session
        if user_identifier or session_id:
            new_session = await chat_service.create_session(user_id=user_identifier)
            return new_session.id

        return None

    def _prepare_filters(self, filters) -> Optional[Dict[str, Any]]:
        """Prepare filters for vector search."""
        if not filters:
            return None

        filter_dict = {}
        if filters.module:
            filter_dict["module"] = filters.module
        if filters.difficulty:
            filter_dict["difficulty"] = filters.difficulty
        if filters.tags:
            filter_dict["tags"] = filters.tags

        return filter_dict if filter_dict else None

    def _calculate_confidence(self, context_chunks: List[Dict]) -> float:
        """Calculate weighted confidence score from retrieved chunks."""
        if not context_chunks:
            return 0.0

        # Weight by position (first result is most important)
        weights = [1.0 / (i + 1) for i in range(len(context_chunks))]
        total_weight = sum(weights)

        # Calculate weighted average
        weighted_sum = sum(
            chunk["score"] * weight for chunk, weight in zip(context_chunks, weights)
        )
        confidence = weighted_sum / total_weight

        # Apply confidence scaling based on number of results
        if len(context_chunks) == 1:
            confidence *= 0.9  # Slightly penalize single result
        elif len(context_chunks) >= 3:
            confidence *= 1.1  # Boost for multiple corroborating sources

        # Clamp to [0, 1]
        return max(0.0, min(1.0, confidence))

    def _create_citations(self, context_chunks: List[Dict]) -> List[SourceCitation]:
        """Create citation objects from context chunks."""
        citations = []
        for chunk in context_chunks:
            citation = SourceCitation(
                content=chunk["content"],
                chapter=chunk["chapter"],
                section=chunk["section"],
                module=chunk.get("module"),
                relevance_score=chunk["score"],
            )
            citations.append(citation)
        return citations

    async def _handle_off_topic_query(self, query: str, session_id: Optional[str]) -> Dict:
        """Handle queries with no relevant context."""
        # Try to get suggestions from FAQ
        suggestions = await self._get_suggested_queries(query)

        return OffTopicResponse(
            message="I can only answer questions about robotics textbook content. "
            "Your question appears to be outside the scope of the material.",
            suggestedTopics=[
                "Robot Fundamentals",
                "Kinematics and Dynamics",
                "Trajectory Planning",
                "Control Systems",
                "Sensors and Perception",
                "Robot Programming",
                "Machine Learning in Robotics",
            ],
            suggestedQueries=suggestions[:5],  # Limit to 5 suggestions
        ).dict()

    async def _get_suggested_queries(self, query: str) -> List[str]:
        """Get suggested queries based on FAQ data."""
        if not self._faq_data:
            return []

        # Simple keyword matching for suggestions
        query_words = set(query.lower().split())
        suggestions = []

        # Search through FAQ for similar questions
        for category, faqs in self._faq_data.items():
            for faq in faqs:
                faq_words = set(faq["question"].lower().split())
                # Calculate simple overlap
                overlap = len(query_words & faq_words)
                if overlap > 0:
                    suggestions.append(faq["question"])

        # Return top suggestions
        return suggestions[:10]

    def _format_low_confidence_response(
        self,
        answer: str,
        confidence: float,
        citations: List[SourceCitation],
        session_id: str,
        token_usage: Dict,
        response_time: float,
        query: str,
    ) -> Dict:
        """Format a low confidence response with warning."""
        return LowConfidenceResponse(
            answer=answer,
            confidence=confidence,
            warning=f"I'm not very confident about this answer (confidence: {confidence:.0%}). "
            f"Please verify the information or try rephrasing your question.",
            suggestedRephrase="Try to include more specific terms like 'kinematics', "
            "'sensors', or 'control systems' in your question.",
            sources=citations,
            sessionId=session_id,
            tokenUsage=TokenUsage(**token_usage),
            responseTime=response_time,
        ).dict()

    def _format_normal_response(
        self,
        answer: str,
        confidence: float,
        citations: List[SourceCitation],
        session_id: str,
        token_usage: Dict,
        response_time: float,
        filter_message: Optional[str] = None,
    ) -> Dict:
        """Format a normal response."""
        return QueryResponse(
            answer=answer,
            confidence=confidence,
            sources=citations,
            sessionId=session_id,
            filterMessage=filter_message,
            tokenUsage=TokenUsage(**token_usage),
            responseTime=response_time,
        ).dict()

    def _get_filter_message(self, filters: Optional[Dict], confidence: float) -> Optional[str]:
        """Generate message about applied filters."""
        if not filters:
            return None

        messages = []
        if "module" in filters:
            messages.append(f"Module {filters['module']}")
        if "difficulty" in filters:
            messages.append(f"{filters['difficulty'].capitalize()} difficulty")

        if messages:
            filter_msg = f"Showing results for: {', '.join(messages)}"
            if confidence < 0.5:
                filter_msg += ". Try removing filters for more results."
            return filter_msg

        return None

    async def _load_faq_data(self) -> None:
        """Load FAQ fallback data from file."""
        try:
            with open("static/data/faq-fallback.json", "r") as f:
                self._faq_data = json.load(f)
            logger.info("Loaded FAQ fallback data")
        except Exception as e:
            logger.warning(f"Could not load FAQ data: {e}")
            self._faq_data = None


# Global RAG service instance
rag_service = RAGService()
