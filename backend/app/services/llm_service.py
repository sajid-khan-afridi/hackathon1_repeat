"""
LLM service for generating embeddings and responses using OpenAI API.
"""

from typing import List, Dict, Any, Optional, AsyncGenerator
from openai import AsyncOpenAI
from app.config import settings
import logging
import json

logger = logging.getLogger(__name__)


class LLMService:
    """Service for OpenAI Chat Completions and embeddings."""

    # Model constants
    EMBEDDING_MODEL = "text-embedding-3-small"
    CHAT_MODEL = "gpt-4o-mini"
    EMBEDDING_DIMENSIONS = 1536

    # System prompt for RAG responses
    SYSTEM_PROMPT = """You are a helpful AI tutor for a robotics textbook. Your role is to answer student questions based ONLY on the provided context from the textbook.

CRITICAL RULES:
1. ONLY use information from the provided context - never introduce external knowledge
2. If the context doesn't contain enough information, say "I don't have enough information in the textbook to answer this question"
3. Always cite which section/chapter your answer comes from
4. Be concise but thorough - aim for 2-4 sentences
5. Use simple language appropriate for students
6. If asked about topics not in robotics, politely redirect to robotics topics

When answering:
- Start with the most relevant information
- Explain technical terms when first used
- Provide examples from the context when available
- End with a suggestion for related topics if relevant"""

    def __init__(self):
        """Initialize AsyncOpenAI client with API key from settings."""
        self.client = AsyncOpenAI(api_key=settings.openai_api_key)
        logger.info("Initialized LLMService with OpenAI client")

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding vector for input text.

        Args:
            text: Input text to embed (max ~8000 tokens)

        Returns:
            List of floats representing the embedding (1536 dimensions)

        Raises:
            Exception: If OpenAI API call fails
        """
        try:
            # Truncate text if too long (rough estimate: 1 token ≈ 4 chars)
            max_chars = 32000  # ~8000 tokens
            if len(text) > max_chars:
                logger.warning(f"Truncating text for embedding: {len(text)} -> {max_chars} chars")
                text = text[:max_chars]

            response = await self.client.embeddings.create(
                model=self.EMBEDDING_MODEL,
                input=text,
                encoding_format="float",
            )

            embedding = response.data[0].embedding

            logger.info(
                f"Generated embedding (dim={len(embedding)}, "
                f"tokens={response.usage.total_tokens})"
            )
            return embedding

        except Exception as e:
            logger.error(f"Failed to generate embedding: {str(e)}", exc_info=True)
            raise Exception(f"OpenAI embedding generation failed: {str(e)}")

    async def generate_response(
        self,
        prompt: str,
        context: str,
        conversation_history: Optional[List[Dict[str, str]]] = None,
    ) -> Dict[str, Any]:
        """
        Generate response using OpenAI Chat Completions with retrieved context.

        Args:
            prompt: User's question
            context: Retrieved context from vector search (concatenated chunks)
            conversation_history: List of previous messages [{"role": "user|assistant", "content": "..."}]

        Returns:
            Dictionary with:
                - response: Generated answer text
                - input_tokens: Input tokens used
                - output_tokens: Output tokens generated
                - total_tokens: Total tokens used

        Raises:
            Exception: If OpenAI API call fails
        """
        try:
            # Build messages array
            messages = [{"role": "system", "content": self.SYSTEM_PROMPT}]

            # Add conversation history (last N exchanges)
            if conversation_history:
                messages.extend(conversation_history)

            # Add current query with context
            user_message = f"""Context from textbook:
---
{context}
---

Student question: {prompt}

Please answer based on the context above."""

            messages.append({"role": "user", "content": user_message})

            # Call OpenAI Chat Completions
            response = await self.client.chat.completions.create(
                model=self.CHAT_MODEL,
                messages=messages,
                temperature=0.3,  # Low temperature for factual responses
                max_tokens=500,  # Limit response length
                top_p=0.9,
                frequency_penalty=0.0,
                presence_penalty=0.0,
            )

            # Extract response and usage
            answer = response.choices[0].message.content.strip()
            usage = response.usage

            logger.info(
                f"Generated response: {len(answer)} chars, "
                f"tokens=(in={usage.prompt_tokens}, out={usage.completion_tokens})"
            )

            return {
                "response": answer,
                "input_tokens": usage.prompt_tokens,
                "output_tokens": usage.completion_tokens,
                "total_tokens": usage.total_tokens,
            }

        except Exception as e:
            logger.error(f"Failed to generate response: {str(e)}", exc_info=True)
            raise Exception(f"OpenAI response generation failed: {str(e)}")

    async def generate_response_stream(
        self,
        prompt: str,
        context: str,
        conversation_history: Optional[List[Dict[str, str]]] = None,
    ) -> AsyncGenerator[str, None]:
        """
        Generate streaming response using OpenAI Chat Completions with retrieved context.

        Args:
            prompt: User's question
            context: Retrieved context from vector search (concatenated chunks)
            conversation_history: List of previous messages [{"role": "user|assistant", "content": "..."}]

        Yields:
            Text chunks as they arrive from the LLM

        Raises:
            Exception: If OpenAI API call fails
        """
        try:
            # Build messages array (same as non-streaming)
            messages = [{"role": "system", "content": self.SYSTEM_PROMPT}]

            # Add conversation history (last N exchanges)
            if conversation_history:
                messages.extend(conversation_history)

            # Add current query with context
            user_message = f"""Context from textbook:
---
{context}
---

Student question: {prompt}

Please answer based on the context above."""

            messages.append({"role": "user", "content": user_message})

            # Call OpenAI Chat Completions with stream=True
            stream = await self.client.chat.completions.create(
                model=self.CHAT_MODEL,
                messages=messages,
                temperature=0.3,  # Low temperature for factual responses
                max_tokens=500,  # Limit response length
                top_p=0.9,
                frequency_penalty=0.0,
                presence_penalty=0.0,
                stream=True,  # Enable streaming
            )

            logger.info("Started streaming response from OpenAI")

            # Yield chunks as they arrive
            async for chunk in stream:
                if chunk.choices and chunk.choices[0].delta.content:
                    content = chunk.choices[0].delta.content
                    yield content

            logger.info("Completed streaming response from OpenAI")

        except Exception as e:
            logger.error(f"Failed to generate streaming response: {str(e)}", exc_info=True)
            raise Exception(f"OpenAI streaming response generation failed: {str(e)}")

    async def calculate_confidence(
        self, question: str, answer: str, sources: List[Dict[str, Any]]
    ) -> float:
        """
        Calculate confidence score for a generated answer.

        This is a heuristic-based approach combining:
        - Average relevance score of sources
        - Answer length (longer = more detailed = higher confidence)
        - Presence of uncertainty phrases

        Args:
            question: Original user question
            answer: Generated answer
            sources: List of source results with relevance_score

        Returns:
            Confidence score between 0.0 and 1.0
        """
        # Base confidence from average relevance score
        if sources:
            avg_relevance = sum(s.get("relevance_score", 0.0) for s in sources) / len(sources)
        else:
            avg_relevance = 0.0

        # Penalize if answer indicates uncertainty
        uncertainty_phrases = [
            "i don't have",
            "not enough information",
            "cannot answer",
            "unclear",
            "not sure",
        ]
        has_uncertainty = any(phrase in answer.lower() for phrase in uncertainty_phrases)

        if has_uncertainty:
            confidence = avg_relevance * 0.5  # Heavy penalty
        else:
            # Boost confidence if answer is detailed (50+ chars)
            length_bonus = min(len(answer) / 200, 0.2)  # Max +0.2 boost
            confidence = min(avg_relevance + length_bonus, 1.0)

        logger.info(
            f"Calculated confidence: {confidence:.2f} "
            f"(avg_relevance={avg_relevance:.2f}, uncertain={has_uncertainty})"
        )

        return round(confidence, 2)

    async def health_check(self) -> bool:
        """
        Check if OpenAI API is accessible.

        Returns:
            True if healthy, False otherwise
        """
        try:
            # Simple embedding call to verify API access
            await self.client.embeddings.create(
                model=self.EMBEDDING_MODEL,
                input="health check",
            )
            logger.info("LLM service health check passed")
            return True

        except Exception as e:
            logger.error(f"LLM service health check failed: {str(e)}")
            return False


# Global instance
llm_service = LLMService()
