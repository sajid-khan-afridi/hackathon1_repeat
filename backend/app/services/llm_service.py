"""LLM service for OpenAI API interactions."""

import logging
from typing import Any, List, Dict, Optional, AsyncGenerator

from openai import AsyncOpenAI, OpenAIError
from openai.types.chat import ChatCompletion, ChatCompletionChunk

from app.config import get_settings

logger = logging.getLogger(__name__)


class LLMService:
    """Service for interacting with OpenAI's language models."""

    def __init__(self):
        """Initialize the LLM service."""
        self.settings = get_settings()
        self._client: Optional[AsyncOpenAI] = None

    async def initialize(self) -> None:
        """Initialize the OpenAI client."""
        if self._client is None:
            self._client = AsyncOpenAI(
                api_key=self.settings.openai_api_key, timeout=self.settings.request_timeout_seconds
            )
            logger.info("Initialized OpenAI client")

    @property
    def client(self) -> AsyncOpenAI:
        """Get or create the OpenAI client."""
        if self._client is None:
            raise RuntimeError("LLMService not initialized. Call initialize() first.")
        return self._client

    async def generate_answer(
        self,
        query: str,
        context_chunks: List[Dict[str, Any]],
        conversation_history: Optional[List[Dict[str, str]]] = None,
    ) -> Dict[str, Any]:
        """Generate an answer based on query and context."""
        # Prepare the prompt
        prompt = self._build_prompt(query, context_chunks, conversation_history)

        try:
            # Generate completion
            response: ChatCompletion = await self.client.chat.completions.create(
                model=self.settings.rag_llm_model,
                messages=[
                    {"role": "system", "content": self._get_system_prompt()},
                    {"role": "user", "content": prompt},
                ],
                temperature=0.3,  # Lower temperature for more factual responses
                max_tokens=1000,
                stream=False,
            )

            # Extract response
            answer = response.choices[0].message.content
            token_usage = {
                "prompt_tokens": response.usage.prompt_tokens,
                "completion_tokens": response.usage.completion_tokens,
                "total_tokens": response.usage.total_tokens,
            }

            logger.info(f"Generated answer with {token_usage['total_tokens']} tokens")
            return {"answer": answer, "token_usage": token_usage}

        except OpenAIError as e:
            logger.error(f"OpenAI API error: {e}")
            raise

    async def generate_answer_stream(
        self,
        query: str,
        context_chunks: List[Dict[str, Any]],
        conversation_history: Optional[List[Dict[str, str]]] = None,
    ) -> AsyncGenerator[str, None]:
        """Generate a streaming answer based on query and context."""
        # Prepare the prompt
        prompt = self._build_prompt(query, context_chunks, conversation_history)

        try:
            # Generate streaming completion
            stream = await self.client.chat.completions.create(
                model=self.settings.rag_llm_model,
                messages=[
                    {"role": "system", "content": self._get_system_prompt()},
                    {"role": "user", "content": prompt},
                ],
                temperature=0.3,
                max_tokens=1000,
                stream=True,
            )

            # Yield chunks as they arrive
            async for chunk in stream:
                if chunk.choices[0].delta.content:
                    yield chunk.choices[0].delta.content

        except OpenAIError as e:
            logger.error(f"OpenAI streaming error: {e}")
            raise

    async def create_embedding(self, text: str) -> List[float]:
        """Create embedding for the given text."""
        try:
            response = await self.client.embeddings.create(
                model=self.settings.rag_embedding_model, input=text
            )
            return response.data[0].embedding

        except OpenAIError as e:
            logger.error(f"Embedding creation error: {e}")
            raise

    def _get_system_prompt(self) -> str:
        """Get the system prompt for the LLM."""
        return """You are a helpful AI assistant answering questions about robotics textbook content.

Your task is to answer the user's question based ONLY on the provided context chunks. Follow these rules:

1. Base your answer entirely on the provided context. Do not use outside knowledge.
2. If multiple chunks are provided, synthesize information from all relevant chunks.
3. Always cite your sources by mentioning which chapter/section the information comes from.
4. If the context doesn't contain enough information to fully answer the question, state this clearly.
5. If the context completely contradicts the question or contains no relevant information, say "I cannot answer this question based on the provided textbook content."
6. Be precise and factual. Use the exact terminology from the textbook when possible.
7. Format your answer in a clear, educational manner suitable for students."""

    def _build_prompt(
        self,
        query: str,
        context_chunks: List[Dict[str, Any]],
        conversation_history: Optional[List[Dict[str, str]]] = None,
    ) -> str:
        """Build the prompt for the LLM."""
        # Add conversation history if provided
        history_text = ""
        if conversation_history:
            history_text = "\n\nPrevious conversation:\n"
            for turn in conversation_history[-10:]:  # Last 10 messages (5 exchanges)
                role = turn.get("role", "")
                content = turn.get("content", "")
                if role == "user":
                    history_text += f"Q: {content}\n"
                elif role == "assistant":
                    history_text += f"A: {content}\n"

        # Format context chunks
        context_text = "\n\n".join(
            [
                f"Context {i+1} (Chapter {chunk['chapter']}, {chunk['section']}):\n{chunk['content']}"
                for i, chunk in enumerate(context_chunks)
            ]
        )

        # Build the complete prompt
        prompt = f"""Question: {query}

{context_text}

{history_text}

Based on the above context, please answer the question. Remember to cite your sources."""

        return prompt

    async def check_health(self) -> str:
        """Check the health of the LLM service."""
        try:
            # Simple test - get available models
            await self.client.models.list()
            return "healthy"
        except Exception as e:
            logger.error(f"LLM service health check failed: {e}")
            return "unhealthy"

    async def close(self) -> None:
        """Close the LLM service connection."""
        if self._client:
            await self._client.close()
            self._client = None
            logger.info("Closed LLM service connection")


# Global LLM service instance
llm_service = LLMService()
