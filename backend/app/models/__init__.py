"""Pydantic models for the RAG chatbot application."""

from .query import (
    QueryRequest,
    QueryResponse,
    ErrorResponse,
    OffTopicResponse,
    LowConfidenceResponse,
    RateLimitResponse,
    HealthCheck,
    StreamChunk,
    FilterParams,
    SourceCitation,
    TokenUsage,
)
from .chat import (
    ChatSession,
    ChatMessage,
    MessageRole,
    CreateSessionRequest,
    CreateSessionResponse,
    GetSessionResponse,
    DeleteSessionResponse,
    SessionStats,
)
from .citations import (
    SourceCitation as FullSourceCitation,
    CitationType,
    SourceMetadata,
    CitationGroup,
    CitationsResponse,
    CitationStatistics,
)

__all__ = [
    # Query models
    "QueryRequest",
    "QueryResponse",
    "ErrorResponse",
    "OffTopicResponse",
    "LowConfidenceResponse",
    "RateLimitResponse",
    "HealthCheck",
    "StreamChunk",
    "FilterParams",
    "SourceCitation",
    "TokenUsage",
    # Chat models
    "ChatSession",
    "ChatMessage",
    "MessageRole",
    "CreateSessionRequest",
    "CreateSessionResponse",
    "GetSessionResponse",
    "DeleteSessionResponse",
    "SessionStats",
    # Citation models
    "FullSourceCitation",
    "CitationType",
    "SourceMetadata",
    "CitationGroup",
    "CitationsResponse",
    "CitationStatistics",
]
