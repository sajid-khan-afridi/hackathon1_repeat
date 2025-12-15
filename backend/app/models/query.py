"""Query request and response models for the RAG chatbot."""

from datetime import datetime
from typing import List, Optional, Dict, Any

from pydantic import BaseModel, Field, validator


class FilterParams(BaseModel):
    """Filter parameters for query requests."""

    module: Optional[int] = Field(None, ge=1, le=10, description="Module number (1-10)")
    difficulty: Optional[str] = Field(None, description="Difficulty level")
    tags: Optional[List[str]] = Field(None, description="List of tags to filter by")

    @validator("difficulty")
    def validate_difficulty(cls, v: Optional[str]) -> Optional[str]:
        """Validate difficulty level."""
        if v is not None and v not in ["beginner", "intermediate", "advanced"]:
            raise ValueError("Difficulty must be: beginner, intermediate, or advanced")
        return v


class QueryRequest(BaseModel):
    """Request model for querying the RAG system."""

    query: str = Field(..., min_length=1, max_length=1000, description="User query")
    sessionId: Optional[str] = Field(
        None, description="Optional session ID for conversation continuity"
    )
    filters: Optional[FilterParams] = Field(None, description="Optional filters for search scope")

    @validator("query")
    def validate_query(cls, v: str) -> str:
        """Validate and sanitize query."""
        # Basic sanitization - remove excessive whitespace
        v = " ".join(v.split())
        return v


class SourceCitation(BaseModel):
    """Individual source citation for a response."""

    content: str = Field(..., description="Relevant content snippet")
    chapter: int = Field(..., ge=1, description="Chapter number")
    section: str = Field(..., description="Section title")
    module: Optional[int] = Field(None, ge=1, le=10, description="Module number")
    page_url: Optional[str] = Field(None, description="URL to the specific page")
    relevance_score: float = Field(..., ge=0, le=1, description="Relevance score (0-1)")


class TokenUsage(BaseModel):
    """Token usage information for transparency."""

    prompt_tokens: int = Field(..., ge=0, description="Tokens in the prompt")
    completion_tokens: int = Field(..., ge=0, description="Tokens in the completion")
    total_tokens: int = Field(..., ge=0, description="Total tokens used")


class QueryResponse(BaseModel):
    """Response model for successful queries."""

    answer: str = Field(..., description="Generated answer")
    confidence: float = Field(..., ge=0, le=1, description="Confidence score (0-1)")
    sources: List[SourceCitation] = Field(..., description="List of source citations")
    sessionId: str = Field(..., description="Session ID for conversation continuity")
    suggestedQuestions: Optional[List[str]] = Field(
        None, description="Suggested follow-up questions"
    )
    filterMessage: Optional[str] = Field(None, description="Message about applied filters")
    tokenUsage: TokenUsage = Field(..., description="Token usage information")
    responseTime: float = Field(..., description="Response time in seconds")


class ErrorResponse(BaseModel):
    """Error response model."""

    error: str = Field(..., description="Error message")
    code: str = Field(..., description="Error code")
    details: Optional[Dict[str, Any]] = Field(None, description="Additional error details")
    suggestedAction: Optional[str] = Field(None, description="Suggested action for user")


class OffTopicResponse(BaseModel):
    """Response for off-topic queries."""

    message: str = Field(..., description="Message explaining the query is off-topic")
    suggestedTopics: List[str] = Field(..., description="Suggested topics to explore")
    suggestedQueries: List[str] = Field(..., description="Suggested query examples")


class LowConfidenceResponse(BaseModel):
    """Response for low-confidence queries."""

    answer: str = Field(..., description="Generated answer with low confidence")
    confidence: float = Field(..., ge=0, le=0.3, description="Low confidence score")
    warning: str = Field(..., description="Warning about low confidence")
    suggestedRephrase: str = Field(..., description="Suggestion for rephrasing the query")
    sources: List[SourceCitation] = Field(..., description="List of source citations")
    sessionId: str = Field(..., description="Session ID")
    tokenUsage: TokenUsage = Field(..., description="Token usage information")
    responseTime: float = Field(..., description="Response time in seconds")


class RateLimitResponse(BaseModel):
    """Response when rate limit is exceeded."""

    error: str = Field(default="Rate limit exceeded", description="Error message")
    limit: int = Field(..., description="Rate limit for the user")
    resetAfter: int = Field(..., description="Seconds until limit resets")
    retryAfter: str = Field(..., description="Human-readable time until retry")


class HealthCheck(BaseModel):
    """Health check response."""

    status: str = Field(..., description="Service status")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Check timestamp")
    services: Dict[str, str] = Field(..., description="Status of external services")
    version: str = Field(default="0.1.0", description="API version")


class StreamChunk(BaseModel):
    """Streaming response chunk for SSE."""

    chunk: str = Field(..., description="Text chunk")
    done: bool = Field(False, description="Whether streaming is complete")
    sources: Optional[List[SourceCitation]] = Field(
        None, description="Sources (sent with final chunk)"
    )
    confidence: Optional[float] = Field(
        None, description="Confidence score (sent with final chunk)"
    )
    sessionId: str = Field(..., description="Session ID")
    tokenUsage: Optional[TokenUsage] = Field(
        None, description="Token usage (sent with final chunk)"
    )
