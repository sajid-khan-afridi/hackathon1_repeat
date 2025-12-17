"""
Pydantic models for query requests and responses.
"""

from pydantic import BaseModel, Field, field_validator
from typing import Optional, Dict, Any, List
from uuid import UUID


class QueryFilters(BaseModel):
    """Optional filters for narrowing search scope."""

    module: Optional[int] = Field(None, ge=1, le=10, description="Module number (1-10)")
    difficulty: Optional[str] = Field(
        None,
        pattern="^(beginner|intermediate|advanced)$",
        description="Difficulty level",
    )
    tags: Optional[List[str]] = Field(None, description="Content tags to filter by")


class QueryRequest(BaseModel):
    """Request model for RAG query endpoint."""

    query: str = Field(..., min_length=1, max_length=1000, description="User question")
    user_id: Optional[UUID] = Field(None, description="Authenticated user ID")
    session_id: Optional[UUID] = Field(None, description="Existing session ID")
    filters: Optional[QueryFilters] = Field(None, description="Optional search filters")
    top_k: Optional[int] = Field(5, ge=1, le=10, description="Number of chunks to retrieve")

    @field_validator("query")
    @classmethod
    def validate_query(cls, v: str) -> str:
        """Validate and sanitize query string."""
        # Remove leading/trailing whitespace
        v = v.strip()
        if not v:
            raise ValueError("Query cannot be empty or whitespace only")
        return v


class SourceCitation(BaseModel):
    """Source citation with metadata."""

    chapter_id: str = Field(..., description="Chapter identifier (e.g., 'module-1-chapter-2')")
    chapter_title: str = Field(..., description="Human-readable chapter title")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Relevance score (0-1)")
    excerpt: str = Field(..., description="Relevant text excerpt from source")
    position: int = Field(..., ge=1, description="Position in ranked results")


class TokenUsage(BaseModel):
    """Token usage statistics."""

    input_tokens: int = Field(..., ge=0, description="Input tokens used")
    output_tokens: int = Field(..., ge=0, description="Output tokens used")
    total_tokens: int = Field(..., ge=0, description="Total tokens used")


class QueryResponse(BaseModel):
    """Response model for RAG query endpoint."""

    answer: str = Field(..., description="Generated answer")
    sources: List[SourceCitation] = Field(..., description="Source citations")
    confidence: float = Field(..., ge=0.0, le=1.0, description="Confidence score (0-1)")
    session_id: UUID = Field(..., description="Session identifier for conversation")
    tokens_used: TokenUsage = Field(..., description="Token usage statistics")
    filter_message: Optional[str] = Field(
        None, description="Message if adaptive filtering occurred"
    )
    suggested_terms: Optional[List[str]] = Field(
        None, description="Suggested search terms for off-topic queries"
    )


class StreamChunk(BaseModel):
    """Streaming response chunk."""

    chunk: str = Field(..., description="Text chunk")
    done: bool = Field(False, description="Whether streaming is complete")
    sources: Optional[List[SourceCitation]] = Field(None, description="Final sources if done")
    confidence: Optional[float] = Field(None, description="Final confidence if done")
    session_id: Optional[UUID] = Field(None, description="Session ID if done")
    tokens_used: Optional[TokenUsage] = Field(None, description="Token usage if done")
