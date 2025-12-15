"""Source citation models for the RAG system."""

from datetime import datetime
from typing import List, Optional
from enum import Enum

from pydantic import BaseModel, Field, HttpUrl


class CitationType(str, Enum):
    """Type of citation source."""

    CHAPTER = "chapter"
    SECTION = "section"
    MODULE = "module"
    PAGE = "page"


class SourceMetadata(BaseModel):
    """Metadata for a source citation."""

    chapter: int = Field(..., ge=1, description="Chapter number")
    chapterTitle: str = Field(..., description="Chapter title")
    section: Optional[str] = Field(None, description="Section title")
    module: Optional[int] = Field(None, ge=1, le=10, description="Module number")
    moduleTitle: Optional[str] = Field(None, description="Module title")
    pageNumber: Optional[int] = Field(None, description="Page number in textbook")
    tags: List[str] = Field(default_factory=list, description="Associated tags")
    difficulty: Optional[str] = Field(None, description="Difficulty level")


class SourceCitation(BaseModel):
    """Individual source citation with full details."""

    id: str = Field(..., description="Unique citation ID")
    content: str = Field(..., description="Content snippet from source")
    metadata: SourceMetadata = Field(..., description="Source metadata")
    relevanceScore: float = Field(..., ge=0, le=1, description="Relevance score (0-1)")
    similarityScore: Optional[float] = Field(
        None, ge=0, le=1, description="Similarity score from vector search"
    )
    position: Optional[int] = Field(None, description="Position in retrieved results")
    pageUrl: Optional[HttpUrl] = Field(None, description="URL to the specific page")
    createdAt: datetime = Field(
        default_factory=datetime.utcnow, description="Citation creation time"
    )

    class Config:
        json_encoders = {datetime: lambda v: v.isoformat()}


class CitationGroup(BaseModel):
    """Group of citations for the same source."""

    sourceId: str = Field(..., description="Source identifier")
    sourceType: CitationType = Field(..., description="Type of source")
    title: str = Field(..., description="Source title")
    citations: List[SourceCitation] = Field(..., description="List of citations")
    combinedRelevance: float = Field(..., ge=0, le=1, description="Combined relevance score")


class CitationsResponse(BaseModel):
    """Response containing multiple citations."""

    query: str = Field(..., description="Original query")
    citations: List[SourceCitation] = Field(..., description="List of citations")
    totalCitations: int = Field(..., description="Total number of citations")
    averageRelevance: float = Field(..., ge=0, le=1, description="Average relevance score")
    processingTime: float = Field(..., description="Time to process citations (seconds)")


class CitationStatistics(BaseModel):
    """Statistics about citations for a query."""

    totalSources: int = Field(..., description="Total number of sources cited")
    uniqueChapters: int = Field(..., description="Number of unique chapters cited")
    uniqueModules: int = Field(..., description="Number of unique modules cited")
    averageRelevanceScore: float = Field(..., ge=0, le=1, description="Average relevance score")
    highestRelevanceSource: Optional[SourceCitation] = Field(
        None, description="Source with highest relevance"
    )
    citationDistribution: dict = Field(..., description="Distribution of citations by source type")
