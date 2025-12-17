"""
Vector search service using Qdrant for semantic similarity search.
"""

from typing import List, Dict, Optional, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models as qdrant_models
from app.config import settings
import logging

logger = logging.getLogger(__name__)


class VectorSearchResult:
    """Container for a single search result with metadata."""

    def __init__(
        self,
        chapter_id: str,
        chapter_title: str,
        excerpt: str,
        relevance_score: float,
        metadata: Optional[Dict[str, Any]] = None,
    ):
        self.chapter_id = chapter_id
        self.chapter_title = chapter_title
        self.excerpt = excerpt
        self.relevance_score = relevance_score
        self.metadata = metadata or {}


class VectorService:
    """Service for vector similarity search operations using Qdrant."""

    def __init__(self):
        """Initialize Qdrant client with configuration from settings."""
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        self.collection_name = settings.qdrant_collection
        logger.info(f"Initialized VectorService with collection: {self.collection_name}")

    async def search_context(
        self,
        query_embedding: List[float],
        filters: Optional[Dict[str, Any]] = None,
        top_k: int = 5,
    ) -> List[VectorSearchResult]:
        """
        Search for semantically similar content in the vector database.

        Args:
            query_embedding: Query vector (1536 dimensions for text-embedding-3-small)
            filters: Optional metadata filters (module, difficulty, tags)
            top_k: Number of results to return (1-10)

        Returns:
            List of VectorSearchResult objects with metadata and scores

        Raises:
            Exception: If Qdrant search fails
        """
        try:
            # Build Qdrant filter conditions
            qdrant_filter = self._build_filter(filters) if filters else None

            # Perform vector search using query_points API
            search_results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                query_filter=qdrant_filter,
                limit=top_k,
                with_payload=True,
                score_threshold=settings.confidence_threshold,
            ).points

            # Convert to VectorSearchResult objects
            results = []
            for idx, result in enumerate(search_results):
                payload = result.payload or {}

                # Extract metadata fields with fallbacks
                chapter_id = payload.get("chapter_id", "unknown")
                chapter_title = payload.get("chapter_title", "Unknown Chapter")
                excerpt = payload.get("text", payload.get("content", ""))

                results.append(
                    VectorSearchResult(
                        chapter_id=chapter_id,
                        chapter_title=chapter_title,
                        excerpt=excerpt[:1000],  # Limit excerpt length
                        relevance_score=result.score,
                        metadata=payload,
                    )
                )

            logger.info(
                f"Vector search returned {len(results)} results "
                f"(top_k={top_k}, filters={filters})"
            )
            return results

        except Exception as e:
            logger.error(f"Vector search failed: {str(e)}", exc_info=True)
            raise Exception(f"Failed to search vector database: {str(e)}")

    def _build_filter(self, filters: Dict[str, Any]) -> qdrant_models.Filter:
        """
        Build Qdrant filter from user-provided filters.

        Args:
            filters: Dictionary with optional keys: module, difficulty, tags

        Returns:
            Qdrant Filter object for search query
        """
        conditions = []

        # Module filter (exact match)
        if "module" in filters and filters["module"] is not None:
            conditions.append(
                qdrant_models.FieldCondition(
                    key="module",
                    match=qdrant_models.MatchValue(value=filters["module"]),
                )
            )

        # Difficulty filter (exact match)
        if "difficulty" in filters and filters["difficulty"]:
            conditions.append(
                qdrant_models.FieldCondition(
                    key="difficulty",
                    match=qdrant_models.MatchValue(value=filters["difficulty"]),
                )
            )

        # Tags filter (any match)
        if "tags" in filters and filters["tags"]:
            for tag in filters["tags"]:
                conditions.append(
                    qdrant_models.FieldCondition(
                        key="tags",
                        match=qdrant_models.MatchValue(value=tag),
                    )
                )

        # Return combined filter (AND conditions)
        if conditions:
            return qdrant_models.Filter(must=conditions)

        return None

    async def health_check(self) -> bool:
        """
        Check if Qdrant service is accessible and collection exists.

        Returns:
            True if healthy, False otherwise
        """
        try:
            collections = self.client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if self.collection_name not in collection_names:
                logger.warning(
                    f"Collection '{self.collection_name}' not found. "
                    f"Available: {collection_names}"
                )
                return False

            logger.info(f"Vector service health check passed")
            return True

        except Exception as e:
            logger.error(f"Vector service health check failed: {str(e)}")
            return False


# Global instance
vector_service = VectorService()
