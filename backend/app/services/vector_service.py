"""Vector database service for semantic search."""

import logging
from typing import List, Dict, Optional, Any
from contextlib import asynccontextmanager

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import (
    Distance,
    VectorParams,
    SearchParams,
    Filter,
    FieldCondition,
    MatchValue,
    Range,
)

from app.config import get_settings

logger = logging.getLogger(__name__)


class VectorService:
    """Service for managing Qdrant vector database operations."""

    def __init__(self):
        """Initialize the vector service."""
        self.settings = get_settings()
        self._client: Optional[QdrantClient] = None

    async def initialize(self) -> None:
        """Initialize the Qdrant client."""
        if self._client is None:
            self._client = QdrantClient(
                url=self.settings.qdrant_url,
                api_key=self.settings.qdrant_api_key,
                timeout=self.settings.request_timeout_seconds,
            )
            logger.info("Connected to Qdrant vector database")

    @asynccontextmanager
    async def get_client(self):
        """Get Qdrant client with lazy initialization."""
        if self._client is None:
            await self.initialize()
        yield self._client

    async def search_context(
        self,
        query_embedding: List[float],
        limit: int = 5,
        score_threshold: float = 0.2,
        filters: Optional[Dict[str, Any]] = None,
    ) -> List[Dict[str, Any]]:
        """Search for relevant context chunks."""
        async with self.get_client() as client:
            # Build filter if provided
            search_filter = self._build_filter(filters) if filters else None

            # Perform search
            search_result = client.query_points(
                collection_name=self.settings.qdrant_collection,
                query=query_embedding,
                query_filter=search_filter,
                limit=limit,
                score_threshold=score_threshold,
                search_params=SearchParams(hnsw_ef=128, exact=False),
            ).points

            # Convert results to standardized format
            results = []
            for hit in search_result:
                if hit.score >= score_threshold:
                    results.append(
                        {
                            "id": str(hit.id),
                            "score": hit.score,
                            "content": hit.payload.get("content", ""),
                            "chapter": hit.payload.get("chapter", 0),
                            "section": hit.payload.get("section", ""),
                            "module": hit.payload.get("module", None),
                            "page_number": hit.payload.get("page_number", None),
                            "tags": hit.payload.get("tags", []),
                            "difficulty": hit.payload.get("difficulty", None),
                            "metadata": {
                                k: v
                                for k, v in hit.payload.items()
                                if k
                                not in [
                                    "content",
                                    "chapter",
                                    "section",
                                    "module",
                                    "page_number",
                                    "tags",
                                    "difficulty",
                                ]
                            },
                        }
                    )

            logger.info(f"Found {len(results)} relevant chunks above threshold {score_threshold}")
            return results

    def _build_filter(self, filters: Dict[str, Any]) -> Filter:
        """Build Qdrant filter from filter parameters."""
        must_conditions = []

        # Module filter
        if "module" in filters:
            must_conditions.append(
                FieldCondition(key="module", match=MatchValue(value=filters["module"]))
            )

        # Difficulty filter
        if "difficulty" in filters:
            must_conditions.append(
                FieldCondition(key="difficulty", match=MatchValue(value=filters["difficulty"]))
            )

        # Tags filter (match any tag in the list)
        if "tags" in filters and filters["tags"]:
            for tag in filters["tags"]:
                must_conditions.append(FieldCondition(key="tags", match=MatchValue(value=tag)))

        # Chapter range filter
        if "chapter_min" in filters or "chapter_max" in filters:
            chapter_range = {}
            if "chapter_min" in filters:
                chapter_range["gte"] = filters["chapter_min"]
            if "chapter_max" in filters:
                chapter_range["lte"] = filters["chapter_max"]
            must_conditions.append(FieldCondition(key="chapter", range=Range(**chapter_range)))

        return Filter(must=must_conditions) if must_conditions else None

    async def get_collection_info(self) -> Dict[str, Any]:
        """Get information about the collection."""
        async with self.get_client() as client:
            try:
                collection_info = client.get_collection(self.settings.qdrant_collection)
                return {
                    "name": self.settings.qdrant_collection,
                    "vector_count": collection_info.points_count,
                    "vector_size": collection_info.config.params.vectors.size,
                    "distance": collection_info.config.params.vectors.distance,
                    "status": "exists",
                }
            except Exception:
                return {"name": self.settings.qdrant_collection, "status": "not_found"}

    async def create_collection(self) -> None:
        """Create the collection if it doesn't exist."""
        async with self.get_client() as client:
            try:
                client.get_collection(self.settings.qdrant_collection)
                logger.info(f"Collection {self.settings.qdrant_collection} already exists")
            except Exception:
                # Create collection with text-embedding-3-small dimensions (1536)
                client.create_collection(
                    collection_name=self.settings.qdrant_collection,
                    vectors_config=VectorParams(size=1536, distance=Distance.COSINE),
                )
                logger.info(f"Created collection {self.settings.qdrant_collection}")

    async def check_health(self) -> str:
        """Check the health of the vector service."""
        try:
            async with self.get_client() as client:
                # Simple health check - get collections
                client.get_collections()
                return "healthy"
        except Exception as e:
            logger.error(f"Vector service health check failed: {e}")
            return "unhealthy"

    async def close(self) -> None:
        """Close the vector service connection."""
        if self._client:
            self._client.close()
            self._client = None
            logger.info("Closed vector database connection")


# Global vector service instance
vector_service = VectorService()
