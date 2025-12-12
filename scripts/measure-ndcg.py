#!/usr/bin/env python3
"""
Measure NDCG@10 score for search queries against indexed content.
Calculates Normalized Discounted Cumulative Gain to evaluate search quality.
"""

import os
import json
import sys
from pathlib import Path
from typing import List, Dict, Any, Tuple
import math

try:
    from qdrant_client import QdrantClient
    from sentence_transformers import SentenceTransformer
except ImportError as e:
    print(f"Error: Missing required dependency - {e}")
    print("Install with: pip install qdrant-client sentence-transformers")
    sys.exit(1)

# Configuration
COLLECTION_NAME = "robotics_textbook_chapters"
EMBEDDING_MODEL = "sentence-transformers/all-MiniLM-L6-v2"

class NDCGMeasurer:
    def __init__(self, qdrant_url: str = "localhost", qdrant_port: int = 6333):
        """Initialize the NDCG measurer."""
        self.client = QdrantClient(host=qdrant_url, port=qdrant_port)
        self.model = SentenceTransformer(EMBEDDING_MODEL)
        self.collection_name = COLLECTION_NAME

    def search(self, query: str, limit: int = 10) -> List[Dict[str, Any]]:
        """Search for relevant documents."""
        # Generate query embedding
        query_embedding = self.model.encode(query)

        # Search Qdrant
        search_result = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding.tolist(),
            limit=limit,
            with_payload=True
        )

        # Extract results
        results = []
        for hit in search_result:
            results.append({
                "id": hit.id,
                "score": hit.score,
                "chapter_id": hit.payload.get("chapter_id", ""),
                "section_title": hit.payload.get("section_title", ""),
                "content": hit.payload.get("content", "")[:200] + "..." if len(hit.payload.get("content", "")) > 200 else hit.payload.get("content", "")
            })

        return results

    def calculate_dcg(self, relevance_scores: List[int]) -> float:
        """Calculate Discounted Cumulative Gain."""
        dcg = 0.0
        for i, score in enumerate(relevance_scores):
            # Using DCG formula: sum(relevance_i / log2(i + 2))
            dcg += score / math.log2(i + 2)
        return dcg

    def calculate_ndcg(self, query_data: Dict[str, Any], k: int = 10) -> Tuple[float, float, List[Dict]]:
        """
        Calculate NDCG@k for a single query.
        Returns: (ndcg_score, dcg_score, results_with_relevance)
        """
        query = query_data["query"]
        relevance_grades = query_data["relevance_grades"]
        target_chapter = query_data.get("target_chapter", "")

        # Search for documents
        results = self.search(query, limit=k)

        # Map results to relevance scores
        result_scores = []
        annotated_results = []

        for result in results:
            chapter_id = result["chapter_id"]
            relevance = relevance_grades.get(chapter_id, 0)
            result_scores.append(relevance)

            annotated_results.append({
                **result,
                "relevance_grade": relevance,
                "is_target": chapter_id == target_chapter
            })

        # Calculate DCG
        dcg = self.calculate_dcg(result_scores)

        # Calculate IDCG (Ideal DCG) - sort relevance grades descending
        ideal_grades = sorted(relevance_grades.values(), reverse=True)[:k]
        idcg = self.calculate_dcg(ideal_grades)

        # Calculate NDCG
        ndcg = dcg / idcg if idcg > 0 else 0

        return ndcg, dcg, annotated_results

    def measure_all_queries(self, queries_file: Path) -> Dict[str, Any]:
        """Measure NDCG for all queries in the test set."""
        with open(queries_file, 'r', encoding='utf-8') as f:
            data = json.load(f)

        queries = data["queries"]
        results = []

        total_ndcg = 0
        successful_queries = 0
        target_hits = 0

        print(f"🔍 Measuring NDCG@10 for {len(queries)} queries...\n")

        for query_data in queries:
            query_id = query_data["id"]
            query_text = query_data["query"]
            query_type = query_data["type"]

            print(f"Query {query_id}: {query_text}")

            try:
                ndcg, dcg, annotated_results = self.calculate_ndcg(query_data, k=10)
                total_ndcg += ndcg
                successful_queries += 1

                # Check if target chapter is in results
                target_in_results = any(r["is_target"] for r in annotated_results)
                if target_in_results:
                    target_hits += 1

                print(f"  NDCG@10: {ndcg:.3f}")
                print(f"  Target found: {'✅' if target_in_results else '❌'}")

                results.append({
                    "query_id": query_id,
                    "query": query_text,
                    "type": query_type,
                    "ndcg": ndcg,
                    "dcg": dcg,
                    "target_found": target_in_results,
                    "results": annotated_results[:5]  # Top 5 results
                })

            except Exception as e:
                print(f"  ❌ Error: {e}")
                results.append({
                    "query_id": query_id,
                    "query": query_text,
                    "error": str(e)
                })

        # Calculate averages
        average_ndcg = total_ndcg / successful_queries if successful_queries > 0 else 0
        target_hit_rate = target_hits / len(queries) if queries else 0

        # Create summary
        summary = {
            "total_queries": len(queries),
            "successful_queries": successful_queries,
            "average_ndcg": average_ndcg,
            "target_hit_rate": target_hit_rate,
            "passing_threshold": 0.8,
            "passed": average_ndcg >= 0.8
        }

        return {
            "summary": summary,
            "results": results
        }

def main():
    if len(sys.argv) < 2:
        print("Usage: python measure-ndcg.py <queries.json> [qdrant_host] [qdrant_port]")
        sys.exit(1)

    queries_file = Path(sys.argv[1])
    if not queries_file.exists():
        print(f"Error: Query file {queries_file} does not exist")
        sys.exit(1)

    qdrant_host = sys.argv[2] if len(sys.argv) > 2 else "localhost"
    qdrant_port = int(sys.argv[3]) if len(sys.argv) > 3 else 6333

    # Initialize measurer
    measurer = NDCGMeasurer(qdrant_host, qdrant_port)

    # Measure all queries
    measurement_results = measurer.measure_all_queries(queries_file)

    # Save results
    output_file = Path("tests/ndcg-measurement-results.json")
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(measurement_results, f, indent=2)

    # Print summary
    summary = measurement_results["summary"]
    print(f"\n📊 NDCG@10 Measurement Results:")
    print(f"   Total Queries: {summary['total_queries']}")
    print(f"   Successful Queries: {summary['successful_queries']}")
    print(f"   Average NDCG@10: {summary['average_ndcg']:.3f}")
    print(f"   Target Hit Rate: {summary['target_hit_rate']:.1%}")
    print(f"   Threshold: {summary['passing_threshold']}")
    print(f"\n   {'✅ PASSED' if summary['passed'] else '❌ FAILED'}")
    print(f"\n📄 Detailed results saved to: {output_file}")

    # Exit with appropriate code
    sys.exit(0 if summary['passed'] else 1)

if __name__ == "__main__":
    main()