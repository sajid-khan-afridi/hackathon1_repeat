"""NDCG@10 relevance testing for RAG pipeline.

This module implements Normalized Discounted Cumulative Gain (NDCG) calculation
to measure the quality of search results from the RAG pipeline.

Target: NDCG@10 > 0.8
"""

import json
import math
from pathlib import Path
from typing import Any

import pytest

# Test data path
BENCHMARK_DIR = Path(__file__).parent
TEST_QUESTIONS_PATH = BENCHMARK_DIR / "test_questions.json"


def load_test_questions() -> dict[str, Any]:
    """Load benchmark test questions from JSON file."""
    with open(TEST_QUESTIONS_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def dcg_at_k(relevance_scores: list[float], k: int = 10) -> float:
    """Calculate Discounted Cumulative Gain at position k.

    DCG = sum(rel_i / log2(i + 1)) for i = 1 to k

    Args:
        relevance_scores: List of relevance scores for each retrieved document
        k: Number of results to consider

    Returns:
        DCG score
    """
    relevance_scores = relevance_scores[:k]
    dcg = 0.0
    for i, rel in enumerate(relevance_scores):
        # Position is 1-indexed in the formula
        dcg += rel / math.log2(i + 2)  # i + 2 because i starts at 0
    return dcg


def ndcg_at_k(relevance_scores: list[float], ideal_scores: list[float], k: int = 10) -> float:
    """Calculate Normalized Discounted Cumulative Gain at position k.

    NDCG = DCG / IDCG

    Args:
        relevance_scores: List of relevance scores for retrieved documents
        ideal_scores: List of ideal relevance scores (sorted descending)
        k: Number of results to consider

    Returns:
        NDCG score between 0 and 1
    """
    dcg = dcg_at_k(relevance_scores, k)
    idcg = dcg_at_k(sorted(ideal_scores, reverse=True), k)

    if idcg == 0:
        return 0.0
    return dcg / idcg


def calculate_concept_relevance(
    retrieved_concepts: list[str],
    expected_concepts: list[str]
) -> float:
    """Calculate relevance score based on concept overlap.

    Args:
        retrieved_concepts: Concepts found in retrieved documents
        expected_concepts: Expected concepts from ground truth

    Returns:
        Relevance score between 0 and 1
    """
    if not expected_concepts:
        return 0.0

    retrieved_set = set(c.lower() for c in retrieved_concepts)
    expected_set = set(c.lower() for c in expected_concepts)

    overlap = len(retrieved_set & expected_set)
    return overlap / len(expected_set)


def evaluate_retrieval_quality(
    question: str,
    retrieved_docs: list[dict[str, Any]],
    expected_concepts: list[str],
    k: int = 10
) -> dict[str, float]:
    """Evaluate retrieval quality for a single question.

    Args:
        question: The query question
        retrieved_docs: List of retrieved documents with 'content' and 'score' fields
        expected_concepts: Expected concepts that should appear in results
        k: Number of results to evaluate

    Returns:
        Dictionary with evaluation metrics
    """
    # Calculate relevance scores for each retrieved document
    relevance_scores = []
    for doc in retrieved_docs[:k]:
        content = doc.get("content", "").lower()
        # Check how many expected concepts appear in the document
        matches = sum(1 for concept in expected_concepts if concept.lower() in content)
        rel_score = matches / len(expected_concepts) if expected_concepts else 0.0
        relevance_scores.append(rel_score)

    # Ideal scores would be all 1.0 (perfect relevance)
    ideal_scores = [1.0] * len(expected_concepts)

    # Calculate metrics
    ndcg = ndcg_at_k(relevance_scores, ideal_scores, k)
    precision = sum(1 for s in relevance_scores if s > 0) / len(relevance_scores) if relevance_scores else 0.0
    recall = sum(relevance_scores) / len(expected_concepts) if expected_concepts else 0.0

    return {
        "ndcg_at_k": ndcg,
        "precision_at_k": precision,
        "recall": recall,
        "relevance_scores": relevance_scores
    }


class TestNDCG:
    """Test suite for NDCG@10 relevance evaluation."""

    def test_dcg_calculation(self) -> None:
        """Test basic DCG calculation."""
        # Perfect relevance scores
        scores = [1.0, 1.0, 1.0, 1.0, 1.0]
        dcg = dcg_at_k(scores, k=5)
        assert dcg > 0, "DCG should be positive for relevant results"

        # Descending relevance
        scores = [1.0, 0.8, 0.6, 0.4, 0.2]
        dcg = dcg_at_k(scores, k=5)
        assert dcg > 0, "DCG should be positive"

        # No relevance
        scores = [0.0, 0.0, 0.0, 0.0, 0.0]
        dcg = dcg_at_k(scores, k=5)
        assert dcg == 0.0, "DCG should be zero for irrelevant results"

    def test_ndcg_calculation(self) -> None:
        """Test NDCG normalization."""
        # Perfect ranking
        relevance = [1.0, 1.0, 1.0]
        ideal = [1.0, 1.0, 1.0]
        ndcg = ndcg_at_k(relevance, ideal, k=3)
        assert ndcg == 1.0, "Perfect ranking should have NDCG = 1.0"

        # Imperfect ranking
        relevance = [0.5, 1.0, 0.5]
        ideal = [1.0, 0.5, 0.5]
        ndcg = ndcg_at_k(relevance, ideal, k=3)
        assert 0 < ndcg < 1, "Imperfect ranking should have NDCG between 0 and 1"

        # Empty ideal
        relevance = [1.0, 1.0]
        ideal: list[float] = []
        ndcg = ndcg_at_k(relevance, ideal, k=2)
        assert ndcg == 0.0, "Empty ideal should return 0"

    def test_concept_relevance(self) -> None:
        """Test concept-based relevance calculation."""
        # Full match
        retrieved = ["kinematics", "forward kinematics", "inverse kinematics"]
        expected = ["kinematics", "forward kinematics"]
        score = calculate_concept_relevance(retrieved, expected)
        assert score == 1.0, "Full concept match should be 1.0"

        # Partial match
        retrieved = ["kinematics"]
        expected = ["kinematics", "dynamics"]
        score = calculate_concept_relevance(retrieved, expected)
        assert score == 0.5, "50% concept match should be 0.5"

        # No match
        retrieved = ["unrelated"]
        expected = ["kinematics", "dynamics"]
        score = calculate_concept_relevance(retrieved, expected)
        assert score == 0.0, "No concept match should be 0.0"

    def test_retrieval_evaluation(self) -> None:
        """Test full retrieval evaluation."""
        question = "What is forward kinematics?"
        retrieved_docs = [
            {"content": "Forward kinematics calculates end effector position from joint angles.", "score": 0.9},
            {"content": "Inverse kinematics finds joint angles for a given position.", "score": 0.8},
            {"content": "Kinematics studies motion without considering forces.", "score": 0.7},
        ]
        expected_concepts = ["forward kinematics", "joint angles", "end effector position"]

        results = evaluate_retrieval_quality(question, retrieved_docs, expected_concepts, k=3)

        assert "ndcg_at_k" in results
        assert "precision_at_k" in results
        assert "recall" in results
        assert 0 <= results["ndcg_at_k"] <= 1
        assert 0 <= results["precision_at_k"] <= 1
        assert 0 <= results["recall"] <= 1


class TestBenchmarkQuestions:
    """Test suite for benchmark question validation."""

    @pytest.fixture
    def test_questions(self) -> dict[str, Any]:
        """Load test questions fixture."""
        return load_test_questions()

    def test_questions_file_exists(self) -> None:
        """Verify test questions file exists."""
        assert TEST_QUESTIONS_PATH.exists(), f"Test questions file not found: {TEST_QUESTIONS_PATH}"

    def test_questions_structure(self, test_questions: dict[str, Any]) -> None:
        """Verify test questions have correct structure."""
        assert "metadata" in test_questions
        assert "questions" in test_questions
        assert test_questions["metadata"]["total_questions"] == 50

    def test_all_chapters_covered(self, test_questions: dict[str, Any]) -> None:
        """Verify all 10 chapters are covered."""
        chapters = set(q["chapter"] for q in test_questions["questions"])
        expected_chapters = set(range(1, 11))
        assert chapters == expected_chapters, f"Missing chapters: {expected_chapters - chapters}"

    def test_questions_have_required_fields(self, test_questions: dict[str, Any]) -> None:
        """Verify each question has required fields."""
        required_fields = ["id", "chapter", "topic", "question", "expected_concepts", "difficulty"]
        for q in test_questions["questions"]:
            for field in required_fields:
                assert field in q, f"Question {q.get('id', 'unknown')} missing field: {field}"

    def test_difficulty_distribution(self, test_questions: dict[str, Any]) -> None:
        """Verify reasonable difficulty distribution."""
        difficulties = [q["difficulty"] for q in test_questions["questions"]]
        difficulty_counts = {
            "beginner": difficulties.count("beginner"),
            "intermediate": difficulties.count("intermediate"),
            "advanced": difficulties.count("advanced")
        }
        # Each difficulty level should have at least some questions
        for level, count in difficulty_counts.items():
            assert count > 0, f"No questions at {level} difficulty"

    def test_off_topic_questions_exist(self, test_questions: dict[str, Any]) -> None:
        """Verify off-topic test questions exist."""
        assert "off_topic_questions" in test_questions
        assert len(test_questions["off_topic_questions"]) >= 5


# Integration test placeholder - requires actual RAG service
@pytest.mark.skip(reason="Requires running RAG service")
class TestRAGRelevance:
    """Integration tests for RAG pipeline relevance.

    These tests require a running RAG service and are skipped by default.
    Run with: pytest -m integration
    """

    @pytest.mark.asyncio
    async def test_ndcg_benchmark(self) -> None:
        """Run NDCG@10 benchmark against live RAG service.

        Target: NDCG@10 > 0.8
        """
        # This would be implemented with actual RAG service calls
        # from app.services.rag_service import RAGService
        # rag = RAGService()
        #
        # questions = load_test_questions()
        # ndcg_scores = []
        #
        # for q in questions["questions"]:
        #     result = await rag.query(q["question"])
        #     metrics = evaluate_retrieval_quality(
        #         q["question"],
        #         result["sources"],
        #         q["expected_concepts"]
        #     )
        #     ndcg_scores.append(metrics["ndcg_at_k"])
        #
        # avg_ndcg = sum(ndcg_scores) / len(ndcg_scores)
        # assert avg_ndcg > 0.8, f"NDCG@10 = {avg_ndcg:.3f}, target > 0.8"
        pass
