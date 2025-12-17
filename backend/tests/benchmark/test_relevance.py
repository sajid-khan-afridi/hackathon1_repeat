"""
NDCG@10 Relevance Testing for RAG Pipeline

This module implements Normalized Discounted Cumulative Gain (NDCG@10)
calculation to measure the quality of retrieved context in the RAG pipeline.

NDCG measures how well the system ranks relevant results, with:
- Perfect ranking = 1.0
- Random ranking ≈ 0.5
- Target threshold: NDCG@10 > 0.8
"""

import json
import math
import asyncio
from pathlib import Path
from typing import List, Dict, Any, Tuple
import pytest
import httpx

from app.config import settings

# Test configuration
API_BASE_URL = f"http://{settings.api_host}:{settings.api_port}/api/v1"
BENCHMARK_FILE = Path(__file__).parent / "test_questions.json"
TIMEOUT = 30.0  # seconds per query


def calculate_dcg(relevance_scores: List[float], k: int = 10) -> float:
    """
    Calculate Discounted Cumulative Gain (DCG) at position k.

    DCG = sum_{i=1}^{k} (relevance_i / log2(i + 1))

    Args:
        relevance_scores: List of relevance scores (0.0 to 1.0) for ranked results
        k: Position cutoff (default 10)

    Returns:
        DCG score (higher is better)
    """
    dcg = 0.0
    for i, score in enumerate(relevance_scores[:k], start=1):
        # Discount by log2(position + 1)
        dcg += score / math.log2(i + 1)
    return dcg


def calculate_ndcg(relevance_scores: List[float], k: int = 10) -> float:
    """
    Calculate Normalized Discounted Cumulative Gain (NDCG) at position k.

    NDCG = DCG / IDCG (ideal DCG with perfect ranking)

    Args:
        relevance_scores: List of relevance scores for ranked results
        k: Position cutoff (default 10)

    Returns:
        NDCG score between 0.0 and 1.0 (1.0 = perfect ranking)
    """
    if not relevance_scores:
        return 0.0

    # Calculate actual DCG
    dcg = calculate_dcg(relevance_scores, k)

    # Calculate ideal DCG (IDCG) with perfect ranking (sorted descending)
    ideal_scores = sorted(relevance_scores, reverse=True)
    idcg = calculate_dcg(ideal_scores, k)

    # Avoid division by zero
    if idcg == 0.0:
        return 0.0

    return dcg / idcg


def assess_relevance(question: Dict[str, Any], sources: List[Dict[str, Any]]) -> List[float]:
    """
    Assess the relevance of retrieved sources for a given question.

    Relevance scoring:
    - 1.0: Perfect match (correct module + contains expected topics)
    - 0.7: Good match (correct module, missing some topics)
    - 0.3: Partial match (wrong module but related content)
    - 0.0: Irrelevant (completely off-topic)

    Args:
        question: Question dict with 'module', 'expected_topics', 'question'
        sources: List of source citations from API response

    Returns:
        List of relevance scores (0.0 to 1.0) for each source
    """
    expected_module = question.get("module")
    expected_topics = set(topic.lower() for topic in question.get("expected_topics", []))

    relevance_scores = []

    for source in sources:
        # Extract module from source metadata
        metadata = source.get("metadata", {})
        source_module = metadata.get("module")
        source_text = source.get("text", "").lower()

        # Count matching topics in source text
        matching_topics = sum(1 for topic in expected_topics if topic in source_text)
        topic_ratio = matching_topics / len(expected_topics) if expected_topics else 0.0

        # Calculate relevance score
        if source_module == expected_module:
            # Correct module
            if topic_ratio >= 0.7:
                # Contains most expected topics
                score = 1.0
            elif topic_ratio >= 0.3:
                # Contains some expected topics
                score = 0.7
            else:
                # Correct module but missing topics
                score = 0.4
        else:
            # Wrong module
            if topic_ratio >= 0.5:
                # Related content despite wrong module
                score = 0.3
            else:
                # Irrelevant
                score = 0.0

        relevance_scores.append(score)

    return relevance_scores


async def query_api(query_text: str, module_filter: int = None) -> Dict[str, Any]:
    """
    Send a query to the RAG API endpoint.

    Args:
        query_text: The question to ask
        module_filter: Optional module number to filter by

    Returns:
        API response as dict

    Raises:
        httpx.HTTPError: If API request fails
    """
    async with httpx.AsyncClient(timeout=TIMEOUT) as client:
        payload = {"query": query_text}
        if module_filter:
            payload["filters"] = {"module": module_filter}

        response = await client.post(f"{API_BASE_URL}/query", json=payload)
        response.raise_for_status()
        return response.json()


async def run_single_benchmark(question: Dict[str, Any]) -> Tuple[str, float, float]:
    """
    Run a single benchmark question and calculate NDCG@10.

    Args:
        question: Question dict from benchmark test set

    Returns:
        Tuple of (question_id, ndcg_score, confidence)
    """
    question_id = question["id"]
    query_text = question["question"]
    module = question.get("module")

    try:
        # Query the API
        response = await query_api(query_text, module_filter=module)

        # Extract sources and confidence
        sources = response.get("sources", [])
        confidence = response.get("confidence", 0.0)

        # Assess relevance of retrieved sources
        relevance_scores = assess_relevance(question, sources)

        # Calculate NDCG@10
        ndcg_score = calculate_ndcg(relevance_scores, k=10)

        return (question_id, ndcg_score, confidence)

    except Exception as e:
        print(f"Error processing question {question_id}: {str(e)}")
        return (question_id, 0.0, 0.0)


async def run_all_benchmarks(questions: List[Dict[str, Any]]) -> List[Tuple[str, float, float]]:
    """
    Run all benchmark questions concurrently with rate limiting.

    Args:
        questions: List of question dicts from benchmark test set

    Returns:
        List of tuples (question_id, ndcg_score, confidence)
    """
    # Run questions in batches to avoid overwhelming the API
    batch_size = 5
    results = []

    for i in range(0, len(questions), batch_size):
        batch = questions[i : i + batch_size]
        batch_results = await asyncio.gather(*[run_single_benchmark(q) for q in batch])
        results.extend(batch_results)

        # Small delay between batches to respect rate limits
        if i + batch_size < len(questions):
            await asyncio.sleep(1)

    return results


@pytest.mark.asyncio
async def test_ndcg_benchmark():
    """
    Main benchmark test: Calculate NDCG@10 across all 50 questions.

    This test:
    1. Loads the 50-question benchmark test set
    2. Sends each question to the RAG API
    3. Assesses relevance of retrieved sources
    4. Calculates NDCG@10 for each question
    5. Verifies average NDCG@10 > 0.8 (target threshold)

    Pass criteria:
    - Average NDCG@10 > 0.8 (high-quality retrieval)
    - At least 80% of questions have NDCG > 0.7
    - No questions with NDCG = 0 (complete failures)
    """
    # Load benchmark questions
    with open(BENCHMARK_FILE, "r", encoding="utf-8") as f:
        benchmark_data = json.load(f)
    questions = benchmark_data["questions"]

    print(f"\n{'=' * 80}")
    print(f"Running NDCG@10 Benchmark Test")
    print(f"Total questions: {len(questions)}")
    print(f"Target threshold: NDCG@10 > 0.8")
    print(f"{'=' * 80}\n")

    # Run all benchmarks
    results = await run_all_benchmarks(questions)

    # Calculate statistics
    ndcg_scores = [score for _, score, _ in results]
    confidences = [conf for _, _, conf in results]

    avg_ndcg = sum(ndcg_scores) / len(ndcg_scores)
    min_ndcg = min(ndcg_scores)
    max_ndcg = max(ndcg_scores)
    avg_confidence = sum(confidences) / len(confidences)

    # Count quality tiers
    excellent_count = sum(1 for score in ndcg_scores if score >= 0.9)
    good_count = sum(1 for score in ndcg_scores if 0.7 <= score < 0.9)
    acceptable_count = sum(1 for score in ndcg_scores if 0.5 <= score < 0.7)
    poor_count = sum(1 for score in ndcg_scores if score < 0.5)
    zero_count = sum(1 for score in ndcg_scores if score == 0.0)

    # Print results by module
    print("\nResults by Module:")
    print(f"{'Module':<10} {'Avg NDCG':<12} {'Count':<8}")
    print("-" * 35)

    for module_num in range(1, 11):
        module_results = [
            (qid, score)
            for (qid, score, _), q in zip(results, questions)
            if q["module"] == module_num
        ]
        if module_results:
            module_avg = sum(score for _, score in module_results) / len(module_results)
            print(f"Module {module_num:<3} {module_avg:<12.3f} {len(module_results):<8}")

    # Print overall statistics
    print(f"\n{'=' * 80}")
    print("Overall Statistics:")
    print(f"{'=' * 80}")
    print(f"Average NDCG@10:     {avg_ndcg:.3f}")
    print(f"Min NDCG@10:         {min_ndcg:.3f}")
    print(f"Max NDCG@10:         {max_ndcg:.3f}")
    print(f"Average Confidence:  {avg_confidence:.3f}")
    print(f"\nQuality Distribution:")
    print(f"  Excellent (≥0.9):  {excellent_count} ({excellent_count/len(ndcg_scores)*100:.1f}%)")
    print(f"  Good (0.7-0.9):    {good_count} ({good_count/len(ndcg_scores)*100:.1f}%)")
    print(
        f"  Acceptable (0.5-0.7): {acceptable_count} ({acceptable_count/len(ndcg_scores)*100:.1f}%)"
    )
    print(f"  Poor (<0.5):       {poor_count} ({poor_count/len(ndcg_scores)*100:.1f}%)")
    print(f"  Zero (0.0):        {zero_count} ({zero_count/len(ndcg_scores)*100:.1f}%)")
    print(f"{'=' * 80}\n")

    # Print worst performing questions
    if poor_count > 0:
        print("Worst Performing Questions (NDCG < 0.5):")
        worst_results = [
            (qid, score, q) for (qid, score, _), q in zip(results, questions) if score < 0.5
        ]
        worst_results.sort(key=lambda x: x[1])  # Sort by score ascending

        for qid, score, q in worst_results[:10]:  # Show top 10 worst
            print(f"  {qid}: {score:.3f} - {q['question'][:70]}...")

        print()

    # Assertions
    assert (
        avg_ndcg > 0.8
    ), f"Average NDCG@10 ({avg_ndcg:.3f}) must be > 0.8 for high-quality retrieval"

    assert (
        good_count + excellent_count >= len(ndcg_scores) * 0.8
    ), f"At least 80% of questions must have NDCG > 0.7 (current: {(good_count + excellent_count)/len(ndcg_scores)*100:.1f}%)"

    assert zero_count == 0, f"Found {zero_count} complete failures (NDCG = 0.0)"

    print("✓ NDCG@10 Benchmark Test PASSED")
    print(f"  Average NDCG@10: {avg_ndcg:.3f} (target: > 0.8)")
    print(
        f"  Quality rate: {(good_count + excellent_count)/len(ndcg_scores)*100:.1f}% (target: ≥ 80%)"
    )


@pytest.mark.asyncio
async def test_ndcg_calculation():
    """
    Unit test for NDCG calculation functions.

    Verifies that DCG and NDCG calculations are correct with known test cases.
    """
    # Test case 1: Perfect ranking (already sorted)
    perfect_scores = [1.0, 0.9, 0.8, 0.7, 0.6]
    ndcg_perfect = calculate_ndcg(perfect_scores, k=5)
    assert ndcg_perfect == 1.0, "Perfect ranking should have NDCG = 1.0"

    # Test case 2: Reversed ranking (worst case)
    reversed_scores = [0.6, 0.7, 0.8, 0.9, 1.0]
    ndcg_reversed = calculate_ndcg(reversed_scores, k=5)
    assert ndcg_reversed < 1.0, "Reversed ranking should have NDCG < 1.0"

    # Test case 3: All zeros
    zero_scores = [0.0, 0.0, 0.0, 0.0, 0.0]
    ndcg_zero = calculate_ndcg(zero_scores, k=5)
    assert ndcg_zero == 0.0, "All zeros should have NDCG = 0.0"

    # Test case 4: Single relevant result at top
    single_top = [1.0, 0.0, 0.0, 0.0, 0.0]
    ndcg_single = calculate_ndcg(single_top, k=5)
    assert ndcg_single == 1.0, "Single relevant result at top should have NDCG = 1.0"

    # Test case 5: Empty list
    empty_scores = []
    ndcg_empty = calculate_ndcg(empty_scores, k=5)
    assert ndcg_empty == 0.0, "Empty list should have NDCG = 0.0"

    print("✓ NDCG calculation unit tests PASSED")


if __name__ == "__main__":
    # Run tests directly
    asyncio.run(test_ndcg_calculation())
    asyncio.run(test_ndcg_benchmark())
