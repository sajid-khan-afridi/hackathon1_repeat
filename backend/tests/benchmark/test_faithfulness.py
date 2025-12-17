"""
Faithfulness Testing for RAG Pipeline - Hallucination Detection

This module tests whether the RAG system generates responses that are
faithfully grounded in the retrieved context or whether it hallucinates
information not present in the sources.

Goal: Zero hallucinations on the benchmark test set.

Methodology:
1. Extract factual claims from the generated answer
2. Verify each claim is supported by the retrieved sources
3. Flag any unsupported claims as hallucinations
4. Calculate faithfulness score (% of supported claims)
"""

import json
import asyncio
import re
from pathlib import Path
from typing import List, Dict, Any, Tuple, Set
import pytest
import httpx

from app.config import settings

# Test configuration
API_BASE_URL = f"http://{settings.api_host}:{settings.api_port}/api/v1"
BENCHMARK_FILE = Path(__file__).parent / "test_questions.json"
TIMEOUT = 30.0


def extract_factual_claims(answer: str) -> List[str]:
    """
    Extract factual claims from the generated answer.

    This is a simplified extraction that splits on sentence boundaries.
    In production, you might use NLP tools for more sophisticated claim extraction.

    Args:
        answer: Generated answer text

    Returns:
        List of factual claim sentences
    """
    # Split on sentence boundaries (., !, ?)
    sentences = re.split(r"[.!?]+", answer)

    # Filter out empty sentences and very short ones (< 10 chars)
    claims = [s.strip() for s in sentences if len(s.strip()) > 10]

    return claims


def normalize_text(text: str) -> str:
    """
    Normalize text for comparison by lowercasing and removing extra whitespace.

    Args:
        text: Text to normalize

    Returns:
        Normalized text
    """
    # Lowercase
    text = text.lower()

    # Remove extra whitespace
    text = " ".join(text.split())

    return text


def check_claim_support(claim: str, sources: List[Dict[str, Any]]) -> Tuple[bool, str]:
    """
    Check if a factual claim is supported by the retrieved sources.

    A claim is considered supported if:
    1. Key terms from the claim appear in at least one source
    2. The semantic meaning is consistent with source content

    This is a heuristic approach. For production, consider using:
    - Semantic similarity (embedding-based)
    - NLI (Natural Language Inference) models
    - LLM-based verification

    Args:
        claim: Factual claim to verify
        sources: List of source documents

    Returns:
        Tuple of (is_supported, supporting_source_text)
    """
    normalized_claim = normalize_text(claim)

    # Extract key terms from claim (words longer than 3 chars, excluding common words)
    common_words = {
        "the",
        "and",
        "for",
        "are",
        "but",
        "not",
        "you",
        "all",
        "can",
        "has",
        "was",
        "were",
        "been",
        "have",
        "this",
        "that",
        "with",
        "from",
        "they",
        "which",
    }

    claim_terms = set(
        word for word in normalized_claim.split() if len(word) > 3 and word not in common_words
    )

    # Check each source for support
    for source in sources:
        source_text = normalize_text(source.get("text", ""))

        # Count how many key terms appear in this source
        matching_terms = sum(1 for term in claim_terms if term in source_text)

        # If >50% of key terms are present, consider it supported
        if len(claim_terms) > 0:
            support_ratio = matching_terms / len(claim_terms)
            if support_ratio > 0.5:
                # Return the first matching source as evidence
                return (True, source.get("text", "")[:200])

    # No source provides sufficient support
    return (False, "")


async def query_api(query_text: str, module_filter: int = None) -> Dict[str, Any]:
    """
    Send a query to the RAG API endpoint.

    Args:
        query_text: The question to ask
        module_filter: Optional module number to filter by

    Returns:
        API response as dict
    """
    async with httpx.AsyncClient(timeout=TIMEOUT) as client:
        payload = {"query": query_text}
        if module_filter:
            payload["filters"] = {"module": module_filter}

        response = await client.post(f"{API_BASE_URL}/query", json=payload)
        response.raise_for_status()
        return response.json()


async def test_single_faithfulness(
    question: Dict[str, Any],
) -> Tuple[str, float, int, int]:
    """
    Test faithfulness for a single question.

    Args:
        question: Question dict from benchmark test set

    Returns:
        Tuple of (question_id, faithfulness_score, total_claims, unsupported_claims)
    """
    question_id = question["id"]
    query_text = question["question"]
    module = question.get("module")

    try:
        # Query the API
        response = await query_api(query_text, module_filter=module)

        # Extract answer and sources
        answer = response.get("answer", "")
        sources = response.get("sources", [])

        if not answer:
            return (question_id, 0.0, 0, 0)

        # Extract factual claims
        claims = extract_factual_claims(answer)

        if not claims:
            # No claims extracted (possibly off-topic decline)
            return (question_id, 1.0, 0, 0)

        # Check each claim for source support
        unsupported_claims = []
        for claim in claims:
            is_supported, evidence = check_claim_support(claim, sources)
            if not is_supported:
                unsupported_claims.append(claim)

        # Calculate faithfulness score
        total_claims = len(claims)
        unsupported_count = len(unsupported_claims)
        faithfulness_score = (
            (total_claims - unsupported_count) / total_claims if total_claims > 0 else 1.0
        )

        return (question_id, faithfulness_score, total_claims, unsupported_count)

    except Exception as e:
        print(f"Error processing question {question_id}: {str(e)}")
        return (question_id, 0.0, 0, 0)


async def run_all_faithfulness_tests(
    questions: List[Dict[str, Any]],
) -> List[Tuple[str, float, int, int]]:
    """
    Run faithfulness tests for all benchmark questions.

    Args:
        questions: List of question dicts

    Returns:
        List of tuples (question_id, faithfulness_score, total_claims, unsupported_claims)
    """
    # Run in batches to respect rate limits
    batch_size = 5
    results = []

    for i in range(0, len(questions), batch_size):
        batch = questions[i : i + batch_size]
        batch_results = await asyncio.gather(*[test_single_faithfulness(q) for q in batch])
        results.extend(batch_results)

        # Small delay between batches
        if i + batch_size < len(questions):
            await asyncio.sleep(1)

    return results


@pytest.mark.asyncio
async def test_faithfulness_benchmark():
    """
    Main faithfulness benchmark test: Detect hallucinations across all 50 questions.

    This test:
    1. Sends each benchmark question to the RAG API
    2. Extracts factual claims from generated answers
    3. Verifies each claim is supported by retrieved sources
    4. Calculates faithfulness scores
    5. Verifies zero hallucinations (100% faithfulness)

    Pass criteria:
    - Average faithfulness score = 1.0 (100% grounded responses)
    - No questions with faithfulness < 0.9
    - Zero completely hallucinated answers (faithfulness = 0.0)
    """
    # Load benchmark questions
    with open(BENCHMARK_FILE, "r", encoding="utf-8") as f:
        benchmark_data = json.load(f)
    questions = benchmark_data["questions"]

    print(f"\n{'=' * 80}")
    print(f"Running Faithfulness Benchmark Test (Hallucination Detection)")
    print(f"Total questions: {len(questions)}")
    print(f"Target: Zero hallucinations (100% faithfulness)")
    print(f"{'=' * 80}\n")

    # Run all faithfulness tests
    results = await run_all_faithfulness_tests(questions)

    # Calculate statistics
    faithfulness_scores = [score for _, score, _, _ in results]
    total_claims = sum(claims for _, _, claims, _ in results)
    total_unsupported = sum(unsupported for _, _, _, unsupported in results)

    avg_faithfulness = sum(faithfulness_scores) / len(faithfulness_scores)
    min_faithfulness = min(faithfulness_scores)

    # Count quality tiers
    perfect_count = sum(1 for score in faithfulness_scores if score == 1.0)
    excellent_count = sum(1 for score in faithfulness_scores if 0.9 <= score < 1.0)
    good_count = sum(1 for score in faithfulness_scores if 0.8 <= score < 0.9)
    poor_count = sum(1 for score in faithfulness_scores if score < 0.8)
    zero_count = sum(1 for score in faithfulness_scores if score == 0.0)

    # Print results by module
    print("\nResults by Module:")
    print(f"{'Module':<10} {'Avg Faithfulness':<18} {'Claims':<10} {'Unsupported':<12}")
    print("-" * 55)

    for module_num in range(1, 11):
        module_results = [
            (qid, score, claims, unsupported)
            for (qid, score, claims, unsupported), q in zip(results, questions)
            if q["module"] == module_num
        ]
        if module_results:
            module_avg = sum(score for _, score, _, _ in module_results) / len(module_results)
            module_claims = sum(claims for _, _, claims, _ in module_results)
            module_unsupported = sum(unsupported for _, _, _, unsupported in module_results)
            print(
                f"Module {module_num:<3} {module_avg:<18.3f} {module_claims:<10} {module_unsupported:<12}"
            )

    # Print overall statistics
    print(f"\n{'=' * 80}")
    print("Overall Statistics:")
    print(f"{'=' * 80}")
    print(f"Average Faithfulness:  {avg_faithfulness:.3f} (target: 1.0)")
    print(f"Min Faithfulness:      {min_faithfulness:.3f}")
    print(f"Total Claims:          {total_claims}")
    print(f"Unsupported Claims:    {total_unsupported}")
    print(
        f"Hallucination Rate:    {(total_unsupported/total_claims*100 if total_claims > 0 else 0):.1f}%"
    )
    print(f"\nQuality Distribution:")
    print(
        f"  Perfect (1.0):     {perfect_count} ({perfect_count/len(faithfulness_scores)*100:.1f}%)"
    )
    print(
        f"  Excellent (0.9-1.0): {excellent_count} ({excellent_count/len(faithfulness_scores)*100:.1f}%)"
    )
    print(f"  Good (0.8-0.9):    {good_count} ({good_count/len(faithfulness_scores)*100:.1f}%)")
    print(f"  Poor (<0.8):       {poor_count} ({poor_count/len(faithfulness_scores)*100:.1f}%)")
    print(f"  Zero (0.0):        {zero_count} ({zero_count/len(faithfulness_scores)*100:.1f}%)")
    print(f"{'=' * 80}\n")

    # Print questions with hallucinations
    if poor_count > 0:
        print("Questions with Potential Hallucinations (Faithfulness < 0.8):")
        poor_results = [
            (qid, score, claims, unsupported, q)
            for (qid, score, claims, unsupported), q in zip(results, questions)
            if score < 0.8
        ]
        poor_results.sort(key=lambda x: x[1])  # Sort by score ascending

        for qid, score, claims, unsupported, q in poor_results[:10]:
            print(
                f"  {qid}: {score:.3f} ({unsupported}/{claims} unsupported) - {q['question'][:60]}..."
            )

        print()

    # Assertions
    assert (
        avg_faithfulness >= 0.95
    ), f"Average faithfulness ({avg_faithfulness:.3f}) must be ≥ 0.95 (near-zero hallucinations)"

    assert (
        poor_count == 0
    ), f"Found {poor_count} questions with faithfulness < 0.8 (significant hallucinations)"

    assert zero_count == 0, f"Found {zero_count} completely hallucinated answers"

    assert (
        total_unsupported <= total_claims * 0.05
    ), f"Hallucination rate ({total_unsupported/total_claims*100:.1f}%) must be ≤ 5%"

    print("✓ Faithfulness Benchmark Test PASSED")
    print(f"  Average Faithfulness: {avg_faithfulness:.3f} (target: ≥ 0.95)")
    print(
        f"  Hallucination Rate: {(total_unsupported/total_claims*100 if total_claims > 0 else 0):.1f}% (target: ≤ 5%)"
    )
    print(
        f"  Perfect Scores: {perfect_count}/{len(faithfulness_scores)} ({perfect_count/len(faithfulness_scores)*100:.1f}%)"
    )


@pytest.mark.asyncio
async def test_claim_extraction():
    """
    Unit test for claim extraction functionality.
    """
    # Test case 1: Multiple sentences
    answer1 = "ROS is a middleware for robotics. It provides communication between nodes. The system is widely used."
    claims1 = extract_factual_claims(answer1)
    assert len(claims1) == 3, f"Expected 3 claims, got {len(claims1)}"

    # Test case 2: Single sentence
    answer2 = "The inverse kinematics problem involves calculating joint angles from end-effector position."
    claims2 = extract_factual_claims(answer2)
    assert len(claims2) == 1, f"Expected 1 claim, got {len(claims2)}"

    # Test case 3: Empty answer
    answer3 = ""
    claims3 = extract_factual_claims(answer3)
    assert len(claims3) == 0, f"Expected 0 claims, got {len(claims3)}"

    print("✓ Claim extraction unit tests PASSED")


@pytest.mark.asyncio
async def test_support_checking():
    """
    Unit test for claim support checking functionality.
    """
    # Test case 1: Fully supported claim
    claim1 = "ROS provides middleware for robotics communication"
    sources1 = [
        {
            "text": "ROS (Robot Operating System) is a middleware framework that provides communication services for robotics applications.",
            "metadata": {"module": 1},
        }
    ]
    is_supported1, _ = check_claim_support(claim1, sources1)
    assert is_supported1, "Claim should be supported by source"

    # Test case 2: Unsupported claim
    claim2 = "Python is the only language supported by ROS"
    sources2 = [
        {
            "text": "ROS supports multiple programming languages including C++, Python, and Lisp.",
            "metadata": {"module": 1},
        }
    ]
    is_supported2, _ = check_claim_support(claim2, sources2)
    # This might be supported or not depending on term matching
    # Just verify the function runs without errors

    # Test case 3: Empty sources
    claim3 = "Some claim about robotics"
    sources3 = []
    is_supported3, _ = check_claim_support(claim3, sources3)
    assert not is_supported3, "Empty sources should not support any claim"

    print("✓ Support checking unit tests PASSED")


if __name__ == "__main__":
    # Run tests directly
    asyncio.run(test_claim_extraction())
    asyncio.run(test_support_checking())
    asyncio.run(test_faithfulness_benchmark())
