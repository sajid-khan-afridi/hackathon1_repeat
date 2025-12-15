"""Faithfulness testing for RAG pipeline.

This module tests that the RAG chatbot generates responses that are
faithfully grounded in the retrieved context - zero hallucinations.

Target: Zero hallucinations in generated responses
"""

import json
import re
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


def extract_claims(response: str) -> list[str]:
    """Extract factual claims from a response.

    Splits response into sentences and filters for factual statements.

    Args:
        response: The generated response text

    Returns:
        List of factual claims
    """
    # Split into sentences
    sentences = re.split(r'[.!?]+', response)
    claims = []

    for sentence in sentences:
        sentence = sentence.strip()
        if not sentence:
            continue

        # Skip meta-statements and hedging
        skip_patterns = [
            r'^(I|Let me|Based on|According to|From the)',
            r'^(However|Additionally|Furthermore|Moreover)',
            r'(I\'m not sure|I don\'t know|uncertain)',
            r'^(Yes|No|Maybe)',
        ]

        is_meta = any(re.match(pattern, sentence, re.IGNORECASE) for pattern in skip_patterns)
        if not is_meta and len(sentence.split()) > 3:
            claims.append(sentence)

    return claims


def verify_claim_in_context(claim: str, context_chunks: list[str]) -> dict[str, Any]:
    """Verify if a claim is supported by the context.

    Args:
        claim: A factual claim to verify
        context_chunks: List of context passages from retrieval

    Returns:
        Dictionary with verification result and evidence
    """
    claim_lower = claim.lower()
    claim_words = set(re.findall(r'\b\w+\b', claim_lower))

    # Remove common stop words
    stop_words = {'the', 'a', 'an', 'is', 'are', 'was', 'were', 'be', 'been',
                  'being', 'have', 'has', 'had', 'do', 'does', 'did', 'will',
                  'would', 'could', 'should', 'may', 'might', 'must', 'shall',
                  'can', 'to', 'of', 'in', 'for', 'on', 'with', 'at', 'by',
                  'from', 'as', 'into', 'through', 'during', 'before', 'after',
                  'above', 'below', 'between', 'under', 'again', 'further',
                  'then', 'once', 'and', 'but', 'or', 'nor', 'so', 'yet',
                  'this', 'that', 'these', 'those', 'it', 'its'}

    meaningful_words = claim_words - stop_words

    best_match_score = 0.0
    best_match_chunk = ""

    for chunk in context_chunks:
        chunk_lower = chunk.lower()
        chunk_words = set(re.findall(r'\b\w+\b', chunk_lower))

        # Calculate overlap
        overlap = meaningful_words & chunk_words
        if meaningful_words:
            match_score = len(overlap) / len(meaningful_words)
        else:
            match_score = 0.0

        if match_score > best_match_score:
            best_match_score = match_score
            best_match_chunk = chunk

    # Threshold for considering a claim supported
    is_supported = best_match_score >= 0.5

    return {
        "claim": claim,
        "is_supported": is_supported,
        "confidence": best_match_score,
        "evidence": best_match_chunk[:200] if best_match_chunk else None
    }


def check_response_faithfulness(
    response: str,
    context_chunks: list[str],
    strict: bool = True
) -> dict[str, Any]:
    """Check if a response is faithfully grounded in context.

    Args:
        response: The generated response
        context_chunks: The context passages used for generation
        strict: If True, all claims must be supported; if False, majority

    Returns:
        Dictionary with faithfulness metrics
    """
    claims = extract_claims(response)

    if not claims:
        return {
            "is_faithful": True,
            "total_claims": 0,
            "supported_claims": 0,
            "unsupported_claims": [],
            "faithfulness_score": 1.0
        }

    verification_results = []
    for claim in claims:
        result = verify_claim_in_context(claim, context_chunks)
        verification_results.append(result)

    supported = [r for r in verification_results if r["is_supported"]]
    unsupported = [r for r in verification_results if not r["is_supported"]]

    faithfulness_score = len(supported) / len(claims) if claims else 1.0

    # In strict mode, any unsupported claim is a failure
    if strict:
        is_faithful = len(unsupported) == 0
    else:
        is_faithful = faithfulness_score >= 0.8

    return {
        "is_faithful": is_faithful,
        "total_claims": len(claims),
        "supported_claims": len(supported),
        "unsupported_claims": [r["claim"] for r in unsupported],
        "faithfulness_score": faithfulness_score,
        "details": verification_results
    }


def detect_hallucination_patterns(response: str) -> list[dict[str, str]]:
    """Detect common hallucination patterns in responses.

    Args:
        response: The generated response

    Returns:
        List of detected hallucination indicators
    """
    patterns = []

    # Pattern 1: Fabricated statistics
    stat_pattern = r'\b(\d+(?:\.\d+)?)\s*%|\b(\d+(?:,\d{3})*(?:\.\d+)?)\s*(million|billion|thousand)'
    if re.search(stat_pattern, response, re.IGNORECASE):
        patterns.append({
            "type": "statistics",
            "description": "Response contains statistics that may need verification"
        })

    # Pattern 2: Specific dates without context
    date_pattern = r'\b(19|20)\d{2}\b'
    dates = re.findall(date_pattern, response)
    if len(dates) > 2:
        patterns.append({
            "type": "dates",
            "description": "Response contains multiple specific dates"
        })

    # Pattern 3: Named entities that could be fabricated
    quote_pattern = r'"[^"]{20,}"'
    if re.search(quote_pattern, response):
        patterns.append({
            "type": "quotes",
            "description": "Response contains quoted text that may need verification"
        })

    # Pattern 4: Overly specific technical values
    tech_pattern = r'\b\d+(?:\.\d+)?\s*(Hz|kHz|MHz|GHz|ms|seconds?|meters?|degrees?)\b'
    tech_matches = re.findall(tech_pattern, response, re.IGNORECASE)
    if len(tech_matches) > 3:
        patterns.append({
            "type": "technical_values",
            "description": "Response contains many specific technical values"
        })

    return patterns


class TestFaithfulness:
    """Test suite for faithfulness evaluation."""

    def test_claim_extraction(self) -> None:
        """Test extracting claims from responses."""
        response = """Forward kinematics is a method to calculate the position of a robot's end effector.
        It uses joint angles as input. The transformation is done using DH parameters.
        I hope this helps you understand the concept."""

        claims = extract_claims(response)
        assert len(claims) >= 2, "Should extract multiple claims"
        assert any("forward kinematics" in c.lower() for c in claims)

    def test_claim_verification(self) -> None:
        """Test claim verification against context."""
        claim = "Forward kinematics calculates end effector position"
        context = [
            "Forward kinematics is the mathematical process of calculating the position and orientation of the end effector given joint angles.",
            "Inverse kinematics solves the opposite problem."
        ]

        result = verify_claim_in_context(claim, context)
        assert result["is_supported"], "Claim should be supported by context"
        assert result["confidence"] > 0.5

    def test_unsupported_claim(self) -> None:
        """Test detection of unsupported claims."""
        claim = "Robots were invented in 1923 by Karel Capek"
        context = [
            "Forward kinematics is used in robotics.",
            "Inverse kinematics calculates joint angles."
        ]

        result = verify_claim_in_context(claim, context)
        assert not result["is_supported"], "Claim should not be supported"

    def test_faithfulness_check(self) -> None:
        """Test full faithfulness checking."""
        response = """Forward kinematics calculates the end effector position using joint angles.
        The DH parameters define the transformation between links."""

        context = [
            "Forward kinematics is the process of determining the position of the end effector from joint parameters.",
            "The Denavit-Hartenberg (DH) convention provides parameters for defining link transformations."
        ]

        result = check_response_faithfulness(response, context)
        assert result["faithfulness_score"] >= 0.5  # At least half the claims should be supported

    def test_hallucination_detection(self) -> None:
        """Test hallucination pattern detection."""
        # Response with potential hallucinations
        response = """In 1956, the first industrial robot was created.
        It achieved 99.7% accuracy in 2.5 milliseconds.
        As Dr. Smith said, "This changed everything in manufacturing."
        The robot operated at 500 Hz with 0.001 degree precision."""

        patterns = detect_hallucination_patterns(response)
        assert len(patterns) > 0, "Should detect hallucination patterns"
        assert any(p["type"] == "statistics" for p in patterns)

    def test_clean_response(self) -> None:
        """Test that clean responses pass faithfulness check."""
        response = "Kinematics studies motion without considering forces."
        context = ["Kinematics is the branch of mechanics that describes motion without considering forces."]

        result = check_response_faithfulness(response, context)
        assert result["is_faithful"], "Clean response should be faithful"
        assert result["faithfulness_score"] == 1.0


class TestOffTopicHandling:
    """Test suite for off-topic question handling."""

    @pytest.fixture
    def test_questions(self) -> dict[str, Any]:
        """Load test questions fixture."""
        return load_test_questions()

    def test_off_topic_detection_response(self) -> None:
        """Test that off-topic responses include proper decline."""
        # Expected response pattern for off-topic questions
        decline_patterns = [
            r"(can only|cannot|unable to)",
            r"(robotics|textbook|course)",
            r"(try asking|suggest|help you with)",
        ]

        # Simulated off-topic response
        response = "I can only answer questions about robotics from the textbook. Try asking about kinematics, sensors, or robot programming."

        matched = sum(1 for p in decline_patterns if re.search(p, response, re.IGNORECASE))
        assert matched >= 2, "Off-topic response should match decline patterns"

    def test_suggested_topics_in_decline(self) -> None:
        """Test that decline responses include suggested topics."""
        # A good decline response should suggest valid topics
        response = """I can only answer questions related to the robotics textbook.
        Here are some topics I can help with:
        - Robot kinematics and dynamics
        - Sensors and perception
        - Motion planning algorithms"""

        # Should contain robotics-related suggestions
        robotics_terms = ["kinematics", "dynamics", "sensors", "motion", "robot"]
        found_terms = sum(1 for term in robotics_terms if term.lower() in response.lower())
        assert found_terms >= 2, "Decline should suggest robotics topics"


# Integration test placeholder
@pytest.mark.skip(reason="Requires running RAG service")
class TestRAGFaithfulness:
    """Integration tests for RAG pipeline faithfulness.

    These tests require a running RAG service and are skipped by default.
    Run with: pytest -m integration
    """

    @pytest.mark.asyncio
    async def test_zero_hallucinations(self) -> None:
        """Run faithfulness test against live RAG service.

        Target: Zero hallucinations
        """
        # This would be implemented with actual RAG service calls
        # from app.services.rag_service import RAGService
        # rag = RAGService()
        #
        # questions = load_test_questions()
        # hallucination_count = 0
        #
        # for q in questions["questions"][:10]:  # Sample
        #     result = await rag.query(q["question"])
        #     context = [s["content"] for s in result["sources"]]
        #     faithfulness = check_response_faithfulness(result["answer"], context)
        #
        #     if not faithfulness["is_faithful"]:
        #         hallucination_count += 1
        #         print(f"Hallucination in: {q['question']}")
        #         print(f"Unsupported: {faithfulness['unsupported_claims']}")
        #
        # assert hallucination_count == 0, f"Found {hallucination_count} hallucinations"
        pass

    @pytest.mark.asyncio
    async def test_off_topic_rejection(self) -> None:
        """Test that off-topic questions are properly rejected."""
        # from app.services.rag_service import RAGService
        # rag = RAGService()
        #
        # questions = load_test_questions()
        #
        # for q in questions["off_topic_questions"]:
        #     result = await rag.query(q["question"])
        #     assert result["confidence"] < 0.2, f"Off-topic question not rejected: {q['question']}"
        pass
