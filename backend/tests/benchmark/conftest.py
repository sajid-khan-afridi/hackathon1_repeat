"""Pytest configuration for benchmark tests.

This conftest overrides the root conftest to allow running benchmark tests
without requiring full application dependencies.
"""

import pytest


@pytest.fixture
def test_questions_path():
    """Return path to test questions JSON."""
    from pathlib import Path
    return Path(__file__).parent / "test_questions.json"
