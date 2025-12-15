"""
Pytest configuration and shared fixtures for backend tests.
"""
import pytest
import asyncio
from typing import Generator


@pytest.fixture(scope="session")
def event_loop() -> Generator:
    """Create an instance of the default event loop for the test session."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


@pytest.fixture
def mock_env_vars(monkeypatch):
    """Mock environment variables for testing."""
    monkeypatch.setenv("OPENAI_API_KEY", "test-key")
    monkeypatch.setenv("QDRANT_URL", "http://localhost:6333")
    monkeypatch.setenv("QDRANT_API_KEY", "test-api-key")
    monkeypatch.setenv("QDRANT_COLLECTION", "test_collection")
    monkeypatch.setenv("DATABASE_URL", "postgres://test:test@localhost:5432/test")
    monkeypatch.setenv("ENVIRONMENT", "development")
    monkeypatch.setenv("API_HOST", "0.0.0.0")
    monkeypatch.setenv("API_PORT", "8000")
    monkeypatch.setenv("CORS_ORIGINS", "http://localhost:3000")
    monkeypatch.setenv("RATE_LIMIT_ANONYMOUS", "10")
    monkeypatch.setenv("RATE_LIMIT_AUTHENTICATED", "50")
    monkeypatch.setenv("CONFIDENCE_THRESHOLD", "0.2")
    monkeypatch.setenv("TOP_K_CHUNKS", "5")
    monkeypatch.setenv("MAX_CONVERSATION_HISTORY", "5")
    monkeypatch.setenv("SESSION_RETENTION_DAYS", "30")
