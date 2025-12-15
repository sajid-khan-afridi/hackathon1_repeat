"""Pytest configuration and shared fixtures."""

import asyncio
import os
import sys
from pathlib import Path
from typing import AsyncGenerator, Generator
from unittest.mock import AsyncMock, MagicMock

import pytest

# Add app directory to Python path
sys.path.insert(0, str(Path(__file__).parent / "app"))

# Conditionally import app modules - may not be available for benchmark tests
try:
    import pytest_asyncio
    from httpx import AsyncClient
    from app.main import app
    from app.config import get_settings
    APP_AVAILABLE = True
except ImportError:
    APP_AVAILABLE = False
    app = None
    get_settings = None


@pytest.fixture(scope="session")
def event_loop() -> Generator:
    """Create an instance of the default event loop for the test session."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


if APP_AVAILABLE:
    @pytest_asyncio.fixture
    async def client() -> AsyncGenerator[AsyncClient, None]:
        """Create a test client for the FastAPI app."""
        async with AsyncClient(app=app, base_url="http://test") as ac:
            yield ac


@pytest.fixture
def mock_settings() -> MagicMock:
    """Mock settings for testing."""
    settings = MagicMock()
    settings.openai_api_key = "test-key"
    settings.qdrant_url = "http://test-qdrant:6333"
    settings.qdrant_api_key = "test-key"
    settings.qdrant_collection = "test-collection"
    settings.database_url = "postgresql://test:test@localhost/test"
    settings.environment = "testing"
    settings.cors_origins = ["http://localhost:3000"]
    settings.rate_limit_anonymous = 10
    settings.rate_limit_authenticated = 50
    settings.log_level = "INFO"
    settings.rag_min_confidence = 0.2
    settings.rag_warning_confidence = 0.3
    settings.rag_max_context_chunks = 5
    settings.rag_embedding_model = "text-embedding-3-small"
    settings.rag_llm_model = "gpt-4o-mini"
    settings.chat_history_limit = 5
    settings.chat_session_duration_days = 30
    settings.request_timeout_seconds = 30
    settings.db_pool_size = 10
    settings.db_max_inactive_connections = 5
    return settings


@pytest.fixture(autouse=True)
def override_settings(mock_settings: MagicMock) -> None:
    """Override settings for all tests."""
    if APP_AVAILABLE and app is not None:
        app.dependency_overrides[get_settings] = lambda: mock_settings


@pytest.fixture
def mock_openai_client() -> AsyncMock:
    """Mock OpenAI client."""
    mock_client = AsyncMock()

    # Mock embeddings response
    mock_client.embeddings.create.return_value = MagicMock(
        data=[MagicMock(embedding=[0.1] * 1536)]
    )

    # Mock chat completions response
    mock_client.chat.completions.create.return_value = MagicMock(
        choices=[
            MagicMock(
                message=MagicMock(
                    content="This is a test answer based on the provided context."
                )
            )
        ],
        usage=MagicMock(
            prompt_tokens=10,
            completion_tokens=20,
            total_tokens=30
        )
    )

    return mock_client


@pytest.fixture
def mock_qdrant_client() -> AsyncMock:
    """Mock Qdrant client."""
    mock_client = AsyncMock()

    # Mock search response
    mock_client.search.return_value = [
        MagicMock(
            id=1,
            score=0.8,
            payload={
                "content": "This is test content about robotics.",
                "chapter": 1,
                "section": "Introduction",
                "module": 1
            }
        ),
        MagicMock(
            id=2,
            score=0.7,
            payload={
                "content": "This is more test content about kinematics.",
                "chapter": 2,
                "section": "Kinematics",
                "module": 2
            }
        )
    ]

    return mock_client


@pytest.fixture
def mock_db_pool() -> AsyncMock:
    """Mock database connection pool."""
    mock_pool = AsyncMock()
    mock_conn = AsyncMock()

    # Mock transaction
    mock_conn.transaction.return_value.__aenter__.return_value = mock_conn
    mock_conn.transaction.return_value.__aexit__.return_value = None

    # Mock fetch operations
    mock_conn.fetch.return_value = []
    mock_conn.fetchrow.return_value = None
    mock_conn.fetchval.return_value = None

    mock_pool.acquire.return_value.__aenter__.return_value = mock_conn
    mock_pool.acquire.return_value.__aexit__.return_value = None

    return mock_pool


@pytest.fixture
def sample_query_request() -> dict:
    """Sample query request for testing."""
    return {
        "query": "What is inverse kinematics?",
        "sessionId": None,
        "filters": None
    }


@pytest.fixture
def sample_context_chunks() -> list:
    """Sample context chunks for testing."""
    return [
        {
            "content": "Inverse kinematics is the process of calculating the joint parameters that provide a desired position of the end-effector.",
            "chapter": 3,
            "section": "Inverse Kinematics",
            "module": 2,
            "score": 0.85
        },
        {
            "content": "The Jacobian matrix relates joint velocities to end-effector velocities in robotic manipulators.",
            "chapter": 3,
            "section": "Jacobian",
            "module": 2,
            "score": 0.75
        }
    ]


@pytest.fixture
def sample_faq_data() -> dict:
    """Sample FAQ fallback data."""
    return {
        "general": [
            {
                "question": "What is robotics?",
                "answer": "Robotics is the interdisciplinary field involving the design, construction, operation, and use of robots."
            }
        ],
        "chapter_1": [
            {
                "question": "What is a robot?",
                "answer": "A robot is a programmable machine capable of carrying out a complex series of actions automatically."
            }
        ]
    }