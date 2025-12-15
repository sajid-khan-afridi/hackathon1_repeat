"""Application configuration using Pydantic settings."""

import os
from typing import List

from pydantic import Field, validator
from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # =============================================================================
    # OpenAI Configuration
    # =============================================================================
    openai_api_key: str = Field(..., description="OpenAI API key")

    # =============================================================================
    # Qdrant Vector Database
    # =============================================================================
    qdrant_url: str = Field(..., description="Qdrant cluster URL")
    qdrant_api_key: str = Field(..., description="Qdrant API key")
    qdrant_collection: str = Field(default="robotics_textbook", description="Collection name")

    # =============================================================================
    # Neon PostgreSQL Database
    # =============================================================================
    database_url: str = Field(..., description="PostgreSQL connection URL")

    # =============================================================================
    # Backend Configuration
    # =============================================================================
    environment: str = Field(
        default="development", description="Environment: development|production"
    )
    api_host: str = Field(default="0.0.0.0", description="API host")
    api_port: int = Field(default=8000, description="API port")
    cors_origins: str = Field(
        default="http://localhost:3000,http://localhost:8000",
        description="CORS allowed origins (comma-separated)",
    )

    # =============================================================================
    # Rate Limiting
    # =============================================================================
    rate_limit_anonymous: int = Field(
        default=10, description="Queries per hour for anonymous users"
    )
    rate_limit_authenticated: int = Field(
        default=50, description="Queries per hour for authenticated users"
    )

    # =============================================================================
    # Logging Configuration
    # =============================================================================
    log_level: str = Field(default="INFO", description="Log level: DEBUG|INFO|WARNING|ERROR")

    # =============================================================================
    # RAG Configuration
    # =============================================================================
    rag_min_confidence: float = Field(
        default=0.2, description="Minimum confidence score for answering"
    )
    rag_warning_confidence: float = Field(default=0.3, description="Warning confidence threshold")
    rag_max_context_chunks: int = Field(default=5, description="Maximum context chunks to retrieve")
    rag_embedding_model: str = Field(
        default="text-embedding-3-small", description="Embedding model"
    )
    rag_llm_model: str = Field(default="gpt-4o-mini", description="LLM model for generation")

    # =============================================================================
    # Chat Configuration
    # =============================================================================
    chat_history_limit: int = Field(
        default=5, description="Previous exchanges to include in context"
    )
    chat_session_duration_days: int = Field(default=30, description="Session duration in days")

    # =============================================================================
    # Performance Configuration
    # =============================================================================
    request_timeout_seconds: int = Field(default=30, description="Request timeout in seconds")
    db_pool_size: int = Field(default=10, description="Database connection pool size")
    db_max_inactive_connections: int = Field(default=5, description="Max inactive DB connections")

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False

    @validator("rag_min_confidence", "rag_warning_confidence")
    def validate_confidence_scores(cls, v: float) -> float:
        """Validate confidence scores are between 0 and 1."""
        if not 0 <= v <= 1:
            raise ValueError("Confidence scores must be between 0 and 1")
        return v

    @validator("rag_warning_confidence")
    def validate_warning_confidence(cls, v: float, values: dict) -> float:
        """Ensure warning confidence is greater than min confidence."""
        if "rag_min_confidence" in values and v <= values["rag_min_confidence"]:
            raise ValueError("Warning confidence must be greater than minimum confidence")
        return v

    @validator("environment")
    def validate_environment(cls, v: str) -> str:
        """Validate environment value."""
        if v not in ["development", "production", "testing"]:
            raise ValueError("Environment must be: development, production, or testing")
        return v

    @property
    def is_development(self) -> bool:
        """Check if running in development mode."""
        return self.environment == "development"

    @property
    def is_production(self) -> bool:
        """Check if running in production mode."""
        return self.environment == "production"

    @property
    def is_testing(self) -> bool:
        """Check if running in testing mode."""
        return self.environment == "testing"

    @property
    def cors_origins_list(self) -> List[str]:
        """Get CORS origins as a list."""
        return [origin.strip() for origin in self.cors_origins.split(",")]


# Global settings instance
settings = Settings()


def get_settings() -> Settings:
    """Get the application settings instance."""
    return settings
