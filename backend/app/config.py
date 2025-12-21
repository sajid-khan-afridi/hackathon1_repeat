"""
Application configuration using pydantic-settings.
"""

from pathlib import Path
from pydantic_settings import BaseSettings, SettingsConfigDict
from typing import List, Optional

# Get the .env file from the project root (parent of backend directory)
ENV_FILE = Path(__file__).parent.parent.parent / ".env"


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    model_config = SettingsConfigDict(
        env_file=str(ENV_FILE),
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore",
    )

    # OpenAI Configuration
    openai_api_key: str

    # Qdrant Configuration
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection: str = "robotics_textbook"

    # Database Configuration
    database_url: str

    # API Configuration
    environment: str = "development"
    api_host: str = "0.0.0.0"
    api_port: int = 8000
    cors_origins: str = "http://localhost:3000,http://localhost:8000"

    # Rate Limiting
    rate_limit_anonymous: int = 10
    rate_limit_authenticated: int = 50

    # RAG Configuration
    confidence_threshold: float = 0.2
    top_k_chunks: int = 5
    max_conversation_history: int = 5
    session_retention_days: int = 30

    # JWT Configuration (Phase 4A: Authentication)
    jwt_private_key: Optional[str] = None
    jwt_public_key: Optional[str] = None
    jwt_algorithm: str = "RS256"
    jwt_access_token_expire_minutes: int = 1440  # 24 hours
    jwt_refresh_token_expire_days: int = 30

    # Google OAuth Configuration
    google_client_id: Optional[str] = None
    google_client_secret: Optional[str] = None
    google_redirect_uri: Optional[str] = None

    # GitHub OAuth Configuration
    github_client_id: Optional[str] = None
    github_client_secret: Optional[str] = None
    github_redirect_uri: Optional[str] = None

    # Security Configuration
    csrf_secret_key: Optional[str] = None

    # Frontend URL for OAuth redirects
    frontend_url: str = "http://localhost:3000"

    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins from comma-separated string."""
        return [origin.strip() for origin in self.cors_origins.split(",")]

    @property
    def is_development(self) -> bool:
        """Check if running in development mode."""
        return self.environment == "development"

    @property
    def is_production(self) -> bool:
        """Check if running in production mode."""
        return self.environment == "production"

    @property
    def jwt_configured(self) -> bool:
        """Check if JWT keys are configured."""
        return self.jwt_private_key is not None and self.jwt_public_key is not None

    @property
    def google_oauth_configured(self) -> bool:
        """Check if Google OAuth is configured."""
        return (
            self.google_client_id is not None
            and self.google_client_secret is not None
            and self.google_redirect_uri is not None
        )

    @property
    def github_oauth_configured(self) -> bool:
        """Check if GitHub OAuth is configured."""
        return (
            self.github_client_id is not None
            and self.github_client_secret is not None
            and self.github_redirect_uri is not None
        )


# Global settings instance
settings = Settings()
