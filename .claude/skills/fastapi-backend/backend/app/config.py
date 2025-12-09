from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    app_name: str = "RAG Chatbot Backend"
    app_version: str = "1.0.0"
    debug: bool = False

    # CORS settings
    cors_origins: list[str] = ["http://localhost:3000", "http://localhost:8080"]

    # JWT settings
    secret_key: str = "your-secret-key-change-in-production"
    algorithm: str = "HS256"
    access_token_expire_minutes: int = 30

    # Database settings (optional)
    database_url: Optional[str] = None

    # External services
    vector_store_url: Optional[str] = None
    llm_service_url: Optional[str] = None

    class Config:
        env_file = ".env"
        case_sensitive = False


settings = Settings()