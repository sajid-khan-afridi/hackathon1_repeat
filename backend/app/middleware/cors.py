"""CORS configuration and middleware."""

from typing import List

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.config import get_settings


def configure_cors(app: FastAPI) -> None:
    """Configure CORS middleware for the application."""
    settings = get_settings()

    # Prepare CORS configuration
    allow_origins = (
        settings.cors_origins
        if isinstance(settings.cors_origins, list)
        else [settings.cors_origins]
    )

    # In development, allow localhost with any port
    if settings.is_development:
        allow_origins.extend(
            [
                "http://localhost:3000",
                "http://localhost:3001",
                "http://localhost:8000",
                "http://127.0.0.1:3000",
                "http://127.0.0.1:3001",
                "http://127.0.0.1:8000",
            ]
        )

    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=allow_origins,
        allow_credentials=True,
        allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
        allow_headers=[
            "Accept",
            "Accept-Language",
            "Content-Language",
            "Content-Type",
            "Authorization",
            "X-Requested-With",
            "X-Correlation-ID",
            "X-Session-Token",
        ],
        expose_headers=[
            "X-Correlation-ID",
            "X-RateLimit-Limit",
            "X-RateLimit-Remaining",
            "X-RateLimit-Reset",
            "Retry-After",
        ],
    )


def get_cors_origins() -> List[str]:
    """Get list of allowed CORS origins."""
    settings = get_settings()

    origins = (
        settings.cors_origins
        if isinstance(settings.cors_origins, list)
        else [settings.cors_origins]
    )

    # Add development origins if needed
    if settings.is_development:
        origins.extend(
            [
                "http://localhost:3000",
                "http://localhost:8000",
            ]
        )

    # Remove duplicates
    return list(set(origins))
