"""
CORS middleware configuration.
"""

from fastapi.middleware.cors import CORSMiddleware
from typing import List


def configure_cors(app, allowed_origins: List[str]) -> None:
    """
    Configure CORS middleware for the FastAPI application.

    Args:
        app: FastAPI application instance
        allowed_origins: List of allowed origin URLs
    """
    app.add_middleware(
        CORSMiddleware,
        allow_origins=allowed_origins,
        allow_credentials=True,
        allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
        allow_headers=[
            "Content-Type",
            "Authorization",
            "X-Correlation-ID",
            "X-Session-Token",
            "X-CSRF-Token",  # Required for CSRF protection in cross-origin requests
        ],
        expose_headers=[
            "X-Correlation-ID",
            "X-RateLimit-Limit",
            "X-RateLimit-Remaining",
            "X-RateLimit-Reset",
        ],
    )
