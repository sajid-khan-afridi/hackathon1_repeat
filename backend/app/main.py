"""
FastAPI application entry point for RAG Chatbot Core.
"""

import logging
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from app.config import settings
from app.middleware.logging import LoggingMiddleware
from app.middleware.cors import configure_cors
from app.middleware.rate_limit import UserIdentificationMiddleware
from app.middleware.csrf import configure_csrf_protection

# Configure logging
logging.basicConfig(
    level=logging.INFO if settings.is_development else logging.WARNING,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# Create FastAPI application
app = FastAPI(
    title="RAG Chatbot API",
    description="AI-powered question answering system for robotics textbook",
    version="1.0.0",
    docs_url="/api/docs" if settings.is_development else None,
    redoc_url="/api/redoc" if settings.is_development else None,
)

# Configure CORS
configure_cors(app, settings.cors_origins_list)

# Configure CSRF protection (FR-027)
configure_csrf_protection(
    app,
    cookie_secure=not settings.is_development,  # Secure cookies in production only
)

# Add user identification middleware
app.add_middleware(UserIdentificationMiddleware)

# Add logging middleware
app.add_middleware(LoggingMiddleware)


@app.on_event("startup")
async def startup_event() -> None:
    """Initialize services on application startup."""
    logger.info("Starting RAG Chatbot API")
    logger.info(f"Environment: {settings.environment}")
    logger.info(f"CORS Origins: {settings.cors_origins_list}")


@app.on_event("shutdown")
async def shutdown_event() -> None:
    """Cleanup on application shutdown."""
    logger.info("Shutting down RAG Chatbot API")


@app.get("/")
async def root() -> JSONResponse:
    """Root endpoint."""
    return JSONResponse(
        content={
            "message": "RAG Chatbot API",
            "version": "1.0.0",
            "docs": "/api/docs" if settings.is_development else None,
        }
    )


@app.get("/health")
async def simple_health() -> JSONResponse:
    """
    Simple health check endpoint that always returns 200 OK.
    Used by Railway for startup health checks.
    For detailed health status, use /api/v1/health
    """
    return JSONResponse(content={"status": "ok"})


# Import and include routers
from app.routers import health, query, sessions, auth, oauth, users

app.include_router(health.router, prefix="/api/v1", tags=["health"])
app.include_router(query.router, prefix="/api/v1", tags=["query"])
app.include_router(sessions.router, prefix="/api/v1/chat", tags=["sessions"])
app.include_router(auth.router, prefix="/auth", tags=["authentication"])
app.include_router(oauth.router, prefix="/auth", tags=["oauth"])
app.include_router(users.router, prefix="/users", tags=["users"])
