"""FastAPI application entry point."""

import logging
import time
from contextlib import asynccontextmanager
from typing import AsyncGenerator

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from slowapi import _rate_limit_exceeded_handler
from slowapi.errors import RateLimitExceeded

from app.config import get_settings
from app.middleware.logging import LoggingMiddleware
from app.middleware.rate_limit import RateLimitMiddleware, create_rate_limit_response
from app.services.rate_limiter import limiter, get_user_identifier, is_authenticated
from app.routers import health, query
from app.models import HealthCheck, ErrorResponse

# Configure logging
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    """Manage application lifecycle."""
    # Startup
    logger.info("Starting RAG Chatbot API...")
    settings = get_settings()

    # Log configuration
    logger.info(f"Environment: {settings.environment}")
    logger.info(f"Log level: {settings.log_level}")
    logger.info(f"CORS origins: {settings.cors_origins}")

    # Initialize services
    from app.services.chat_service import chat_service
    from app.services.rag_service import rag_service

    try:
        await chat_service.initialize()
        logger.info("Chat service initialized")
        await rag_service.initialize()
        logger.info("RAG service initialized")
    except Exception as e:
        logger.error(f"Failed to initialize services: {e}")
        raise

    yield

    # Shutdown
    logger.info("Shutting down RAG Chatbot API...")


# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for RAG-powered chatbot to answer textbook questions",
    version="0.1.0",
    lifespan=lifespan,
    docs_url="/docs" if get_settings().is_development else None,
    redoc_url="/redoc" if get_settings().is_development else None,
)

# Add rate limiter state to app
app.state.limiter = limiter


# Add CORS middleware
settings = get_settings()
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE"],
    allow_headers=["*"],
)


# Add logging middleware
app.add_middleware(LoggingMiddleware)

# Add rate limit middleware
app.add_middleware(RateLimitMiddleware)


# Rate limit exception handler
@app.exception_handler(RateLimitExceeded)
async def rate_limit_exceeded_handler(request: Request, exc: RateLimitExceeded) -> JSONResponse:
    """Handle rate limit exceeded exceptions with custom response."""
    settings = get_settings()
    identifier = get_user_identifier(request)

    # Determine limit based on user type
    if is_authenticated(request):
        limit = settings.rate_limit_authenticated
    else:
        limit = settings.rate_limit_anonymous

    # Calculate reset time (1 hour from now for hourly limits)
    reset_after = 3600  # 1 hour in seconds

    logger.warning(f"Rate limit exceeded for {identifier}")

    return create_rate_limit_response(limit=limit, reset_after=reset_after, identifier=identifier)


# Exception handlers
@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception) -> JSONResponse:
    """Handle uncaught exceptions."""
    logger.error(f"Unhandled exception: {exc}", exc_info=True)

    if get_settings().is_development:
        return JSONResponse(
            status_code=500,
            content=ErrorResponse(
                error="Internal server error",
                code="INTERNAL_ERROR",
                details={"exception": str(exc), "type": type(exc).__name__},
            ).dict(),
        )

    return JSONResponse(
        status_code=500,
        content=ErrorResponse(error="Internal server error", code="INTERNAL_ERROR").dict(),
    )


@app.exception_handler(ValueError)
async def value_error_handler(request: Request, exc: ValueError) -> JSONResponse:
    """Handle ValueError exceptions."""
    logger.warning(f"ValueError: {exc}")

    return JSONResponse(
        status_code=400,
        content=ErrorResponse(
            error=str(exc),
            code="INVALID_VALUE",
            suggestedAction="Please check your request parameters",
        ).dict(),
    )


# Include routers
app.include_router(health.router, prefix="/api/v1", tags=["Health"])
app.include_router(query.router, prefix="/api/v1", tags=["Query"])


# Root endpoint
@app.get("/", tags=["Root"])
async def root() -> dict:
    """Root endpoint with basic info."""
    return {
        "name": "RAG Chatbot API",
        "version": "0.1.0",
        "status": "running",
        "environment": get_settings().environment,
        "docs": "/docs" if get_settings().is_development else None,
    }


# Health check endpoint
@app.get("/health", response_model=HealthCheck, tags=["Health"])
async def health_check() -> HealthCheck:
    """Simple health check endpoint."""
    from datetime import datetime

    return HealthCheck(
        status="healthy",
        timestamp=datetime.utcnow(),
        services={
            "api": "healthy",
            "database": "unknown",
            "qdrant": "unknown",
            "openai": "unknown",
        },
    )


# Run with: uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
