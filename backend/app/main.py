"""
FastAPI application entry point for RAG Chatbot Core.
Version: 2.0.0 - Added auth, personalization, and progress endpoints
"""

import logging
import asyncpg
from fastapi import FastAPI, Request, HTTPException
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException
from app.config import settings
from app.middleware.logging import LoggingMiddleware
from app.middleware.cors import configure_cors
from app.middleware.rate_limit import UserIdentificationMiddleware
from app.middleware.csrf import configure_csrf_protection
from app.services.recommendation_service import initialize_recommendation_service

# Configure logging
logging.basicConfig(
    level=logging.INFO if settings.is_development else logging.WARNING,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# Global database pool for services that need it
db_pool: asyncpg.Pool | None = None

# Create FastAPI application
app = FastAPI(
    title="RAG Chatbot API",
    description="AI-powered question answering system for robotics textbook",
    version="2.0.0",
    docs_url="/api/docs" if settings.is_development else None,
    redoc_url="/api/redoc" if settings.is_development else None,
)

# Middleware order matters! In Starlette, middleware added LAST runs FIRST (outermost).
# CORS must be outermost to handle OPTIONS preflight before other middleware.

# Configure CSRF protection (FR-027) - runs after CORS
configure_csrf_protection(
    app,
    cookie_secure=not settings.is_development,  # Secure cookies in production only
)

# Add user identification middleware
app.add_middleware(UserIdentificationMiddleware)

# Add logging middleware
app.add_middleware(LoggingMiddleware)

# Configure CORS - MUST BE LAST so it runs FIRST (outermost middleware)
# This ensures OPTIONS preflight requests are handled before any other middleware
configure_cors(app, settings.cors_origins_list)


def add_cors_headers(response: JSONResponse, request: Request) -> JSONResponse:
    """Add CORS headers to response based on request origin."""
    origin = request.headers.get("origin", "")
    allowed_origins = settings.cors_origins_list

    # If origin is in allowed list, add CORS headers
    if origin in allowed_origins or "*" in allowed_origins:
        response.headers["Access-Control-Allow-Origin"] = origin or "*"
        response.headers["Access-Control-Allow-Credentials"] = "true"
        response.headers["Access-Control-Allow-Methods"] = "GET, POST, PUT, DELETE, OPTIONS, PATCH"
        response.headers["Access-Control-Allow-Headers"] = "*"

    return response


@app.exception_handler(StarletteHTTPException)
async def cors_http_exception_handler(request: Request, exc: StarletteHTTPException) -> JSONResponse:
    """
    Custom HTTP exception handler that includes CORS headers.
    This ensures CSRF errors (403) and other HTTP errors have proper CORS headers.
    """
    response = JSONResponse(
        status_code=exc.status_code,
        content={"detail": exc.detail},
    )
    return add_cors_headers(response, request)


@app.exception_handler(RequestValidationError)
async def cors_validation_exception_handler(request: Request, exc: RequestValidationError) -> JSONResponse:
    """
    Custom validation exception handler that includes CORS headers.
    """
    response = JSONResponse(
        status_code=422,
        content={"detail": exc.errors()},
    )
    return add_cors_headers(response, request)


@app.exception_handler(Exception)
async def cors_generic_exception_handler(request: Request, exc: Exception) -> JSONResponse:
    """
    Generic exception handler that includes CORS headers.
    Ensures all unhandled exceptions still return proper CORS headers.
    """
    logger.error(f"Unhandled exception: {exc}", exc_info=True)
    response = JSONResponse(
        status_code=500,
        content={"detail": "Internal server error"},
    )
    return add_cors_headers(response, request)


@app.on_event("startup")
async def startup_event() -> None:
    """Initialize services on application startup."""
    global db_pool
    logger.info("Starting RAG Chatbot API")
    logger.info(f"Environment: {settings.environment}")
    logger.info(f"CORS Origins: {settings.cors_origins_list}")

    # Initialize database connection pool
    try:
        db_pool = await asyncpg.create_pool(
            settings.database_url,
            min_size=2,
            max_size=10,
            command_timeout=60,
        )
        logger.info("Database connection pool created successfully")

        # Initialize RecommendationService with the pool
        initialize_recommendation_service(db_pool)
        logger.info("RecommendationService initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize database pool or services: {e}")
        # Don't fail startup - recommendations will just return errors
        # Other endpoints can still create individual connections


@app.on_event("shutdown")
async def shutdown_event() -> None:
    """Cleanup on application shutdown."""
    global db_pool
    logger.info("Shutting down RAG Chatbot API")

    # Close database connection pool
    if db_pool:
        await db_pool.close()
        logger.info("Database connection pool closed")


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


@app.get("/debug/cors")
async def debug_cors() -> JSONResponse:
    """Debug endpoint to verify CORS configuration."""
    return JSONResponse(content={
        "cors_origins": settings.cors_origins_list,
        "environment": settings.environment,
        "middleware_order": "CSRF -> UserID -> Logging -> CORS (outermost)",
    })


# Import and include routers
from app.routers import health, query, sessions, auth, oauth, users, personalization, progress

app.include_router(health.router, prefix="/api/v1", tags=["health"])
app.include_router(query.router, prefix="/api/v1", tags=["query"])
app.include_router(sessions.router, prefix="/api/v1/chat", tags=["sessions"])
app.include_router(auth.router, prefix="/auth", tags=["authentication"])
app.include_router(oauth.router, prefix="/auth", tags=["oauth"])
app.include_router(users.router, prefix="/users", tags=["users"])
app.include_router(personalization.router, tags=["personalization"])
app.include_router(progress.router, tags=["progress"])
