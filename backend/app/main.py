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

        # Run critical migrations on startup
        await run_startup_migrations(db_pool)

        # Initialize RecommendationService with the pool
        initialize_recommendation_service(db_pool)
        logger.info("RecommendationService initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize database pool or services: {e}")
        # Don't fail startup - recommendations will just return errors
        # Other endpoints can still create individual connections


async def run_startup_migrations(pool: asyncpg.Pool) -> None:
    """Run critical database migrations on startup."""
    logger.info("Running startup migrations...")

    async with pool.acquire() as conn:
        # Create skill_level_classifications table if not exists
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS skill_level_classifications (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                user_id UUID NOT NULL UNIQUE REFERENCES users(id) ON DELETE CASCADE,
                skill_level VARCHAR(20) NOT NULL CHECK (skill_level IN ('beginner', 'intermediate', 'advanced')),
                calculated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
                updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
                based_on_profile JSONB NOT NULL
            )
        """)
        await conn.execute("CREATE INDEX IF NOT EXISTS idx_skill_level_user ON skill_level_classifications(user_id)")
        await conn.execute("CREATE INDEX IF NOT EXISTS idx_skill_level_tier ON skill_level_classifications(skill_level)")
        logger.info("skill_level_classifications table ready")

        # Create chapter_progress table if not exists
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS chapter_progress (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
                chapter_id VARCHAR(255) NOT NULL,
                status VARCHAR(20) NOT NULL CHECK (status IN ('started', 'completed')),
                is_bookmarked BOOLEAN NOT NULL DEFAULT FALSE,
                started_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
                completed_at TIMESTAMP WITH TIME ZONE,
                updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
                CONSTRAINT unique_user_chapter UNIQUE (user_id, chapter_id)
            )
        """)
        await conn.execute("CREATE INDEX IF NOT EXISTS idx_chapter_progress_user ON chapter_progress(user_id)")
        await conn.execute("CREATE INDEX IF NOT EXISTS idx_chapter_progress_status ON chapter_progress(status)")
        logger.info("chapter_progress table ready")

        # Create chapter_metadata table if not exists
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS chapter_metadata (
                chapter_id VARCHAR(255) PRIMARY KEY,
                module_number INTEGER NOT NULL CHECK (module_number > 0),
                title VARCHAR(500) NOT NULL,
                difficulty_level VARCHAR(20) NOT NULL CHECK (difficulty_level IN ('beginner', 'intermediate', 'advanced')),
                prerequisites JSONB NOT NULL DEFAULT '[]',
                requires_hardware BOOLEAN NOT NULL DEFAULT FALSE,
                learning_goal_tags JSONB NOT NULL DEFAULT '[]',
                created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
                updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
            )
        """)
        await conn.execute("CREATE INDEX IF NOT EXISTS idx_chapter_metadata_difficulty ON chapter_metadata(difficulty_level)")
        await conn.execute("CREATE INDEX IF NOT EXISTS idx_chapter_metadata_module ON chapter_metadata(module_number)")
        logger.info("chapter_metadata table ready")

        # Insert sample chapter metadata if empty
        count = await conn.fetchval("SELECT COUNT(*) FROM chapter_metadata")
        if count == 0:
            await conn.execute("""
                INSERT INTO chapter_metadata (chapter_id, module_number, title, difficulty_level, prerequisites, requires_hardware, learning_goal_tags) VALUES
                ('module-1/ros-intro', 1, 'Introduction to ROS 2', 'beginner', '[]', FALSE, '["theoretical", "practical"]'),
                ('module-1/linux-basics', 1, 'Linux Basics for Robotics', 'beginner', '[]', FALSE, '["practical"]'),
                ('module-1/python-basics', 1, 'Python Programming for ROS', 'beginner', '[]', FALSE, '["practical"]'),
                ('module-2/ros-publishers', 2, 'Creating ROS 2 Publishers', 'intermediate', '["module-1/ros-intro", "module-1/python-basics"]', FALSE, '["practical", "theoretical"]'),
                ('module-2/ros-subscribers', 2, 'Creating ROS 2 Subscribers', 'intermediate', '["module-1/ros-intro", "module-1/python-basics"]', FALSE, '["practical", "theoretical"]'),
                ('module-3/advanced-control', 3, 'Advanced Robot Control', 'advanced', '["module-2/ros-publishers", "module-2/ros-subscribers"]', TRUE, '["practical", "research"]')
                ON CONFLICT (chapter_id) DO NOTHING
            """)
            logger.info("Inserted sample chapter metadata")

    logger.info("Startup migrations completed")


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
