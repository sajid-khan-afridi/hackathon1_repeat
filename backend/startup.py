"""Startup script to initialize the RAG chatbot services."""

import asyncio
import logging

from app.config import get_settings
from app.services.rag_service import rag_service
from app.middleware.logging import configure_logging

# Configure logging
configure_logging()
logger = logging.getLogger(__name__)


async def initialize_services():
    """Initialize all services."""
    settings = get_settings()
    logger.info(f"Initializing services in {settings.environment} environment")

    try:
        # Initialize RAG service (which initializes all others)
        await rag_service.initialize()

        # Create Qdrant collection if needed
        await rag_service.vector_service.create_collection()

        # Run database migrations
        await run_database_migrations()

        logger.info("All services initialized successfully")

    except Exception as e:
        logger.error(f"Failed to initialize services: {e}", exc_info=True)
        raise


async def run_database_migrations():
    """Run database migrations."""
    import asyncpg
    from app.config import get_settings

    settings = get_settings()

    # Connect to database
    conn = await asyncpg.connect(settings.database_url)

    try:
        # Read and run migration files
        import os
        migrations_dir = os.path.join(os.path.dirname(__file__), "app", "migrations")

        for filename in sorted(os.listdir(migrations_dir)):
            if filename.endswith(".sql"):
                logger.info(f"Running migration: {filename}")
                with open(os.path.join(migrations_dir, filename), "r") as f:
                    migration_sql = f.read()
                await conn.execute(migration_sql)

        logger.info("Database migrations completed")

    finally:
        await conn.close()


if __name__ == "__main__":
    asyncio.run(initialize_services())