#!/usr/bin/env python3
"""
Database migration runner for RAG Chatbot.
Applies SQL migrations to the PostgreSQL database.
"""
import asyncio
import asyncpg
import os
import sys
from pathlib import Path
from dotenv import load_dotenv

# Load environment variables from parent directory (optional for production)
env_path = Path(__file__).parent.parent / ".env"
if env_path.exists():
    load_dotenv(env_path)
else:
    print(f"Note: .env file not found at {env_path}, using environment variables")

DATABASE_URL = os.getenv("DATABASE_URL")
MIGRATIONS_DIR = Path(__file__).parent / "app" / "migrations"


async def run_migration(conn: asyncpg.Connection, migration_file: Path) -> None:
    """Execute a single migration file."""
    print(f"Running migration: {migration_file.name}")

    sql = migration_file.read_text(encoding="utf-8")

    try:
        await conn.execute(sql)
        print(f"[OK] Successfully applied {migration_file.name}")
    except Exception as e:
        print(f"[FAIL] Failed to apply {migration_file.name}: {e}")
        raise


async def main():
    """Main migration runner."""
    if not DATABASE_URL:
        print("ERROR: DATABASE_URL environment variable not set")
        print(f"Looked for .env at: {env_path}")
        sys.exit(1)

    print("=" * 60)
    print("RAG Chatbot - Database Migration Runner")
    print("=" * 60)
    print(f"Database: {DATABASE_URL.split('@')[1] if '@' in DATABASE_URL else 'hidden'}")
    print(f"Migrations directory: {MIGRATIONS_DIR}")
    print("=" * 60)

    # Get all migration files (sorted)
    migration_files = sorted(MIGRATIONS_DIR.glob("*.sql"))

    if not migration_files:
        print("No migration files found!")
        return

    print(f"Found {len(migration_files)} migration(s):")
    for f in migration_files:
        print(f"  - {f.name}")
    print()

    # Connect to database
    try:
        print("Connecting to database...")
        conn = await asyncpg.connect(DATABASE_URL, timeout=10.0)
        print("[OK] Connected successfully\n")
    except Exception as e:
        print(f"[FAIL] Failed to connect to database: {e}")
        sys.exit(1)

    try:
        # Run migrations in transaction
        async with conn.transaction():
            for migration_file in migration_files:
                await run_migration(conn, migration_file)

        print("\n" + "=" * 60)
        print("[SUCCESS] All migrations completed successfully!")
        print("=" * 60)

        # Verify tables were created
        print("\nVerifying tables...")
        tables = await conn.fetch("""
            SELECT table_name
            FROM information_schema.tables
            WHERE table_schema = 'public'
            AND table_type = 'BASE TABLE'
            ORDER BY table_name
        """)

        print(f"Found {len(tables)} table(s):")
        for table in tables:
            print(f"  - {table['table_name']}")

    except Exception as e:
        print(f"\n[FAIL] Migration failed: {e}")
        sys.exit(1)
    finally:
        await conn.close()


if __name__ == "__main__":
    asyncio.run(main())
