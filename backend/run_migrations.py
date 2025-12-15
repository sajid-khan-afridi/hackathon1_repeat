"""Database migration runner script.

This script runs SQL migrations in order to set up the database schema.
"""

import os
import sys
from pathlib import Path

import psycopg2
from dotenv import load_dotenv


def run_migration(cursor, migration_file: Path) -> None:
    """Run a single migration file.

    Args:
        cursor: Database cursor
        migration_file: Path to the migration SQL file
    """
    print(f"Running migration: {migration_file.name}")

    with open(migration_file, 'r', encoding='utf-8') as f:
        sql = f.read()

    try:
        cursor.execute(sql)
        print(f"[OK] Successfully applied {migration_file.name}")
    except Exception as e:
        print(f"[ERROR] Error applying {migration_file.name}: {e}")
        raise


def main():
    """Main migration runner."""
    # Load environment variables
    env_path = Path(__file__).parent.parent / '.env'
    if env_path.exists():
        load_dotenv(env_path)
    else:
        print(f"Warning: .env file not found at {env_path}")

    # Get database URL
    database_url = os.getenv('DATABASE_URL')
    if not database_url:
        print("Error: DATABASE_URL environment variable not set")
        sys.exit(1)

    # Migration files directory
    migrations_dir = Path(__file__).parent / 'app' / 'migrations'

    # Get ordered list of migration files
    migration_files = sorted([
        f for f in migrations_dir.glob('*.sql')
        if f.name.startswith(('001_', '002_'))  # Only numbered migrations
    ])

    if not migration_files:
        print(f"No migration files found in {migrations_dir}")
        sys.exit(1)

    print(f"Found {len(migration_files)} migration(s) to run")
    print("-" * 60)

    # Connect to database
    conn = None
    try:
        print("Connecting to database...")
        conn = psycopg2.connect(database_url)
        conn.autocommit = False  # Use transactions
        cursor = conn.cursor()

        # Run migrations in order
        for migration_file in migration_files:
            run_migration(cursor, migration_file)

        # Commit all migrations
        conn.commit()
        print("-" * 60)
        print(f"[OK] All {len(migration_files)} migration(s) applied successfully!")

        # Verify tables were created
        cursor.execute("""
            SELECT table_name
            FROM information_schema.tables
            WHERE table_schema = 'public'
            AND table_name IN ('chat_sessions', 'chat_messages', 'source_citations', 'scheduled_jobs')
            ORDER BY table_name
        """)
        tables = cursor.fetchall()

        print("\nCreated tables:")
        for table in tables:
            print(f"  - {table[0]}")

    except Exception as e:
        print(f"\n[ERROR] Migration failed: {e}")
        if conn:
            conn.rollback()
            print("[OK] Changes rolled back")
        sys.exit(1)

    finally:
        if conn:
            conn.close()
            print("\nDatabase connection closed")


if __name__ == "__main__":
    main()
