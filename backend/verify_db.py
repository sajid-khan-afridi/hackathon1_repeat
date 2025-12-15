"""Verify database health after migrations."""

import os
import sys
from pathlib import Path

import psycopg2
from dotenv import load_dotenv


def main():
    """Verify database setup."""
    # Load environment variables
    env_path = Path(__file__).parent.parent / '.env'
    if env_path.exists():
        load_dotenv(env_path)

    # Get database URL
    database_url = os.getenv('DATABASE_URL')
    if not database_url:
        print("[ERROR] DATABASE_URL environment variable not set")
        sys.exit(1)

    try:
        # Connect to database
        print("Connecting to database...")
        conn = psycopg2.connect(database_url)
        cursor = conn.cursor()

        print("[OK] Database connection successful")

        # Check required tables
        required_tables = ['chat_sessions', 'chat_messages', 'source_citations', 'scheduled_jobs']

        cursor.execute("""
            SELECT table_name
            FROM information_schema.tables
            WHERE table_schema = 'public'
            ORDER BY table_name
        """)
        existing_tables = [row[0] for row in cursor.fetchall()]

        print(f"\nFound {len(existing_tables)} table(s) in database:")
        for table in existing_tables:
            status = "[OK]" if table in required_tables else "[INFO]"
            print(f"  {status} {table}")

        # Verify all required tables exist
        missing_tables = set(required_tables) - set(existing_tables)
        if missing_tables:
            print(f"\n[ERROR] Missing required tables: {', '.join(missing_tables)}")
            sys.exit(1)

        print(f"\n[OK] All required tables exist")

        # Test a simple query
        cursor.execute("SELECT COUNT(*) FROM chat_sessions")
        count = cursor.fetchone()[0]
        print(f"[OK] chat_sessions table accessible (current count: {count})")

        # Check functions
        cursor.execute("""
            SELECT routine_name
            FROM information_schema.routines
            WHERE routine_schema = 'public'
            AND routine_type = 'FUNCTION'
            ORDER BY routine_name
        """)
        functions = [row[0] for row in cursor.fetchall()]

        print(f"\nFound {len(functions)} function(s):")
        for func in functions:
            print(f"  - {func}")

        conn.close()

        print("\n" + "=" * 60)
        print("[OK] Database health check PASSED")
        print("=" * 60)

    except Exception as e:
        print(f"\n[ERROR] Database health check FAILED: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
