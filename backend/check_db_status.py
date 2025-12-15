#!/usr/bin/env python3
"""
Check current database status.
"""
import asyncio
import asyncpg
import os
from pathlib import Path
from dotenv import load_dotenv

# Load environment variables
env_path = Path(__file__).parent.parent / ".env"
load_dotenv(env_path)

DATABASE_URL = os.getenv("DATABASE_URL")


async def main():
    """Check database status."""
    if not DATABASE_URL:
        print("ERROR: DATABASE_URL not set")
        return

    print("Connecting to database...")
    conn = await asyncpg.connect(DATABASE_URL, timeout=10.0)

    try:
        # Check all tables
        print("\n=== EXISTING TABLES ===")
        tables = await conn.fetch("""
            SELECT table_name
            FROM information_schema.tables
            WHERE table_schema = 'public'
            AND table_type = 'BASE TABLE'
            ORDER BY table_name
        """)

        if not tables:
            print("No tables found in public schema")
        else:
            for table in tables:
                print(f"  - {table['table_name']}")

        # Check columns for each table
        for table in tables:
            table_name = table['table_name']
            print(f"\n=== COLUMNS IN {table_name} ===")
            columns = await conn.fetch("""
                SELECT column_name, data_type, is_nullable
                FROM information_schema.columns
                WHERE table_schema = 'public'
                AND table_name = $1
                ORDER BY ordinal_position
            """, table_name)

            for col in columns:
                nullable = "NULL" if col['is_nullable'] == 'YES' else "NOT NULL"
                print(f"  - {col['column_name']}: {col['data_type']} {nullable}")

        # Check functions
        print("\n=== FUNCTIONS ===")
        functions = await conn.fetch("""
            SELECT routine_name
            FROM information_schema.routines
            WHERE routine_schema = 'public'
            ORDER BY routine_name
        """)

        if not functions:
            print("No functions found")
        else:
            for func in functions:
                print(f"  - {func['routine_name']}")

        # Check materialized views
        print("\n=== MATERIALIZED VIEWS ===")
        views = await conn.fetch("""
            SELECT matviewname
            FROM pg_matviews
            WHERE schemaname = 'public'
            ORDER BY matviewname
        """)

        if not views:
            print("No materialized views found")
        else:
            for view in views:
                print(f"  - {view['matviewname']}")

    finally:
        await conn.close()


if __name__ == "__main__":
    asyncio.run(main())
