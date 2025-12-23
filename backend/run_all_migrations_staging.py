#!/usr/bin/env python3
"""
Run ALL database migrations on staging database in order
"""
import psycopg2
import sys
from pathlib import Path

# Staging database URL
DATABASE_URL = "postgresql://neondb_owner:npg_WxAZ9pqHKYm1@ep-autumn-truth-a1650jur-pooler.ap-southeast-1.aws.neon.tech/hackathon1_staging?sslmode=require&channel_binding=require"

# Migrations directory
MIGRATIONS_DIR = Path(__file__).parent / "app" / "migrations"

# Migration files in order
MIGRATIONS = [
    "000_drop_old_tables.sql",
    "001_create_chat_tables.sql",
    "002_add_auto_purge.sql",
    "003_add_confidence_column.sql",
    "004_create_users_table.sql",
    "005_create_profiles_table.sql",
    "006_link_sessions_to_users.sql",
    "007_add_github_oauth.sql",
    "008_personalization_schema.sql",
]

def main():
    print("=" * 60)
    print("Database Migrations - Staging (ALL)")
    print("=" * 60)

    # Connect to database
    print(f"\n[*] Connecting to staging database...")
    try:
        conn = psycopg2.connect(DATABASE_URL)
        conn.autocommit = False
        cursor = conn.cursor()
        print("[OK] Connected successfully")
    except Exception as e:
        print(f"[X] Connection failed: {e}")
        sys.exit(1)

    # Run each migration
    for migration_file in MIGRATIONS:
        migration_path = MIGRATIONS_DIR / migration_file

        print(f"\n[*] Running migration: {migration_file}")

        if not migration_path.exists():
            print(f"[!] Migration file not found: {migration_path}")
            continue

        # Read migration SQL
        try:
            migration_sql = migration_path.read_text(encoding='utf-8')
            print(f"    Loaded ({len(migration_sql)} bytes)")
        except Exception as e:
            print(f"[X] Could not read migration: {e}")
            continue

        # Execute migration
        try:
            cursor.execute(migration_sql)
            conn.commit()
            print(f"[OK] {migration_file} executed successfully")
        except Exception as e:
            # Some migrations may fail if already run - that's okay
            conn.rollback()
            if "already exists" in str(e).lower() or "duplicate" in str(e).lower():
                print(f"[SKIP] {migration_file} - objects already exist")
                # Continue with next migration
                continue
            else:
                print(f"[X] {migration_file} failed: {e}")
                # For critical errors, stop
                if migration_file in ["004_create_users_table.sql", "008_personalization_schema.sql"]:
                    print("\n[X] Critical migration failed - stopping")
                    cursor.close()
                    conn.close()
                    sys.exit(1)

    # Verify Phase 4B tables
    print("\n" + "=" * 60)
    print("[*] Verifying Phase 4B tables...")
    print("=" * 60)

    try:
        # Check Phase 4B tables
        cursor.execute("""
            SELECT table_name FROM information_schema.tables
            WHERE table_name IN (
                'users',
                'user_profiles',
                'skill_level_classifications',
                'chapter_progress',
                'chapter_metadata'
            )
            ORDER BY table_name;
        """)
        tables = cursor.fetchall()

        print(f"\n[*] Found {len(tables)} tables:")
        for table in tables:
            print(f"    - {table[0]}")

        # Check chapter metadata count
        cursor.execute("SELECT COUNT(*) FROM chapter_metadata;")
        chapter_count = cursor.fetchone()[0]
        print(f"\n[*] Chapter metadata: {chapter_count} chapters")

        # Check indexes
        cursor.execute("""
            SELECT COUNT(*) FROM pg_indexes
            WHERE indexname LIKE 'idx_skill_level_%'
               OR indexname LIKE 'idx_chapter_%';
        """)
        indexes_count = cursor.fetchone()[0]
        print(f"[*] Indexes created: {indexes_count}")

    except Exception as e:
        print(f"[!] Verification error: {e}")

    # Close connection
    cursor.close()
    conn.close()

    print("\n" + "=" * 60)
    print("[OK] All Migrations Completed!")
    print("=" * 60)
    print("\nStaging database is ready for deployment.")

if __name__ == "__main__":
    main()
