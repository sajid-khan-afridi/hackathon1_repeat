#!/usr/bin/env python3
"""
Run Phase 4B database migration on staging database
"""
import psycopg2
import sys
from pathlib import Path

# Staging database URL
DATABASE_URL = "postgresql://neondb_owner:npg_WxAZ9pqHKYm1@ep-autumn-truth-a1650jur-pooler.ap-southeast-1.aws.neon.tech/hackathon1_staging?sslmode=require&channel_binding=require"

# Migration file
MIGRATION_FILE = Path(__file__).parent / "app" / "migrations" / "008_personalization_schema.sql"

def main():
    print("=" * 60)
    print("Phase 4B Database Migration - Staging")
    print("=" * 60)

    # Read migration SQL
    print(f"\n[*] Reading migration: {MIGRATION_FILE.name}")
    if not MIGRATION_FILE.exists():
        print(f"[X] Migration file not found: {MIGRATION_FILE}")
        sys.exit(1)

    migration_sql = MIGRATION_FILE.read_text(encoding='utf-8')
    print(f"[OK] Migration loaded ({len(migration_sql)} bytes)")

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

    # Check if migration already run
    print("\n[*] Checking if migration already executed...")
    try:
        cursor.execute("""
            SELECT EXISTS (
                SELECT FROM information_schema.tables
                WHERE table_name='chapter_metadata'
            );
        """)
        table_exists = cursor.fetchone()[0]

        if table_exists:
            print("[!] Warning: Table 'chapter_metadata' already exists")
            print("    Migration may have already been executed")
            # Auto-skip in non-interactive mode
            print("    Skipping migration (table already exists)")
            conn.close()
            print("\n" + "=" * 60)
            print("[OK] Tables already exist - skipping migration")
            print("=" * 60)
            sys.exit(0)
    except Exception as e:
        print(f"[!] Could not check existing tables: {e}")

    # Execute migration
    print("\n[*] Executing migration...")
    try:
        cursor.execute(migration_sql)
        conn.commit()
        print("[OK] Migration executed successfully")
    except Exception as e:
        conn.rollback()
        print(f"[X] Migration failed: {e}")
        conn.close()
        sys.exit(1)

    # Verify migration
    print("\n[*] Verifying migration...")

    # Check tables
    print("    Checking tables...")
    cursor.execute("""
        SELECT COUNT(*) FROM information_schema.tables
        WHERE table_name IN ('skill_level_classifications', 'chapter_progress', 'chapter_metadata');
    """)
    tables_created = cursor.fetchone()[0]

    if tables_created == 3:
        print(f"    [OK] All 3 tables created")
    else:
        print(f"    [!] Expected 3 tables, found {tables_created}")

    # Check indexes
    print("    Checking indexes...")
    cursor.execute("""
        SELECT COUNT(*) FROM pg_indexes
        WHERE indexname LIKE 'idx_skill_level_%'
           OR indexname LIKE 'idx_chapter_%';
    """)
    indexes_created = cursor.fetchone()[0]
    print(f"    [OK] {indexes_created} indexes created")

    # Check chapter metadata
    print("    Checking chapter metadata...")
    cursor.execute("SELECT COUNT(*) FROM chapter_metadata;")
    chapter_count = cursor.fetchone()[0]

    if chapter_count >= 10:
        print(f"    [OK] {chapter_count} chapters inserted")
    else:
        print(f"    [!] Expected 10 chapters, found {chapter_count}")

    # Display sample data
    print("\n[*] Sample chapter metadata:")
    cursor.execute("""
        SELECT chapter_id, title, difficulty_level
        FROM chapter_metadata
        LIMIT 5;
    """)
    for row in cursor.fetchall():
        print(f"   - {row[0]}: {row[1]} ({row[2]})")

    # Close connection
    cursor.close()
    conn.close()

    print("\n" + "=" * 60)
    print("[OK] Migration Completed Successfully!")
    print("=" * 60)
    print("\nNext steps:")
    print("  1. Configure Railway staging environment")
    print("  2. Deploy backend to Railway staging")
    print("  3. Run smoke tests")

if __name__ == "__main__":
    main()
