#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Database Migration Execution Script
Feature: 001-user-auth
Date: 2025-12-20

This script executes all pending database migrations in the correct order
with proper error handling, validation, and rollback capabilities.

Usage:
    python backend/app/migrations/run_migrations.py [--dry-run] [--rollback]
"""

import os
import sys
import argparse
import psycopg2
from pathlib import Path
from datetime import datetime
from typing import List, Tuple

# Set UTF-8 encoding for Windows console
if sys.platform == "win32":
    import codecs
    sys.stdout = codecs.getwriter("utf-8")(sys.stdout.detach())
    sys.stderr = codecs.getwriter("utf-8")(sys.stderr.detach())

# Add project root to path
project_root = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(project_root))

from backend.app.config import settings


class MigrationExecutor:
    """Execute database migrations with validation and rollback."""

    def __init__(self, dry_run: bool = False):
        self.dry_run = dry_run
        self.migrations_dir = Path(__file__).parent
        self.conn = None
        self.cursor = None

    def connect(self) -> None:
        """Connect to the database."""
        try:
            self.conn = psycopg2.connect(settings.database_url)
            self.cursor = self.conn.cursor()
            print(f"✅ Connected to database: {settings.database_url.split('@')[1].split('/')[0]}")
        except Exception as e:
            print(f"❌ Failed to connect to database: {e}")
            sys.exit(1)

    def disconnect(self) -> None:
        """Close database connection."""
        if self.cursor:
            self.cursor.close()
        if self.conn:
            self.conn.close()
        print("✅ Database connection closed")

    def get_migrations(self) -> List[Tuple[int, str, Path]]:
        """
        Get all migration files in order.

        Returns:
            List of tuples: (order, name, path)
        """
        migrations = []
        migration_files = sorted(self.migrations_dir.glob("*.sql"))

        for file_path in migration_files:
            # Extract order number from filename (e.g., 004_create_users_table.sql -> 4)
            try:
                order = int(file_path.stem.split("_")[0])
                name = "_".join(file_path.stem.split("_")[1:])
                migrations.append((order, name, file_path))
            except (ValueError, IndexError):
                print(f"⚠️  Skipping invalid migration file: {file_path.name}")
                continue

        return sorted(migrations, key=lambda x: x[0])

    def create_migration_table(self) -> None:
        """Create migrations tracking table if it doesn't exist."""
        create_table_sql = """
        CREATE TABLE IF NOT EXISTS schema_migrations (
            id SERIAL PRIMARY KEY,
            version INTEGER NOT NULL UNIQUE,
            name VARCHAR(255) NOT NULL,
            executed_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
            execution_time_ms INTEGER,
            success BOOLEAN NOT NULL DEFAULT TRUE
        );

        CREATE INDEX IF NOT EXISTS idx_schema_migrations_version
        ON schema_migrations(version);
        """

        try:
            if self.dry_run:
                print("🔍 [DRY RUN] Would create schema_migrations table")
                return

            self.cursor.execute(create_table_sql)
            self.conn.commit()
            print("✅ Migration tracking table ready")
        except Exception as e:
            print(f"❌ Failed to create migration table: {e}")
            self.conn.rollback()
            raise

    def get_executed_migrations(self) -> List[int]:
        """Get list of already executed migration versions."""
        try:
            self.cursor.execute(
                "SELECT version FROM schema_migrations WHERE success = TRUE ORDER BY version"
            )
            return [row[0] for row in self.cursor.fetchall()]
        except psycopg2.errors.UndefinedTable:
            return []

    def execute_migration(self, version: int, name: str, file_path: Path) -> bool:
        """
        Execute a single migration file.

        Args:
            version: Migration version number
            name: Migration name
            file_path: Path to migration SQL file

        Returns:
            True if successful, False otherwise
        """
        print(f"\n{'='*80}")
        print(f"📝 Migration {version:03d}: {name}")
        print(f"{'='*80}")

        # Read migration file
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                sql_content = f.read()
        except Exception as e:
            print(f"❌ Failed to read migration file: {e}")
            return False

        if self.dry_run:
            print("🔍 [DRY RUN] SQL Preview:")
            print("-" * 80)
            print(sql_content[:500] + "..." if len(sql_content) > 500 else sql_content)
            print("-" * 80)
            return True

        # Execute migration
        start_time = datetime.now()
        try:
            self.cursor.execute(sql_content)
            execution_time = int((datetime.now() - start_time).total_seconds() * 1000)

            # Record migration success
            self.cursor.execute(
                """
                INSERT INTO schema_migrations (version, name, execution_time_ms, success)
                VALUES (%s, %s, %s, %s)
                ON CONFLICT (version) DO UPDATE
                SET executed_at = NOW(), execution_time_ms = EXCLUDED.execution_time_ms
                """,
                (version, name, execution_time, True)
            )

            self.conn.commit()
            print(f"✅ Migration executed successfully ({execution_time}ms)")
            return True

        except Exception as e:
            self.conn.rollback()
            print(f"❌ Migration failed: {e}")

            # Record migration failure
            try:
                self.cursor.execute(
                    """
                    INSERT INTO schema_migrations (version, name, success)
                    VALUES (%s, %s, %s)
                    ON CONFLICT (version) DO NOTHING
                    """,
                    (version, name, False)
                )
                self.conn.commit()
            except:
                pass

            return False

    def validate_schema(self) -> bool:
        """Validate database schema after migrations."""
        print(f"\n{'='*80}")
        print("🔍 Validating Database Schema")
        print(f"{'='*80}\n")

        validations = [
            ("users table exists", "SELECT to_regclass('public.users')"),
            ("user_profiles table exists", "SELECT to_regclass('public.user_profiles')"),
            ("refresh_tokens table exists", "SELECT to_regclass('public.refresh_tokens')"),
            ("login_attempts table exists", "SELECT to_regclass('public.login_attempts')"),
            ("chat_sessions table exists", "SELECT to_regclass('public.chat_sessions')"),
            ("users has github_id column",
             "SELECT column_name FROM information_schema.columns WHERE table_name='users' AND column_name='github_id'"),
            ("chat_sessions linked to users",
             "SELECT constraint_name FROM information_schema.table_constraints WHERE table_name='chat_sessions' AND constraint_name='fk_chat_sessions_user_id'"),
            ("purge functions exist",
             "SELECT COUNT(*) FROM pg_proc WHERE proname LIKE 'purge%'"),
        ]

        all_valid = True
        for description, query in validations:
            try:
                self.cursor.execute(query)
                result = self.cursor.fetchone()
                if result and result[0]:
                    print(f"✅ {description}")
                else:
                    print(f"❌ {description}")
                    all_valid = False
            except Exception as e:
                print(f"❌ {description}: {e}")
                all_valid = False

        return all_valid

    def run(self) -> int:
        """
        Execute all pending migrations.

        Returns:
            Exit code (0 for success, 1 for failure)
        """
        print("🚀 Database Migration Execution")
        print(f"Environment: {settings.environment}")
        print(f"Database: {settings.database_url.split('@')[1].split('/')[0]}")
        print(f"Mode: {'DRY RUN' if self.dry_run else 'EXECUTE'}\n")

        self.connect()

        try:
            # Create migration tracking table
            self.create_migration_table()

            # Get all migrations
            all_migrations = self.get_migrations()
            print(f"📋 Found {len(all_migrations)} migration files")

            # Get executed migrations
            executed_migrations = self.get_executed_migrations()
            print(f"✅ {len(executed_migrations)} migrations already executed")

            # Find pending migrations
            pending_migrations = [
                m for m in all_migrations if m[0] not in executed_migrations
            ]

            if not pending_migrations:
                print("\n✅ No pending migrations. Database is up to date!")
                self.validate_schema()
                return 0

            print(f"📝 {len(pending_migrations)} pending migrations:\n")
            for version, name, _ in pending_migrations:
                print(f"   {version:03d}: {name}")

            if self.dry_run:
                print("\n🔍 DRY RUN MODE - No changes will be made")

            # Execute pending migrations
            success_count = 0
            for version, name, file_path in pending_migrations:
                if self.execute_migration(version, name, file_path):
                    success_count += 1
                else:
                    print(f"\n❌ Migration {version:03d} failed. Stopping execution.")
                    return 1

            # Validate schema
            print()
            if not self.dry_run:
                if self.validate_schema():
                    print(f"\n✅ All {success_count} migrations executed successfully!")
                    print("✅ Database schema validated successfully!")
                    return 0
                else:
                    print("\n⚠️  Migrations executed but schema validation failed")
                    print("    Please review the validation errors above")
                    return 1
            else:
                print(f"\n🔍 DRY RUN COMPLETE - Would execute {len(pending_migrations)} migrations")
                return 0

        except Exception as e:
            print(f"\n❌ Migration execution failed: {e}")
            return 1

        finally:
            self.disconnect()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Execute database migrations for 001-user-auth feature"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Preview migrations without executing them"
    )

    args = parser.parse_args()

    # Verify database connection is configured
    if not settings.database_url:
        print("❌ DATABASE_URL not configured in environment variables")
        print("   Please set DATABASE_URL in .env file")
        return 1

    # Execute migrations
    executor = MigrationExecutor(dry_run=args.dry_run)
    return executor.run()


if __name__ == "__main__":
    sys.exit(main())
