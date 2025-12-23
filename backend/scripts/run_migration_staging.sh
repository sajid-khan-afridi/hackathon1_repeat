#!/bin/bash
# Migration execution script for Neon staging database
# Phase 4B: Personalization Engine
# Usage: ./backend/scripts/run_migration_staging.sh

set -e  # Exit on error

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
MIGRATION_FILE="backend/app/migrations/008_personalization_schema.sql"
BACKUP_DIR="backend/app/migrations/backups"
MIGRATION_NAME="008_personalization_schema"

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}Phase 4B Migration Execution - Staging${NC}"
echo -e "${BLUE}Migration: ${MIGRATION_NAME}${NC}"
echo -e "${BLUE}Date: $(date '+%Y-%m-%d %H:%M:%S')${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""

# Check if STAGING_DB is set
if [ -z "$STAGING_DB" ]; then
  echo -e "${RED}ERROR: STAGING_DB environment variable not set${NC}"
  echo ""
  echo "Please set it to your Neon staging connection string:"
  echo -e "${YELLOW}  export STAGING_DB='postgresql://user:password@ep-staging-*.neon.tech/hackathon1_staging?sslmode=require'${NC}"
  echo ""
  exit 1
fi

# Verify migration file exists
if [ ! -f "$MIGRATION_FILE" ]; then
  echo -e "${RED}ERROR: Migration file not found: $MIGRATION_FILE${NC}"
  exit 1
fi

# Create backup directory
mkdir -p "$BACKUP_DIR"

echo -e "${GREEN}Step 1: Pre-migration validation${NC}"
echo "  Checking database connection..."
if psql "$STAGING_DB" -c "SELECT version();" > /dev/null 2>&1; then
  echo -e "  ${GREEN}✅ Database connection successful${NC}"
else
  echo -e "  ${RED}❌ Failed to connect to database${NC}"
  exit 1
fi

echo "  Checking prerequisite tables..."
if psql "$STAGING_DB" -c "SELECT COUNT(*) FROM users;" > /dev/null 2>&1; then
  USER_COUNT=$(psql "$STAGING_DB" -t -c "SELECT COUNT(*) FROM users;")
  echo -e "  ${GREEN}✅ users table exists (${USER_COUNT} rows)${NC}"
else
  echo -e "  ${RED}❌ users table missing - apply base migrations first${NC}"
  exit 1
fi

if psql "$STAGING_DB" -c "SELECT COUNT(*) FROM user_profiles;" > /dev/null 2>&1; then
  PROFILE_COUNT=$(psql "$STAGING_DB" -t -c "SELECT COUNT(*) FROM user_profiles;")
  echo -e "  ${GREEN}✅ user_profiles table exists (${PROFILE_COUNT} rows)${NC}"
else
  echo -e "  ${RED}❌ user_profiles table missing - apply Phase 4A migrations first${NC}"
  exit 1
fi

echo "  Checking for existing Phase 4B tables..."
EXISTING_TABLES=$(psql "$STAGING_DB" -t -c "SELECT COUNT(*) FROM information_schema.tables WHERE table_schema = 'public' AND table_name IN ('skill_level_classifications', 'chapter_progress', 'chapter_metadata');")
if [ "$EXISTING_TABLES" -gt 0 ]; then
  echo -e "  ${YELLOW}⚠️  Warning: Found $EXISTING_TABLES Phase 4B tables already exist${NC}"
  echo -e "  ${YELLOW}   Migration may fail or skip existing tables${NC}"
  echo ""
  read -p "Continue anyway? (y/n): " CONTINUE
  if [ "$CONTINUE" != "y" ]; then
    echo "Migration cancelled."
    exit 0
  fi
else
  echo -e "  ${GREEN}✅ No conflicting tables found${NC}"
fi
echo ""

echo -e "${GREEN}Step 2: Create pre-migration backup${NC}"
BACKUP_FILE="$BACKUP_DIR/staging_schema_pre_${MIGRATION_NAME}_$(date +%Y%m%d_%H%M%S).sql"
echo "  Backing up current schema..."
if pg_dump "$STAGING_DB" --schema-only > "$BACKUP_FILE" 2>/dev/null; then
  BACKUP_SIZE=$(du -h "$BACKUP_FILE" | cut -f1)
  echo -e "  ${GREEN}✅ Schema backed up successfully${NC}"
  echo -e "  ${BLUE}   Location: $BACKUP_FILE${NC}"
  echo -e "  ${BLUE}   Size: $BACKUP_SIZE${NC}"
else
  echo -e "  ${YELLOW}⚠️  Backup failed, but continuing...${NC}"
fi
echo ""

echo -e "${GREEN}Step 3: Apply migration ${MIGRATION_NAME}${NC}"
echo "  Executing migration SQL..."
if psql "$STAGING_DB" -f "$MIGRATION_FILE" 2>&1 | tee /tmp/migration_output.log; then
  echo -e "  ${GREEN}✅ Migration executed successfully${NC}"
else
  echo -e "  ${RED}❌ Migration failed${NC}"
  echo "  Check /tmp/migration_output.log for details"
  exit 1
fi
echo ""

echo -e "${GREEN}Step 4: Post-migration verification${NC}"

echo "  Verifying tables created..."
TABLE_COUNT=$(psql "$STAGING_DB" -t -c "SELECT COUNT(*) FROM information_schema.tables WHERE table_schema = 'public' AND table_name IN ('skill_level_classifications', 'chapter_progress', 'chapter_metadata');")
if [ "$TABLE_COUNT" -eq 3 ]; then
  echo -e "  ${GREEN}✅ All 3 tables created successfully${NC}"
  psql "$STAGING_DB" -t -c "SELECT '    - ' || table_name FROM information_schema.tables WHERE table_schema = 'public' AND table_name IN ('skill_level_classifications', 'chapter_progress', 'chapter_metadata') ORDER BY table_name;"
else
  echo -e "  ${RED}❌ Expected 3 tables, found $TABLE_COUNT${NC}"
  exit 1
fi

echo "  Verifying indexes created..."
INDEX_COUNT=$(psql "$STAGING_DB" -t -c "SELECT COUNT(*) FROM pg_indexes WHERE tablename IN ('skill_level_classifications', 'chapter_progress', 'chapter_metadata');")
if [ "$INDEX_COUNT" -ge 10 ]; then
  echo -e "  ${GREEN}✅ Indexes created ($INDEX_COUNT indexes)${NC}"
else
  echo -e "  ${YELLOW}⚠️  Expected 10+ indexes, found $INDEX_COUNT${NC}"
fi

echo "  Verifying chapter metadata seeded..."
METADATA_COUNT=$(psql "$STAGING_DB" -t -c "SELECT COUNT(*) FROM chapter_metadata;")
if [ "$METADATA_COUNT" -eq 10 ]; then
  echo -e "  ${GREEN}✅ Chapter metadata seeded (10 chapters)${NC}"
elif [ "$METADATA_COUNT" -gt 0 ]; then
  echo -e "  ${YELLOW}⚠️  Expected 10 chapter metadata rows, found $METADATA_COUNT${NC}"
else
  echo -e "  ${RED}❌ No chapter metadata found${NC}"
  exit 1
fi

echo "  Verifying foreign key constraints..."
FK_COUNT=$(psql "$STAGING_DB" -t -c "SELECT COUNT(*) FROM information_schema.table_constraints WHERE constraint_type = 'FOREIGN KEY' AND table_name IN ('skill_level_classifications', 'chapter_progress');")
if [ "$FK_COUNT" -ge 2 ]; then
  echo -e "  ${GREEN}✅ Foreign key constraints created ($FK_COUNT constraints)${NC}"
else
  echo -e "  ${YELLOW}⚠️  Expected 2+ foreign keys, found $FK_COUNT${NC}"
fi

echo "  Verifying check constraints..."
CHECK_COUNT=$(psql "$STAGING_DB" -t -c "SELECT COUNT(*) FROM information_schema.table_constraints WHERE constraint_type = 'CHECK' AND table_name IN ('skill_level_classifications', 'chapter_progress', 'chapter_metadata');")
if [ "$CHECK_COUNT" -ge 3 ]; then
  echo -e "  ${GREEN}✅ Check constraints created ($CHECK_COUNT constraints)${NC}"
else
  echo -e "  ${YELLOW}⚠️  Expected 3+ check constraints, found $CHECK_COUNT${NC}"
fi
echo ""

echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}✅ Migration completed successfully!${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo -e "${BLUE}Database Summary:${NC}"
psql "$STAGING_DB" -c "
SELECT
  t.table_name,
  (SELECT COUNT(*) FROM information_schema.columns WHERE table_name = t.table_name) as columns,
  COALESCE(s.n_live_tup, 0) as rows
FROM information_schema.tables t
LEFT JOIN pg_stat_user_tables s ON t.table_name = s.relname
WHERE t.table_schema = 'public'
  AND t.table_name IN ('skill_level_classifications', 'chapter_progress', 'chapter_metadata')
ORDER BY t.table_name;
"
echo ""

echo -e "${BLUE}Next Steps:${NC}"
echo "  1. ✅ Migration 008 applied to staging database"
echo "  2. ⏭️  Update Railway staging environment variables"
echo "  3. ⏭️  Deploy backend to Railway staging"
echo "  4. ⏭️  Deploy frontend to staging"
echo "  5. ⏭️  Run smoke tests (see SMOKE_TEST_CHECKLIST.md)"
echo ""
echo -e "${YELLOW}Rollback available:${NC}"
echo "  If issues occur, run: ./backend/scripts/rollback_migration_staging.sh"
echo ""
