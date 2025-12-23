#!/bin/bash
# Rollback script for Phase 4B migration (008_personalization_schema)
# DANGER: This will delete all Phase 4B tables and data
# Usage: ./backend/scripts/rollback_migration_staging.sh

set -e

# Colors
RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

BACKUP_DIR="backend/app/migrations/backups"
MIGRATION_NAME="008_personalization_schema"

echo -e "${RED}============================================${NC}"
echo -e "${RED}⚠️  MIGRATION ROLLBACK - STAGING ⚠️${NC}"
echo -e "${RED}Migration: ${MIGRATION_NAME}${NC}"
echo -e "${RED}Date: $(date '+%Y-%m-%d %H:%M:%S')${NC}"
echo -e "${RED}============================================${NC}"
echo ""
echo -e "${YELLOW}WARNING: This operation will:${NC}"
echo -e "${YELLOW}  - Drop skill_level_classifications table${NC}"
echo -e "${YELLOW}  - Drop chapter_progress table${NC}"
echo -e "${YELLOW}  - Drop chapter_metadata table${NC}"
echo -e "${YELLOW}  - Delete ALL related data permanently${NC}"
echo ""
echo -e "${RED}THIS ACTION CANNOT BE UNDONE!${NC}"
echo ""
read -p "Type 'ROLLBACK' in uppercase to confirm: " CONFIRM

if [ "$CONFIRM" != "ROLLBACK" ]; then
  echo ""
  echo -e "${GREEN}Rollback cancelled. No changes made.${NC}"
  exit 0
fi

# Check if STAGING_DB is set
if [ -z "$STAGING_DB" ]; then
  echo -e "${RED}ERROR: STAGING_DB environment variable not set${NC}"
  echo ""
  echo "Please set it to your Neon staging connection string:"
  echo -e "${YELLOW}  export STAGING_DB='postgresql://user:password@ep-staging-*.neon.tech/hackathon1_staging?sslmode=require'${NC}"
  exit 1
fi

echo ""
echo -e "${GREEN}Step 1: Verify database connection${NC}"
if psql "$STAGING_DB" -c "SELECT version();" > /dev/null 2>&1; then
  echo -e "  ${GREEN}✅ Database connection successful${NC}"
else
  echo -e "  ${RED}❌ Failed to connect to database${NC}"
  exit 1
fi
echo ""

echo -e "${GREEN}Step 2: Check current table status${NC}"
echo "  Tables to be dropped:"
psql "$STAGING_DB" -c "
SELECT
  table_name,
  COALESCE(s.n_live_tup, 0) as row_count
FROM information_schema.tables t
LEFT JOIN pg_stat_user_tables s ON t.table_name = s.relname
WHERE t.table_schema = 'public'
  AND t.table_name IN ('skill_level_classifications', 'chapter_progress', 'chapter_metadata')
ORDER BY table_name;
" 2>/dev/null || echo -e "  ${YELLOW}No Phase 4B tables found${NC}"
echo ""

echo -e "${GREEN}Step 3: Create pre-rollback backup${NC}"
mkdir -p "$BACKUP_DIR"
BACKUP_FILE="$BACKUP_DIR/pre_rollback_${MIGRATION_NAME}_$(date +%Y%m%d_%H%M%S).sql"
echo "  Creating schema backup..."
if pg_dump "$STAGING_DB" --schema-only > "$BACKUP_FILE" 2>/dev/null; then
  BACKUP_SIZE=$(du -h "$BACKUP_FILE" | cut -f1)
  echo -e "  ${GREEN}✅ Backup created successfully${NC}"
  echo -e "  ${BLUE}   Location: $BACKUP_FILE${NC}"
  echo -e "  ${BLUE}   Size: $BACKUP_SIZE${NC}"
else
  echo -e "  ${YELLOW}⚠️  Backup failed${NC}"
  read -p "Continue rollback without backup? (y/n): " CONTINUE
  if [ "$CONTINUE" != "y" ]; then
    echo "Rollback cancelled."
    exit 0
  fi
fi
echo ""

echo -e "${GREEN}Step 4: Export data before deletion (optional)${NC}"
DATA_BACKUP_FILE="$BACKUP_DIR/pre_rollback_data_${MIGRATION_NAME}_$(date +%Y%m%d_%H%M%S).sql"
echo "  Exporting table data..."
if pg_dump "$STAGING_DB" \
    --data-only \
    --table=skill_level_classifications \
    --table=chapter_progress \
    --table=chapter_metadata \
    > "$DATA_BACKUP_FILE" 2>/dev/null; then
  DATA_SIZE=$(du -h "$DATA_BACKUP_FILE" | cut -f1)
  echo -e "  ${GREEN}✅ Data exported successfully${NC}"
  echo -e "  ${BLUE}   Location: $DATA_BACKUP_FILE${NC}"
  echo -e "  ${BLUE}   Size: $DATA_SIZE${NC}"
else
  echo -e "  ${YELLOW}⚠️  Data export failed (tables may not exist)${NC}"
  rm -f "$DATA_BACKUP_FILE"
fi
echo ""

echo -e "${GREEN}Step 5: Execute rollback${NC}"
echo "  Dropping Phase 4B tables..."

psql "$STAGING_DB" <<SQL
-- Phase 4B Rollback: Drop tables in reverse dependency order

BEGIN;

-- Drop chapter_recommendations if exists (not created in 008 but may exist)
DROP TABLE IF EXISTS chapter_recommendations CASCADE;
  RAISE NOTICE 'Dropped: chapter_recommendations';

-- Drop chapter_progress (has FK to users)
DROP TABLE IF EXISTS chapter_progress CASCADE;
  RAISE NOTICE 'Dropped: chapter_progress';

-- Drop chapter_metadata (standalone)
DROP TABLE IF EXISTS chapter_metadata CASCADE;
  RAISE NOTICE 'Dropped: chapter_metadata';

-- Drop skill_level_classifications (has FK to users)
DROP TABLE IF EXISTS skill_level_classifications CASCADE;
  RAISE NOTICE 'Dropped: skill_level_classifications';

COMMIT;

SELECT 'Rollback transaction committed successfully' as status;
SQL

echo -e "  ${GREEN}✅ Tables dropped successfully${NC}"
echo ""

echo -e "${GREEN}Step 6: Verify rollback${NC}"
REMAINING_TABLES=$(psql "$STAGING_DB" -t -c "SELECT COUNT(*) FROM information_schema.tables WHERE table_schema = 'public' AND table_name IN ('skill_level_classifications', 'chapter_progress', 'chapter_metadata', 'chapter_recommendations');")
if [ "$REMAINING_TABLES" -eq 0 ]; then
  echo -e "  ${GREEN}✅ All Phase 4B tables removed${NC}"
else
  echo -e "  ${RED}❌ Warning: $REMAINING_TABLES tables still exist${NC}"
  psql "$STAGING_DB" -c "SELECT table_name FROM information_schema.tables WHERE table_schema = 'public' AND table_name IN ('skill_level_classifications', 'chapter_progress', 'chapter_metadata', 'chapter_recommendations');"
fi
echo ""

echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}✅ Rollback completed${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo -e "${BLUE}Rollback Summary:${NC}"
echo "  - Phase 4B tables removed from staging database"
echo "  - Schema backup: $BACKUP_FILE"
if [ -f "$DATA_BACKUP_FILE" ]; then
  echo "  - Data backup: $DATA_BACKUP_FILE"
fi
echo ""
echo -e "${YELLOW}To restore (if needed):${NC}"
if [ -f "$DATA_BACKUP_FILE" ]; then
  echo "  1. Reapply migration: ./backend/scripts/run_migration_staging.sh"
  echo "  2. Restore data: psql \$STAGING_DB -f $DATA_BACKUP_FILE"
else
  echo "  1. Reapply migration: ./backend/scripts/run_migration_staging.sh"
  echo "  2. Manually recreate any test data"
fi
echo ""
echo -e "${BLUE}Next Steps:${NC}"
echo "  - Fix migration issues identified during UAT"
echo "  - Reapply corrected migration when ready"
echo "  - Update Railway staging deployment"
echo ""
