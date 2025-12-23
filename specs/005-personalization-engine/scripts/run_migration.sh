#!/bin/bash
# =============================================================================
# Phase 4B Database Migration Script
# =============================================================================
# Purpose: Execute migration 008_personalization_schema.sql safely
# Usage: bash specs/1-personalization-engine/scripts/run_migration.sh [staging|production]
# Prerequisites: psql installed, DATABASE_URL set
# =============================================================================

set -e  # Exit on error

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
ENVIRONMENT="${1:-staging}"
MIGRATION_FILE="backend/app/migrations/008_personalization_schema.sql"
ROLLBACK_FILE="specs/1-personalization-engine/scripts/rollback_migration.sql"

echo "=========================================="
echo "Phase 4B Database Migration"
echo "=========================================="
echo -e "Environment: ${BLUE}$ENVIRONMENT${NC}"
echo ""

# Validate environment
if [[ "$ENVIRONMENT" != "staging" && "$ENVIRONMENT" != "production" ]]; then
    echo -e "${RED}Error: Invalid environment '$ENVIRONMENT'${NC}"
    echo "Usage: $0 [staging|production]"
    exit 1
fi

# Check if DATABASE_URL is set
if [ -z "$DATABASE_URL" ]; then
    echo -e "${RED}Error: DATABASE_URL environment variable not set${NC}"
    echo ""
    echo "Set DATABASE_URL to your Neon connection string:"
    echo "  export DATABASE_URL='postgresql://user:password@host:5432/db?sslmode=require'"
    exit 1
fi

# Validate migration file exists
if [ ! -f "$MIGRATION_FILE" ]; then
    echo -e "${RED}Error: Migration file not found: $MIGRATION_FILE${NC}"
    exit 1
fi

# Display database info (mask password)
DB_HOST=$(echo "$DATABASE_URL" | sed -E 's|.*@([^:/]+).*|\1|')
DB_NAME=$(echo "$DATABASE_URL" | sed -E 's|.*/([^?]+).*|\1|')

echo "Database Connection:"
echo "  Host: $DB_HOST"
echo "  Database: $DB_NAME"
echo ""

# Test database connection
echo -n "Testing database connection ... "
if psql "$DATABASE_URL" -c "SELECT version();" > /dev/null 2>&1; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}FAILED${NC}"
    echo "Could not connect to database. Check DATABASE_URL and network connectivity."
    exit 1
fi
echo ""

# Check if migration was already run
echo "Checking if migration was already executed ..."
TABLE_EXISTS=$(psql "$DATABASE_URL" -tAc "SELECT EXISTS (SELECT FROM information_schema.tables WHERE table_name='chapter_metadata');")

if [ "$TABLE_EXISTS" == "t" ]; then
    echo -e "${YELLOW}Warning: Table 'chapter_metadata' already exists.${NC}"
    echo "Migration may have already been executed."
    echo ""
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Migration cancelled."
        exit 0
    fi
fi

# Production safety check
if [ "$ENVIRONMENT" == "production" ]; then
    echo -e "${RED}╔═══════════════════════════════════════╗${NC}"
    echo -e "${RED}║  WARNING: PRODUCTION DEPLOYMENT       ║${NC}"
    echo -e "${RED}╚═══════════════════════════════════════╝${NC}"
    echo ""
    echo "This will modify the PRODUCTION database."
    echo "Ensure you have:"
    echo "  ✓ Completed UAT on staging successfully"
    echo "  ✓ Received stakeholder sign-off"
    echo "  ✓ Created database backup/snapshot"
    echo "  ✓ Notified team of deployment window"
    echo ""
    read -p "Type 'DEPLOY TO PRODUCTION' to confirm: " confirm

    if [ "$confirm" != "DEPLOY TO PRODUCTION" ]; then
        echo "Production deployment cancelled."
        exit 0
    fi
fi

# Create backup (Neon-specific)
echo ""
echo "=========================================="
echo "Pre-Migration Backup"
echo "=========================================="
echo "Recommended: Create a Neon snapshot before migration"
echo "  1. Go to https://console.neon.tech"
echo "  2. Select your database"
echo "  3. Go to 'Backups' → 'Create Snapshot'"
echo "  4. Name: pre-phase4b-migration-$(date +%Y%m%d)"
echo ""
read -p "Have you created a backup snapshot? (y/N): " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${YELLOW}Warning: Proceeding without backup snapshot.${NC}"
    read -p "Continue? (y/N): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Migration cancelled."
        exit 0
    fi
fi

# Execute migration
echo ""
echo "=========================================="
echo "Executing Migration"
echo "=========================================="
echo "Migration file: $MIGRATION_FILE"
echo ""

echo "Running SQL migration ..."
if psql "$DATABASE_URL" -f "$MIGRATION_FILE" > migration_output.log 2>&1; then
    echo -e "${GREEN}✓ Migration executed successfully${NC}"
else
    echo -e "${RED}✗ Migration failed${NC}"
    echo ""
    echo "Error log:"
    cat migration_output.log
    echo ""
    echo "Migration has been rolled back by PostgreSQL (transaction)."
    rm -f migration_output.log
    exit 1
fi

# Verify migration
echo ""
echo "=========================================="
echo "Verifying Migration"
echo "=========================================="

# Check tables created
echo -n "Checking tables ... "
TABLES_CREATED=$(psql "$DATABASE_URL" -tAc "
    SELECT COUNT(*) FROM information_schema.tables
    WHERE table_name IN ('skill_level_classifications', 'chapter_progress', 'chapter_metadata');
")

if [ "$TABLES_CREATED" -eq "3" ]; then
    echo -e "${GREEN}OK${NC} (3/3 tables created)"
else
    echo -e "${RED}FAILED${NC} (Expected 3 tables, found $TABLES_CREATED)"
    exit 1
fi

# Check indexes created
echo -n "Checking indexes ... "
INDEXES_CREATED=$(psql "$DATABASE_URL" -tAc "
    SELECT COUNT(*) FROM pg_indexes
    WHERE indexname LIKE 'idx_skill_level_%'
       OR indexname LIKE 'idx_chapter_%';
")

if [ "$INDEXES_CREATED" -ge "8" ]; then
    echo -e "${GREEN}OK${NC} ($INDEXES_CREATED indexes created)"
else
    echo -e "${YELLOW}WARNING${NC} (Expected 8+ indexes, found $INDEXES_CREATED)"
fi

# Check sample data inserted
echo -n "Checking chapter metadata ... "
CHAPTER_COUNT=$(psql "$DATABASE_URL" -tAc "SELECT COUNT(*) FROM chapter_metadata;")

if [ "$CHAPTER_COUNT" -ge "10" ]; then
    echo -e "${GREEN}OK${NC} ($CHAPTER_COUNT chapters inserted)"
else
    echo -e "${YELLOW}WARNING${NC} (Expected 10 chapters, found $CHAPTER_COUNT)"
fi

# Display created tables
echo ""
echo "Created tables:"
psql "$DATABASE_URL" -c "
    SELECT
        table_name,
        (SELECT COUNT(*) FROM information_schema.columns WHERE table_name = t.table_name) as column_count
    FROM information_schema.tables t
    WHERE table_name IN ('skill_level_classifications', 'chapter_progress', 'chapter_metadata')
    ORDER BY table_name;
"

echo ""
echo "Sample chapter metadata:"
psql "$DATABASE_URL" -c "
    SELECT chapter_id, title, difficulty_level
    FROM chapter_metadata
    LIMIT 5;
"

# Clean up
rm -f migration_output.log

# Success
echo ""
echo "=========================================="
echo -e "${GREEN}Migration Completed Successfully!${NC}"
echo "=========================================="
echo ""
echo "Next steps:"
if [ "$ENVIRONMENT" == "staging" ]; then
    echo "  1. Deploy backend to Railway staging"
    echo "  2. Run smoke tests: bash specs/1-personalization-engine/scripts/smoke_test.sh"
    echo "  3. Proceed with UAT testing"
else
    echo "  1. Deploy backend to Railway production"
    echo "  2. Run smoke tests on production"
    echo "  3. Monitor application logs for 24 hours"
    echo "  4. Notify stakeholders of successful deployment"
fi
echo ""

# Generate rollback script
echo "Generating rollback script ..."
cat > "$ROLLBACK_FILE" << 'EOF'
-- =============================================================================
-- ROLLBACK SCRIPT - Phase 4B Migration
-- =============================================================================
-- WARNING: This will DELETE all Phase 4B tables and data
-- Use only in emergency rollback scenarios
-- =============================================================================

BEGIN;

-- Drop tables in reverse dependency order
DROP TABLE IF EXISTS chapter_recommendations CASCADE;
DROP TABLE IF EXISTS skill_level_classifications CASCADE;
DROP TABLE IF EXISTS chapter_progress CASCADE;
DROP TABLE IF EXISTS chapter_metadata CASCADE;

COMMIT;

-- Verify rollback
SELECT 'Rollback completed. Phase 4B tables removed.' AS status;
EOF

echo -e "${GREEN}✓${NC} Rollback script created: $ROLLBACK_FILE"
echo ""
echo "To rollback this migration (EMERGENCY ONLY):"
echo "  psql \"\$DATABASE_URL\" -f $ROLLBACK_FILE"
echo ""
