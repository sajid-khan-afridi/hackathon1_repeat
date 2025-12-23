# Neon Staging Database Setup Guide

**Purpose**: Create and configure Neon PostgreSQL staging database for Phase 4B UAT
**Migration**: `008_personalization_schema.sql`
**Database**: Staging instance (isolated from production)
**Date**: 2025-12-23

---

## Prerequisites

- Neon account with admin access
- PostgreSQL client installed (`psql`) or use Neon SQL Editor
- Access to production database schema (for cloning)
- Migration file: `backend/app/migrations/008_personalization_schema.sql`

---

## Step 1: Create Staging Database

### Option A: Create New Database (Recommended)

1. **Login to Neon Console**
   - Go to [Neon Console](https://console.neon.tech)
   - Select your project or create new one

2. **Create Database**
   - Click **"Create Database"**
   - Database name: `hackathon1_staging`
   - Region: Same as production (e.g., `ap-southeast-1`)
   - Click **"Create"**

3. **Enable Connection Pooling**
   - In database settings
   - Enable **"Connection Pooling"**
   - Mode: **"Transaction"** (recommended for FastAPI)
   - Pool size: **5** (staging can use smaller pool)

4. **Get Connection String**
   - Click on database → **"Connection Details"**
   - Copy **"Pooler"** connection string (not direct)
   - Format: `postgresql://user:password@ep-staging-*.neon.tech/hackathon1_staging?sslmode=require`
   - Save for Railway environment variables

### Option B: Clone from Production

1. **Create Database Branch (Neon Feature)**
   - In Neon Console → **"Branches"**
   - Click **"New Branch"**
   - Name: `staging`
   - Branch from: `main` (production branch)
   - Click **"Create Branch"**

2. **Access Branched Database**
   - Neon creates complete copy of production data
   - Use branched connection string for staging
   - **Warning**: Contains production data - sanitize if needed

---

## Step 2: Initialize Database Schema

### If Using New Database (Option A)

You need to apply all migrations from Phase 1-4A first, then 4B.

#### 2.1 Apply Base Schema Migrations

**Locate Migration Files:**
```bash
cd backend/app/migrations
ls -la
```

Expected migrations:
```
001_initial_schema.sql          # Phase 1: Base tables
002_add_auth_tables.sql         # Phase 4A: Authentication
003_add_user_profiles.sql       # Phase 4A: User profiles
...
007_*.sql                       # Any previous migrations
008_personalization_schema.sql  # Phase 4B: This deployment
```

**Apply Migrations in Order:**

Using `psql`:
```bash
# Set connection string
export STAGING_DB="postgresql://user:password@ep-staging-*.neon.tech/hackathon1_staging?sslmode=require"

# Apply migrations sequentially
psql $STAGING_DB -f backend/app/migrations/001_initial_schema.sql
psql $STAGING_DB -f backend/app/migrations/002_add_auth_tables.sql
psql $STAGING_DB -f backend/app/migrations/003_add_user_profiles.sql
# ... apply all up to 007

# Verify schema
psql $STAGING_DB -c "\dt"
```

Using Neon SQL Editor:
1. Navigate to Neon Console → **"SQL Editor"**
2. Select `hackathon1_staging` database
3. Copy content of each migration file
4. Execute sequentially (001 → 007)

#### 2.2 Verify Base Schema

```sql
-- List all tables
\dt

-- Expected tables before Phase 4B:
-- users
-- sessions
-- user_profiles
-- chat_sessions
-- chat_messages
-- (other Phase 1-3 tables)

-- Verify user_profiles table exists (required for Phase 4B)
SELECT COUNT(*) FROM user_profiles;
```

### If Using Cloned Database (Option B)

Schema is already present. Verify:

```bash
export STAGING_DB="postgresql://user:password@ep-staging-*.neon.tech/hackathon1_staging?sslmode=require"

psql $STAGING_DB -c "\dt"
psql $STAGING_DB -c "SELECT COUNT(*) FROM users;"
```

---

## Step 3: Apply Phase 4B Migration

### Pre-Migration Validation

1. **Verify Prerequisites**

```sql
-- Connect to staging database
\c hackathon1_staging

-- Check users table exists
SELECT COUNT(*) FROM users;

-- Check user_profiles table exists (CRITICAL for Phase 4B)
SELECT COUNT(*) FROM user_profiles;

-- Verify no conflicting tables exist
SELECT table_name
FROM information_schema.tables
WHERE table_schema = 'public'
  AND table_name IN (
    'skill_level_classifications',
    'chapter_progress',
    'chapter_metadata',
    'chapter_recommendations'
  );
-- Should return 0 rows (tables don't exist yet)
```

2. **Backup Current Schema (Safety)**

```bash
# Create schema dump before migration
pg_dump $STAGING_DB --schema-only > staging_schema_pre_migration_$(date +%Y%m%d_%H%M%S).sql

# Store backup
mv staging_schema_pre_migration_*.sql backend/app/migrations/backups/
```

### Execute Migration

#### Method 1: Using psql (Recommended)

```bash
# Set connection string
export STAGING_DB="postgresql://user:password@ep-staging-*.neon.tech/hackathon1_staging?sslmode=require"

# Apply migration
psql $STAGING_DB -f backend/app/migrations/008_personalization_schema.sql

# Verify output - should show:
# CREATE TABLE (x4)
# CREATE INDEX (x10)
# INSERT 0 10 (chapter metadata)
```

#### Method 2: Using Neon SQL Editor

1. Navigate to Neon Console → **"SQL Editor"**
2. Select `hackathon1_staging` database
3. Open migration file: `backend/app/migrations/008_personalization_schema.sql`
4. Copy entire content
5. Paste into SQL Editor
6. Click **"Run Query"**
7. Verify success messages

#### Method 3: Using Migration Script (Automated)

See **Migration Execution Script** section below.

### Post-Migration Validation

1. **Verify Tables Created**

```sql
-- Check new tables exist
SELECT table_name,
       (SELECT COUNT(*) FROM information_schema.columns
        WHERE table_name = t.table_name) as column_count
FROM information_schema.tables t
WHERE table_schema = 'public'
  AND table_name IN (
    'skill_level_classifications',
    'chapter_progress',
    'chapter_metadata',
    'chapter_recommendations'
  )
ORDER BY table_name;

-- Expected output:
-- chapter_metadata                | 9
-- chapter_progress                | 8
-- skill_level_classifications     | 6
-- (chapter_recommendations not created in 008 migration)
```

2. **Verify Indexes Created**

```sql
-- List all indexes on new tables
SELECT tablename, indexname
FROM pg_indexes
WHERE tablename IN (
  'skill_level_classifications',
  'chapter_progress',
  'chapter_metadata'
)
ORDER BY tablename, indexname;

-- Expected: 10+ indexes across the 3 tables
```

3. **Verify Sample Data Inserted**

```sql
-- Check chapter metadata seeded
SELECT COUNT(*) FROM chapter_metadata;
-- Expected: 10 rows

-- View sample chapters
SELECT chapter_id, title, difficulty_level
FROM chapter_metadata
ORDER BY module_number, chapter_id
LIMIT 5;

-- Expected output:
-- module-1/ros-intro      | Introduction to ROS 2                | beginner
-- module-1/linux-basics   | Linux Basics for Robotics            | beginner
-- module-1/python-basics  | Python Programming for ROS           | beginner
-- module-1/gazebo-simulation | Gazebo Simulation Setup           | beginner
-- module-2/ros-publishers | Creating ROS 2 Publishers            | intermediate
```

4. **Verify Foreign Key Constraints**

```sql
-- Check foreign key relationships
SELECT
    tc.table_name,
    kcu.column_name,
    ccu.table_name AS foreign_table_name,
    ccu.column_name AS foreign_column_name
FROM information_schema.table_constraints AS tc
JOIN information_schema.key_column_usage AS kcu
  ON tc.constraint_name = kcu.constraint_name
JOIN information_schema.constraint_column_usage AS ccu
  ON ccu.constraint_name = tc.constraint_name
WHERE tc.constraint_type = 'FOREIGN KEY'
  AND tc.table_name IN (
    'skill_level_classifications',
    'chapter_progress'
  )
ORDER BY tc.table_name;

-- Expected:
-- skill_level_classifications | user_id | users | id
-- chapter_progress            | user_id | users | id
```

5. **Verify Check Constraints**

```sql
-- Test skill_level enum constraint
INSERT INTO skill_level_classifications (user_id, skill_level, based_on_profile)
SELECT id, 'expert', '{}'::jsonb FROM users LIMIT 1;
-- Should FAIL with: check constraint "skill_level_classifications_skill_level_check"

-- Test status enum constraint
INSERT INTO chapter_progress (user_id, chapter_id, status)
SELECT id, 'test-chapter', 'in_progress' FROM users LIMIT 1;
-- Should FAIL with: check constraint "chapter_progress_status_check"

-- Rollback test inserts
ROLLBACK;
```

---

## Step 4: Seed Test Data (Optional)

For UAT testing, seed some test users and progress data.

### Create Test Users

```sql
-- Insert test users (if not using OAuth in staging)
INSERT INTO users (email, username, created_at)
VALUES
  ('test.beginner@staging.test', 'test_beginner', NOW()),
  ('test.intermediate@staging.test', 'test_intermediate', NOW()),
  ('test.advanced@staging.test', 'test_advanced', NOW())
ON CONFLICT (email) DO NOTHING
RETURNING id, email;
```

### Create User Profiles

```sql
-- Create profiles for test users
WITH test_users AS (
  SELECT id, email FROM users WHERE email LIKE '%@staging.test'
)
INSERT INTO user_profiles (
  user_id,
  native_language,
  ros_experience_level,
  python_proficiency,
  hardware_access,
  learning_goals,
  created_at,
  updated_at
)
SELECT
  id,
  'en',
  CASE
    WHEN email LIKE '%beginner%' THEN 'none'
    WHEN email LIKE '%intermediate%' THEN 'beginner'
    WHEN email LIKE '%advanced%' THEN 'intermediate'
  END,
  CASE
    WHEN email LIKE '%beginner%' THEN 'beginner'
    WHEN email LIKE '%intermediate%' THEN 'intermediate'
    WHEN email LIKE '%advanced%' THEN 'advanced'
  END,
  email LIKE '%advanced%',
  ARRAY['practical', 'theoretical']::text[],
  NOW(),
  NOW()
FROM test_users
ON CONFLICT (user_id) DO NOTHING;
```

### Verify Test Data

```sql
-- Check test users and profiles
SELECT
  u.email,
  up.ros_experience_level,
  up.python_proficiency,
  up.hardware_access,
  up.learning_goals
FROM users u
JOIN user_profiles up ON u.id = up.user_id
WHERE u.email LIKE '%@staging.test';
```

---

## Migration Execution Script

Create automated migration script for reproducibility.

### Script: `backend/scripts/run_migration_staging.sh`

```bash
#!/bin/bash
# Migration execution script for Neon staging database
# Usage: ./backend/scripts/run_migration_staging.sh

set -e  # Exit on error

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
MIGRATION_FILE="backend/app/migrations/008_personalization_schema.sql"
BACKUP_DIR="backend/app/migrations/backups"

echo -e "${YELLOW}================================${NC}"
echo -e "${YELLOW}Phase 4B Migration - Staging${NC}"
echo -e "${YELLOW}================================${NC}"
echo ""

# Check if STAGING_DB is set
if [ -z "$STAGING_DB" ]; then
  echo -e "${RED}ERROR: STAGING_DB environment variable not set${NC}"
  echo "Please set it to your Neon staging connection string:"
  echo "  export STAGING_DB='postgresql://user:password@ep-staging-*.neon.tech/hackathon1_staging?sslmode=require'"
  exit 1
fi

# Verify migration file exists
if [ ! -f "$MIGRATION_FILE" ]; then
  echo -e "${RED}ERROR: Migration file not found: $MIGRATION_FILE${NC}"
  exit 1
fi

# Create backup directory
mkdir -p "$BACKUP_DIR"

echo -e "${GREEN}Step 1: Pre-migration backup${NC}"
BACKUP_FILE="$BACKUP_DIR/staging_schema_$(date +%Y%m%d_%H%M%S).sql"
pg_dump "$STAGING_DB" --schema-only > "$BACKUP_FILE"
echo -e "  ✅ Schema backed up to: $BACKUP_FILE"
echo ""

echo -e "${GREEN}Step 2: Validate prerequisites${NC}"
psql "$STAGING_DB" -c "SELECT COUNT(*) as user_count FROM users;" -t | grep -q "[0-9]" && echo "  ✅ users table exists" || { echo -e "  ${RED}❌ users table missing${NC}"; exit 1; }
psql "$STAGING_DB" -c "SELECT COUNT(*) as profile_count FROM user_profiles;" -t | grep -q "[0-9]" && echo "  ✅ user_profiles table exists" || { echo -e "  ${RED}❌ user_profiles table missing${NC}"; exit 1; }
echo ""

echo -e "${GREEN}Step 3: Apply migration${NC}"
psql "$STAGING_DB" -f "$MIGRATION_FILE"
echo ""

echo -e "${GREEN}Step 4: Verify migration${NC}"
TABLE_COUNT=$(psql "$STAGING_DB" -t -c "SELECT COUNT(*) FROM information_schema.tables WHERE table_schema = 'public' AND table_name IN ('skill_level_classifications', 'chapter_progress', 'chapter_metadata');")
if [ "$TABLE_COUNT" -eq 3 ]; then
  echo -e "  ✅ All tables created successfully"
else
  echo -e "  ${RED}❌ Expected 3 tables, found $TABLE_COUNT${NC}"
  exit 1
fi

METADATA_COUNT=$(psql "$STAGING_DB" -t -c "SELECT COUNT(*) FROM chapter_metadata;")
if [ "$METADATA_COUNT" -eq 10 ]; then
  echo -e "  ✅ Chapter metadata seeded (10 rows)"
else
  echo -e "  ${YELLOW}⚠️  Expected 10 chapter metadata rows, found $METADATA_COUNT${NC}"
fi
echo ""

echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}Migration completed successfully!${NC}"
echo -e "${GREEN}================================${NC}"
echo ""
echo "Next steps:"
echo "  1. Update Railway staging environment variables"
echo "  2. Deploy backend to Railway staging"
echo "  3. Run smoke tests"
```

Make script executable:
```bash
chmod +x backend/scripts/run_migration_staging.sh
```

Run migration:
```bash
export STAGING_DB="postgresql://user:password@ep-staging-*.neon.tech/hackathon1_staging?sslmode=require"
./backend/scripts/run_migration_staging.sh
```

---

## Migration Rollback Script

Create rollback script for emergency situations.

### Script: `backend/scripts/rollback_migration_staging.sh`

```bash
#!/bin/bash
# Rollback script for Phase 4B migration
# Usage: ./backend/scripts/rollback_migration_staging.sh

set -e

RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
NC='\033[0m'

echo -e "${RED}================================${NC}"
echo -e "${RED}Migration Rollback - Staging${NC}"
echo -e "${RED}================================${NC}"
echo ""
echo -e "${YELLOW}WARNING: This will delete all Phase 4B tables and data!${NC}"
echo ""
read -p "Are you sure you want to rollback? (type 'yes' to confirm): " CONFIRM

if [ "$CONFIRM" != "yes" ]; then
  echo "Rollback cancelled."
  exit 0
fi

if [ -z "$STAGING_DB" ]; then
  echo -e "${RED}ERROR: STAGING_DB environment variable not set${NC}"
  exit 1
fi

echo -e "${GREEN}Creating pre-rollback backup...${NC}"
mkdir -p backend/app/migrations/backups
pg_dump "$STAGING_DB" --schema-only > "backend/app/migrations/backups/pre_rollback_$(date +%Y%m%d_%H%M%S).sql"

echo -e "${GREEN}Dropping Phase 4B tables...${NC}"
psql "$STAGING_DB" <<SQL
-- Drop tables in reverse dependency order
DROP TABLE IF EXISTS chapter_recommendations CASCADE;
DROP TABLE IF EXISTS chapter_progress CASCADE;
DROP TABLE IF EXISTS chapter_metadata CASCADE;
DROP TABLE IF EXISTS skill_level_classifications CASCADE;
SQL

echo -e "${GREEN}Rollback completed.${NC}"
echo "Phase 4B tables removed from staging database."
```

Make script executable:
```bash
chmod +x backend/scripts/rollback_migration_staging.sh
```

---

## Troubleshooting

### Migration Fails: "relation users does not exist"

**Cause**: Base schema not initialized

**Fix**: Apply base migrations first (001-007) before 008.

### Migration Fails: "column user_id does not exist"

**Cause**: `user_profiles` table missing or schema mismatch

**Fix**:
```sql
-- Verify user_profiles schema
\d user_profiles

-- If missing, apply Phase 4A migrations first
```

### Duplicate Key Error on chapter_metadata Insert

**Cause**: Migration already partially applied

**Fix**:
```sql
-- Clear existing data and reapply
TRUNCATE chapter_metadata CASCADE;
-- Then rerun migration
```

### Connection Timeout

**Cause**: Invalid connection string or Neon database not accessible

**Fix**:
```bash
# Test connection
psql "$STAGING_DB" -c "SELECT version();"

# Verify sslmode parameter present
echo $STAGING_DB | grep "sslmode=require"
```

---

## Post-Migration Checklist

- ✅ Staging database created in Neon
- ✅ Connection pooling enabled
- ✅ Base schema migrations applied (001-007)
- ✅ Migration 008 applied successfully
- ✅ All 3 tables created (skill_level_classifications, chapter_progress, chapter_metadata)
- ✅ All indexes created (10+ indexes)
- ✅ Chapter metadata seeded (10 rows)
- ✅ Foreign key constraints verified
- ✅ Check constraints validated
- ✅ Test data seeded (optional)
- ✅ Schema backup created
- ✅ Connection string saved in Railway staging variables

Proceed to: **Railway Backend Deployment** (see `RAILWAY_STAGING_SETUP.md`)

---

**Document Version**: 1.0
**Last Updated**: 2025-12-23
**Maintained By**: DevOps Team
