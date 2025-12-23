# Phase 4B Staging Deployment Runbook

**Feature**: Personalization Engine
**Branch**: `1-personalization-engine`
**Migration**: `008_personalization_schema.sql`
**Target Environment**: Staging
**Date**: 2025-12-23

---

## Executive Summary

This runbook provides step-by-step instructions for deploying Phase 4B Personalization Engine to the staging environment for User Acceptance Testing (UAT).

**Deployment Components**:
- ✅ Backend API (FastAPI on Railway staging)
- ✅ Database Migration (Neon PostgreSQL staging)
- ✅ Frontend (Docusaurus on Vercel/Netlify staging)
- ✅ Personalization Features (skill classification, recommendations, progress tracking)

**Estimated Time**: 60-90 minutes

---

## Pre-Deployment Checklist

### Prerequisites Validation

- [ ] **Access & Permissions**
  - [ ] Railway admin access
  - [ ] Neon database admin access
  - [ ] Vercel/Netlify account (or GitHub Pages access)
  - [ ] GitHub repository write access

- [ ] **Environment Preparation**
  - [ ] Railway CLI installed (optional): `npm install -g @railway/cli`
  - [ ] PostgreSQL client installed: `psql --version`
  - [ ] Node.js 20+ installed: `node --version`
  - [ ] Git repository up to date: `git pull origin 1-personalization-engine`

- [ ] **Credentials Ready**
  - [ ] Neon staging database connection string
  - [ ] OpenAI API key
  - [ ] Qdrant URL and API key
  - [ ] OAuth credentials (Google/GitHub staging apps)
  - [ ] JWT RSA key pair (staging-specific)
  - [ ] CSRF secret (staging-specific)

- [ ] **Code Validation**
  - [ ] All 89 Phase 4B tasks completed
  - [ ] Local tests passing: `cd backend && pytest`
  - [ ] Frontend builds successfully: `npm run build`
  - [ ] Migration file validated: `backend/app/migrations/008_personalization_schema.sql`
  - [ ] No merge conflicts with main branch

---

## Deployment Workflow

### Phase 1: Database Setup (15-20 minutes)

#### Step 1.1: Create Neon Staging Database

**Reference**: See `NEON_STAGING_DATABASE_SETUP.md` for detailed instructions.

```bash
# Navigate to Neon Console
open https://console.neon.tech

# Create new database:
# - Name: hackathon1_staging
# - Region: ap-southeast-1 (same as production)
# - Enable connection pooling: Transaction mode

# Copy connection string (pooler endpoint)
export STAGING_DB="postgresql://user:password@ep-staging-*.neon.tech/hackathon1_staging?sslmode=require"
```

**Validation**:
```bash
# Test connection
psql "$STAGING_DB" -c "SELECT version();"
# Expected: PostgreSQL version output
```

#### Step 1.2: Apply Base Schema Migrations

If using new database (not cloned from production):

```bash
# Navigate to project
cd "D:\GitHub Connected\hackathon1_repeat"

# Apply base migrations (001-007)
# Note: Adjust migration files based on your actual migration history
psql "$STAGING_DB" -f backend/app/migrations/001_initial_schema.sql
psql "$STAGING_DB" -f backend/app/migrations/002_add_auth_tables.sql
# ... continue for all pre-Phase 4B migrations

# Verify base tables exist
psql "$STAGING_DB" -c "\dt"
# Expected: users, user_profiles, sessions, chat_sessions, chat_messages, etc.
```

#### Step 1.3: Apply Phase 4B Migration

```bash
# Run automated migration script
export STAGING_DB="postgresql://user:password@ep-staging-*.neon.tech/hackathon1_staging?sslmode=require"
./backend/scripts/run_migration_staging.sh
```

**Expected Output**:
```
✅ Database connection successful
✅ users table exists (X rows)
✅ user_profiles table exists (X rows)
✅ No conflicting tables found
✅ Schema backed up successfully
✅ Migration executed successfully
✅ All 3 tables created successfully
    - chapter_metadata
    - chapter_progress
    - skill_level_classifications
✅ Indexes created (10+ indexes)
✅ Chapter metadata seeded (10 chapters)
✅ Foreign key constraints created (2+ constraints)
✅ Check constraints created (3+ constraints)
```

**Validation**:
```bash
# Verify Phase 4B tables
psql "$STAGING_DB" -c "
SELECT table_name,
       (SELECT COUNT(*) FROM information_schema.columns WHERE table_name = t.table_name) as columns
FROM information_schema.tables t
WHERE table_schema = 'public'
  AND table_name IN ('skill_level_classifications', 'chapter_progress', 'chapter_metadata')
ORDER BY table_name;
"

# Verify sample data
psql "$STAGING_DB" -c "SELECT COUNT(*) FROM chapter_metadata;"
# Expected: 10
```

**Rollback Plan** (if migration fails):
```bash
./backend/scripts/rollback_migration_staging.sh
# Fix issues, then rerun migration
```

---

### Phase 2: Backend Deployment (20-30 minutes)

#### Step 2.1: Create Railway Staging Environment

**Reference**: See `RAILWAY_STAGING_SETUP.md` for detailed instructions.

**Via Railway Dashboard**:

1. Navigate to [Railway Dashboard](https://railway.app/dashboard)
2. Select project
3. Create new environment: **"staging"**
4. Switch to staging environment

#### Step 2.2: Configure Environment Variables

**Method 1: Bulk Import**

1. Open `backend/.env.staging` in text editor
2. Replace all placeholders with actual values:
   - `DATABASE_URL` → Neon staging connection string
   - `OPENAI_API_KEY` → Your OpenAI API key
   - `QDRANT_URL` → Qdrant cluster URL
   - `QDRANT_API_KEY` → Qdrant API key
   - `JWT_PRIVATE_KEY` → Staging RSA private key
   - `JWT_PUBLIC_KEY` → Staging RSA public key
   - `CSRF_SECRET_KEY` → Generate: `python -c "import secrets; print(secrets.token_urlsafe(32))"`
   - `GOOGLE_CLIENT_ID` → Staging Google OAuth app
   - `GOOGLE_CLIENT_SECRET` → Staging Google OAuth secret
   - `GOOGLE_REDIRECT_URI` → Update with Railway staging URL
   - `GITHUB_CLIENT_ID` → Staging GitHub OAuth app
   - `GITHUB_CLIENT_SECRET` → Staging GitHub OAuth secret
   - `GITHUB_REDIRECT_URI` → Update with Railway staging URL

3. In Railway dashboard → **Variables** → **Raw Editor**
4. Paste all variables from `.env.staging`
5. Click **Save**

**Method 2: Railway CLI**

```bash
railway environment staging
railway variables --set-from-file backend/.env.staging
```

#### Step 2.3: Configure Deployment Source

In Railway dashboard:

1. Select backend service
2. Go to **Settings** → **Source**
3. Configure:
   - **Branch**: `1-personalization-engine`
   - **Root Directory**: `backend` (if monorepo)
   - **Build Command**: (default)
   - **Start Command**: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

#### Step 2.4: Deploy Backend

**Manual Deployment** (recommended for first staging deploy):

1. In Railway dashboard → **Deployments** tab
2. Click **Deploy** button
3. Select branch: `1-personalization-engine`
4. Monitor build logs

**Expected Build Output**:
```
Installing dependencies...
Successfully installed fastapi uvicorn psycopg2 openai qdrant-client ...
Starting application...
INFO: Application startup complete.
INFO: Uvicorn running on http://0.0.0.0:8000
```

#### Step 2.5: Get Railway Staging URL

After deployment completes:

1. Railway dashboard → **Settings** → **Networking**
2. Click **Generate Domain** (if not already generated)
3. Copy URL: `https://hackathon1-staging.up.railway.app`
4. Save for frontend configuration

#### Step 2.6: Verify Backend Health

```bash
# Set staging URL
export STAGING_BACKEND_URL="https://hackathon1-staging.up.railway.app"

# Health check
curl $STAGING_BACKEND_URL/health
# Expected: {"status":"ok"}

# Detailed health check
curl $STAGING_BACKEND_URL/api/v1/health
# Expected: JSON with database, Qdrant, OpenAI status

# API docs (if enabled for staging)
open $STAGING_BACKEND_URL/api/docs
```

#### Step 2.7: Update OAuth Redirect URIs

**Google OAuth**:
1. Go to [Google Cloud Console](https://console.cloud.google.com/apis/credentials)
2. Select staging OAuth app
3. Update **Authorized redirect URIs**:
   - Add: `https://hackathon1-staging.up.railway.app/auth/google/callback`
4. Save

**GitHub OAuth**:
1. Go to [GitHub Settings](https://github.com/settings/developers)
2. Select staging OAuth app
3. Update **Authorization callback URL**:
   - Set: `https://hackathon1-staging.up.railway.app/auth/github/callback`
4. Update Application

**Update Railway Variables**:
```bash
# Update redirect URIs in Railway
railway variables set GOOGLE_REDIRECT_URI="https://hackathon1-staging.up.railway.app/auth/google/callback"
railway variables set GITHUB_REDIRECT_URI="https://hackathon1-staging.up.railway.app/auth/github/callback"

# Redeploy to apply changes
railway up
```

---

### Phase 3: Frontend Deployment (15-20 minutes)

#### Step 3.1: Choose Deployment Platform

**Recommended**: Vercel (easiest for staging)

**Alternative**: Netlify or GitHub Pages staging branch

#### Step 3.2: Deploy to Vercel

**Reference**: See `FRONTEND_STAGING_DEPLOYMENT.md` for detailed instructions.

```bash
# Install Vercel CLI
npm install -g vercel

# Login
vercel login

# Navigate to project root
cd "D:\GitHub Connected\hackathon1_repeat"

# Deploy
vercel

# When prompted:
# - Project name: hackathon1-staging
# - Settings: Use defaults
```

#### Step 3.3: Configure Environment Variables

**Via Vercel Dashboard**:

1. Go to [Vercel Dashboard](https://vercel.com/dashboard)
2. Select project: `hackathon1-staging`
3. Go to **Settings** → **Environment Variables**
4. Add:
   - Name: `API_URL`
   - Value: `https://hackathon1-staging.up.railway.app`
   - Environments: ✅ Preview, ✅ Development

**Via Vercel CLI**:
```bash
vercel env add API_URL
# Value: https://hackathon1-staging.up.railway.app
# Environments: Preview, Development
```

#### Step 3.4: Deploy from Feature Branch

```bash
# Ensure on correct branch
git checkout 1-personalization-engine

# Deploy to production (staging environment)
vercel --prod
```

**Expected Output**:
```
Deploying to production...
✅ Preview URL: https://hackathon1-staging-abc123.vercel.app
✅ Production URL: https://hackathon1-staging.vercel.app
```

#### Step 3.5: Update CORS in Railway Backend

Copy Vercel staging URL and update Railway:

```bash
# Update CORS_ORIGINS
railway variables set CORS_ORIGINS="https://hackathon1-staging-abc123.vercel.app,http://localhost:3000"

# Redeploy backend
railway up
```

**Or via Railway Dashboard**:
1. Railway → Staging environment → Variables
2. Update `CORS_ORIGINS`:
   ```
   https://hackathon1-staging-abc123.vercel.app,http://localhost:3000
   ```
3. Redeploy service

#### Step 3.6: Verify Frontend

```bash
# Open staging frontend
open https://hackathon1-staging-abc123.vercel.app

# Check in browser console:
# 1. API URL configured correctly
# 2. No CORS errors
# 3. Homepage loads
# 4. Chatbot page accessible
# 5. Profile page accessible
```

---

### Phase 4: Integration Validation (10-15 minutes)

#### Step 4.1: Verify End-to-End Connectivity

**Test 1: Frontend → Backend Health Check**
```bash
# In browser console (on staging frontend)
fetch('https://hackathon1-staging.up.railway.app/health')
  .then(r => r.json())
  .then(console.log);
// Expected: {status: "ok"}
```

**Test 2: OAuth Login Flow**
1. Navigate to staging frontend
2. Click "Login" (Google or GitHub)
3. Complete OAuth flow
4. Verify redirect back to staging frontend
5. Check user session created

**Test 3: Database Connectivity**
```bash
# Test user profile endpoint
curl -H "Authorization: Bearer $STAGING_TOKEN" \
  https://hackathon1-staging.up.railway.app/users/profile
# Expected: User profile JSON
```

#### Step 4.2: Verify Phase 4B Endpoints

```bash
# Set staging backend URL
export STAGING_API="https://hackathon1-staging.up.railway.app/api/v1"

# Test skill level endpoint
curl $STAGING_API/skill-level
# Expected: Skill level data or 401 if auth required

# Test recommendations endpoint
curl $STAGING_API/recommendations
# Expected: Recommendation list or 401

# Test progress tracking
curl -X POST $STAGING_API/progress/start \
  -H "Content-Type: application/json" \
  -d '{"chapter_id": "module-1/ros-intro"}'
# Expected: Progress record created or 401
```

---

### Phase 5: Smoke Testing (15-20 minutes)

#### Step 5.1: Run Automated Smoke Tests

**Reference**: See `SMOKE_TEST_CHECKLIST.md` for detailed test cases.

```bash
# Navigate to project
cd "D:\GitHub Connected\hackathon1_repeat"

# Run smoke test script
./backend/scripts/smoke_test_staging.sh
```

#### Step 5.2: Manual Smoke Tests

Execute critical user journeys:

1. **User Registration/Login**
   - [ ] Google OAuth login works
   - [ ] GitHub OAuth login works
   - [ ] User profile created
   - [ ] Session persists across page refresh

2. **Profile Management**
   - [ ] Navigate to `/profile` page
   - [ ] View user profile settings
   - [ ] Update profile (ROS experience, Python proficiency, etc.)
   - [ ] Changes saved to database

3. **Skill Classification**
   - [ ] Skill level badge displays on profile
   - [ ] Skill level calculated correctly (beginner/intermediate/advanced)
   - [ ] Recalculates after profile update

4. **Recommendations**
   - [ ] Recommended chapters display on homepage
   - [ ] Recommendations match skill level
   - [ ] Recommendations update after progress changes

5. **Progress Tracking**
   - [ ] Start chapter from recommendation
   - [ ] Progress recorded in database
   - [ ] Complete chapter action works
   - [ ] Bookmark chapter works
   - [ ] Progress displays on profile page

6. **RAG Chatbot with Personalization**
   - [ ] Navigate to `/chatbot` page
   - [ ] Ask question about ROS
   - [ ] Response personalized to skill level
   - [ ] Context includes user's current progress
   - [ ] Chat history persists

#### Step 5.3: Performance Validation

```bash
# Test API response times
time curl $STAGING_API/recommendations
# Expected: < 500ms

time curl $STAGING_API/skill-level
# Expected: < 200ms

# Test database query performance
psql "$STAGING_DB" -c "EXPLAIN ANALYZE SELECT * FROM chapter_progress WHERE user_id = 'some-uuid';"
# Verify index usage
```

---

## Post-Deployment Validation

### Validation Checklist

- [ ] **Infrastructure**
  - [ ] Railway staging environment active
  - [ ] Neon staging database accessible
  - [ ] Vercel staging deployment live
  - [ ] All services healthy (green status)

- [ ] **Backend API**
  - [ ] Health endpoint responding: `/health`
  - [ ] API docs accessible (if enabled): `/api/docs`
  - [ ] All Phase 4B endpoints deployed
  - [ ] Database connection successful
  - [ ] Qdrant integration working
  - [ ] OpenAI API integration functional
  - [ ] OAuth flows working (Google + GitHub)
  - [ ] CORS configured correctly
  - [ ] No critical errors in logs

- [ ] **Database**
  - [ ] Migration 008 applied successfully
  - [ ] All tables created (3 new tables)
  - [ ] Indexes created (10+ indexes)
  - [ ] Sample data seeded (10 chapters)
  - [ ] Foreign keys enforced
  - [ ] Check constraints validated

- [ ] **Frontend**
  - [ ] Staging URL accessible
  - [ ] API URL configured correctly
  - [ ] Homepage loads without errors
  - [ ] Chatbot page functional
  - [ ] Profile page functional
  - [ ] OAuth login working
  - [ ] No console errors
  - [ ] No CORS errors
  - [ ] Responsive design working
  - [ ] Dark mode toggle working

- [ ] **Personalization Features**
  - [ ] Skill classification endpoint working
  - [ ] Skill level badge displays
  - [ ] Recommendations endpoint working
  - [ ] Recommended chapters display
  - [ ] Progress tracking endpoints working
  - [ ] Start chapter action works
  - [ ] Complete chapter action works
  - [ ] Bookmark chapter works
  - [ ] Progress displays on profile
  - [ ] RAG responses personalized

---

## Rollback Procedures

### Emergency Rollback (If Critical Issues Found)

#### Backend Rollback

**Via Railway Dashboard**:
1. Go to **Deployments** tab
2. Find last successful deployment before Phase 4B
3. Click **Redeploy** on that deployment

**Via Railway CLI**:
```bash
railway rollback
```

#### Database Rollback

```bash
# Run rollback script
export STAGING_DB="postgresql://user:password@ep-staging-*.neon.tech/hackathon1_staging?sslmode=require"
./backend/scripts/rollback_migration_staging.sh

# Type 'ROLLBACK' to confirm
```

**Manual Rollback**:
```sql
-- Connect to staging database
psql "$STAGING_DB"

-- Drop Phase 4B tables
DROP TABLE IF EXISTS chapter_recommendations CASCADE;
DROP TABLE IF EXISTS chapter_progress CASCADE;
DROP TABLE IF EXISTS chapter_metadata CASCADE;
DROP TABLE IF EXISTS skill_level_classifications CASCADE;
```

#### Frontend Rollback

**Vercel**: Rollback to previous deployment
1. Vercel Dashboard → **Deployments**
2. Select previous successful deployment
3. Click **Promote to Production**

---

## UAT Handoff

### Staging Environment URLs

Document all staging URLs for UAT team:

```
Frontend: https://hackathon1-staging-abc123.vercel.app
Backend API: https://hackathon1-staging.up.railway.app
API Docs: https://hackathon1-staging.up.railway.app/api/docs
Database: [Provided separately to DBAs only]
```

### UAT Test Plan

**Reference**: See `UAT_TEST_PLAN.md` for comprehensive test cases.

**Test Accounts**:
Create test users with different profiles for UAT:

```sql
-- Test user credentials (shared with UAT team)
-- Beginner user: test.beginner@staging.test
-- Intermediate user: test.intermediate@staging.test
-- Advanced user: test.advanced@staging.test
```

### UAT Timeline

- **UAT Duration**: 2-3 days
- **UAT Team**: Product Owner, QA Lead, 2-3 test users
- **Success Criteria**: All critical user journeys pass without major issues
- **Sign-off Required**: Product Owner approval before production deployment

---

## Production Deployment (After UAT Approval)

### Pre-Production Checklist

- [ ] UAT completed successfully
- [ ] All critical bugs fixed
- [ ] Performance validated
- [ ] Security review passed
- [ ] Product Owner sign-off obtained
- [ ] Production environment variables prepared
- [ ] Production OAuth apps configured
- [ ] Production database backup created

### Production Deployment Steps

1. **Merge to Main**
   ```bash
   git checkout main
   git merge 1-personalization-engine
   git push origin main
   ```

2. **Apply Migration to Production Database**
   ```bash
   export PRODUCTION_DB="postgresql://user:password@ep-production-*.neon.tech/neondb?sslmode=require"

   # Backup production database
   pg_dump "$PRODUCTION_DB" --schema-only > production_backup_$(date +%Y%m%d).sql

   # Apply migration (carefully!)
   psql "$PRODUCTION_DB" -f backend/app/migrations/008_personalization_schema.sql
   ```

3. **Deploy Backend to Production Railway**
   - Railway automatically deploys from `main` branch
   - Monitor deployment in Railway dashboard

4. **Deploy Frontend to Production GitHub Pages**
   - GitHub Actions automatically deploys from `main` branch
   - Workflow: `.github/workflows/deploy.yml`

5. **Verify Production**
   - Run smoke tests on production
   - Monitor error rates and performance
   - Check user reports

---

## Monitoring & Observability

### Railway Logs

Monitor staging logs during UAT:

```bash
# Via Railway CLI
railway logs --environment staging

# Or via dashboard: Railway → Logs tab
```

### Database Monitoring

```bash
# Monitor active connections
psql "$STAGING_DB" -c "SELECT COUNT(*) FROM pg_stat_activity;"

# Monitor slow queries
psql "$STAGING_DB" -c "SELECT query, mean_exec_time FROM pg_stat_statements ORDER BY mean_exec_time DESC LIMIT 10;"
```

### Frontend Performance

Use Vercel Analytics:
1. Vercel Dashboard → Analytics
2. Monitor Core Web Vitals
3. Check API request performance

---

## Troubleshooting Guide

### Issue: Migration Fails

**Symptom**: `run_migration_staging.sh` exits with errors

**Common Causes**:
- Base tables missing (users, user_profiles)
- Conflicting table names already exist
- Invalid SQL syntax

**Resolution**:
```bash
# Check prerequisite tables
psql "$STAGING_DB" -c "\dt"

# If tables missing, apply base migrations first
psql "$STAGING_DB" -f backend/app/migrations/001_initial_schema.sql
# ... continue with 002-007

# If tables conflict, drop and reapply
./backend/scripts/rollback_migration_staging.sh
./backend/scripts/run_migration_staging.sh
```

### Issue: Backend Deployment Fails

**Symptom**: Railway deployment stuck or fails

**Common Causes**:
- Missing dependencies in `requirements.txt`
- Environment variables not set
- Port binding issues

**Resolution**:
1. Check Railway build logs for specific error
2. Verify all environment variables set in Railway
3. Test locally: `cd backend && uvicorn app.main:app`

### Issue: CORS Errors

**Symptom**: Browser console shows CORS policy errors

**Resolution**:
```bash
# Update Railway CORS_ORIGINS
railway variables set CORS_ORIGINS="https://hackathon1-staging-abc123.vercel.app,http://localhost:3000"

# Redeploy backend
railway up

# Clear browser cache and retry
```

### Issue: OAuth Login Fails

**Symptom**: Redirect URI mismatch error

**Resolution**:
1. Verify Railway staging URL matches OAuth app redirect URI
2. Update Google/GitHub OAuth apps with correct callback URL
3. Update Railway environment variables:
   ```
   GOOGLE_REDIRECT_URI=https://hackathon1-staging.up.railway.app/auth/google/callback
   GITHUB_REDIRECT_URI=https://hackathon1-staging.up.railway.app/auth/github/callback
   ```

---

## Success Criteria

Deployment is considered successful when:

✅ **Infrastructure**: All services deployed and healthy
✅ **Functionality**: All Phase 4B features working in staging
✅ **Performance**: API response times < 500ms, page load < 2s
✅ **Reliability**: No critical errors in logs for 24 hours
✅ **Security**: OAuth flows secure, secrets not exposed
✅ **UAT Ready**: Test plan executed, staging URLs documented

---

## Contact & Escalation

**Deployment Issues**: DevOps Team
**Database Issues**: Database Administrator
**OAuth/Security Issues**: Security Lead
**UAT Questions**: Product Owner

---

**Runbook Version**: 1.0
**Last Updated**: 2025-12-23
**Next Review**: After UAT completion
**Maintained By**: DevOps Team
