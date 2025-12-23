# Phase 4B Staging Deployment Runbook

**Feature**: Personalization Engine
**Branch**: `1-personalization-engine`
**Target Environment**: Staging
**Deployment Date**: 2025-12-23
**Estimated Time**: 45-60 minutes

---

## Table of Contents

1. [Pre-Deployment Checklist](#pre-deployment-checklist)
2. [Phase 1: Infrastructure Setup](#phase-1-infrastructure-setup)
3. [Phase 2: Database Migration](#phase-2-database-migration)
4. [Phase 3: Backend Deployment](#phase-3-backend-deployment)
5. [Phase 4: Frontend Deployment](#phase-4-frontend-deployment)
6. [Phase 5: Smoke Tests](#phase-5-smoke-tests)
7. [Phase 6: UAT Preparation](#phase-6-uat-preparation)
8. [Rollback Procedure](#rollback-procedure)
9. [Production Deployment](#production-deployment)

---

## Pre-Deployment Checklist

Before starting the deployment, ensure you have:

- [ ] **Access Credentials**
  - [ ] Railway account with staging environment access
  - [ ] Neon console access for database management
  - [ ] GitHub repository write access
  - [ ] Google Cloud Console (for OAuth staging app)

- [ ] **Environment Setup**
  - [ ] `backend/.env.staging` file configured (see checklist in file)
  - [ ] Staging RSA keys generated for JWT
  - [ ] Staging OAuth apps created (Google & GitHub)

- [ ] **Code Readiness**
  - [ ] All 89 tasks (T001-T089) completed and marked ✅
  - [ ] All tests passing locally: `pytest backend/tests/ -v`
  - [ ] Frontend builds successfully: `npm run build`
  - [ ] Branch `1-personalization-engine` up to date with `main`

- [ ] **Communication**
  - [ ] Stakeholders notified of staging deployment window
  - [ ] UAT testers identified and available
  - [ ] Rollback team on standby

---

## Phase 1: Infrastructure Setup

### 1.1 Create Neon Staging Database

**Time**: 5 minutes

```bash
# 1. Go to Neon Console
open https://console.neon.tech

# 2. Create new project or use existing project
# Click "Create Project" or select existing project

# 3. Create staging database
# - Project name: hackathon1-staging (or clone from production)
# - Region: Same as production (e.g., ap-southeast-1)
# - Click "Create Database" → Name: hackathon1_staging

# 4. Copy connection string
# - Click "Connection String" → Select "Pooled connection"
# - Format: postgresql://user:password@ep-xxx.neon.tech/hackathon1_staging?sslmode=require
# - Save this for .env.staging configuration
```

**Verification**:
```bash
# Test connection
psql "postgresql://user:password@ep-xxx.neon.tech/hackathon1_staging?sslmode=require" -c "SELECT version();"
```

### 1.2 Configure Railway Staging Environment

**Time**: 10 minutes

```bash
# 1. Go to Railway Dashboard
open https://railway.app/dashboard

# 2. Select your project: hackathon1_repeat

# 3. Create staging environment
# - Click "Environments" → "New Environment"
# - Name: staging
# - Branch: 1-personalization-engine

# 4. Configure environment variables from .env.staging
# Click "Variables" → "Raw Editor" → Paste from backend/.env.staging

# CRITICAL VARIABLES TO SET:
# - DATABASE_URL (from Neon staging instance)
# - OPENAI_API_KEY
# - QDRANT_URL, QDRANT_API_KEY
# - JWT_PRIVATE_KEY, JWT_PUBLIC_KEY
# - GOOGLE_CLIENT_ID, GOOGLE_CLIENT_SECRET
# - GITHUB_CLIENT_ID, GITHUB_CLIENT_SECRET
# - CORS_ORIGINS (staging frontend URL)
```

**Verification**:
```bash
# Check Railway environment is created
railway status --environment staging
```

### 1.3 Create Staging OAuth Apps

#### Google OAuth Staging App

**Time**: 5 minutes

```bash
# 1. Go to Google Cloud Console
open https://console.cloud.google.com/apis/credentials

# 2. Create OAuth 2.0 Client ID
# - Application type: Web application
# - Name: RAG Chatbot Staging
# - Authorized redirect URIs:
#   - https://hackathon1-staging.up.railway.app/auth/google/callback
#   - http://localhost:8000/auth/google/callback

# 3. Copy Client ID and Secret to Railway staging environment variables
```

#### GitHub OAuth Staging App

**Time**: 5 minutes

```bash
# 1. Go to GitHub Developer Settings
open https://github.com/settings/developers

# 2. Click "New OAuth App"
# - Application name: RAG Chatbot Staging
# - Homepage URL: https://staging-hackathon1.vercel.app
# - Authorization callback URL: https://hackathon1-staging.up.railway.app/auth/github/callback

# 3. Copy Client ID and Secret to Railway staging environment variables
```

---

## Phase 2: Database Migration

### 2.1 Run Migration 008

**Time**: 3 minutes

```bash
# Navigate to project root
cd hackathon1_repeat

# Set staging database URL
export DATABASE_URL="postgresql://user:password@ep-xxx.neon.tech/hackathon1_staging?sslmode=require"

# Run migration
psql "$DATABASE_URL" -f backend/app/migrations/008_personalization_schema.sql

# Expected output:
# CREATE TABLE
# CREATE INDEX
# ... (repeats for all tables)
# INSERT 0 10 (chapter metadata inserted)
```

### 2.2 Verify Migration

```bash
# Check tables were created
psql "$DATABASE_URL" -c "\dt"

# Expected tables:
# - skill_level_classifications
# - chapter_progress
# - chapter_metadata
# - users (from Phase 4A)
# - user_profiles (from Phase 4A)

# Verify chapter metadata
psql "$DATABASE_URL" -c "SELECT chapter_id, title, difficulty_level FROM chapter_metadata LIMIT 5;"

# Expected: 10 rows of sample chapters
```

### 2.3 Create Rollback Snapshot (Optional)

```bash
# In Neon Console, create database snapshot
# Navigate to: Database → Backups → Create Snapshot
# Name: "pre-phase4b-migration"
```

---

## Phase 3: Backend Deployment

### 3.1 Deploy to Railway Staging

**Time**: 10 minutes

```bash
# Method 1: Using Railway CLI
railway login
railway link  # Select your project
railway environment staging
railway up --detach

# Method 2: Using GitHub Push
git push origin 1-personalization-engine

# Railway will auto-deploy if GitHub integration is enabled
# Check deployment status:
railway status

# Wait for deployment to complete (5-8 minutes)
```

### 3.2 Verify Backend Health

```bash
# Get staging URL from Railway
export STAGING_BACKEND_URL=$(railway variables get RAILWAY_PUBLIC_DOMAIN --environment staging)

# Check health endpoint
curl https://$STAGING_BACKEND_URL/health

# Expected response:
# {"status": "healthy", "environment": "staging", "version": "phase-4b-personalization"}

# Check API docs
open https://$STAGING_BACKEND_URL/docs
```

### 3.3 Verify Phase 4B Endpoints

```bash
# Check new personalization endpoints exist
curl https://$STAGING_BACKEND_URL/docs | grep -o "api/v1/skill-level"
curl https://$STAGING_BACKEND_URL/docs | grep -o "api/v1/recommendations"
curl https://$STAGING_BACKEND_URL/docs | grep -o "api/v1/progress"

# All should return matches
```

---

## Phase 4: Frontend Deployment

### 4.1 Configure Staging Build

**Time**: 5 minutes

```bash
# Create .env.production.local for staging build
cat > .env.production.local << EOF
REACT_APP_API_URL=https://hackathon1-staging.up.railway.app
REACT_APP_ENVIRONMENT=staging
EOF

# Alternatively, set in docusaurus.config.ts:
# customFields: {
#   apiUrl: process.env.REACT_APP_API_URL || 'https://hackathon1-staging.up.railway.app',
# }
```

### 4.2 Build and Deploy Frontend

**Option A: Vercel/Netlify Staging (Recommended)**

```bash
# Deploy to Vercel staging
npm install -g vercel
vercel --prod --scope your-team

# Or deploy to Netlify staging
npm install -g netlify-cli
netlify deploy --prod --dir=build

# Set environment variable:
# REACT_APP_API_URL=https://hackathon1-staging.up.railway.app
```

**Option B: GitHub Pages Staging Branch**

```bash
# Create staging branch for GitHub Pages
git checkout -b gh-pages-staging
npm run build

# Deploy to staging branch
npx gh-pages -d build -b gh-pages-staging

# Update GitHub Pages settings:
# - Repository Settings → Pages
# - Source: gh-pages-staging branch
# - Save

# Staging URL will be: https://sajid-khan-afridi.github.io/hackathon1_repeat/
```

### 4.3 Verify Frontend

```bash
# Open staging frontend
open https://staging-hackathon1.vercel.app
# OR
open https://sajid-khan-afridi.github.io/hackathon1_repeat/

# Check browser console for API connection
# Should see successful API calls to staging backend
```

---

## Phase 5: Smoke Tests

### 5.1 Run Automated Smoke Tests

**Time**: 10 minutes

```bash
# Run smoke test script
cd hackathon1_repeat
bash specs/1-personalization-engine/scripts/smoke_test.sh

# Or run manually:
export STAGING_API="https://hackathon1-staging.up.railway.app/api/v1"

# Test 1: Health check
curl $STAGING_API/../health

# Test 2: Unauthenticated endpoints should work
curl $STAGING_API/chat/query -X POST -H "Content-Type: application/json" \
  -d '{"query": "What is ROS?", "session_id": "test-session"}'

# Expected: 200 OK with personalized response
```

### 5.2 Manual Smoke Tests

Use the smoke test checklist in `specs/1-personalization-engine/SMOKE_TEST_CHECKLIST.md`

**Critical Tests**:
- [ ] User can sign up with Google OAuth
- [ ] User can complete profile settings
- [ ] Skill level classification works (GET /api/v1/skill-level)
- [ ] Recommendations load (GET /api/v1/recommendations)
- [ ] Progress tracking works (POST /api/v1/progress/start)
- [ ] RAG chatbot provides personalized answers
- [ ] Content filtering based on user profile works

---

## Phase 6: UAT Preparation

### 6.1 Document Staging URLs

```markdown
## Staging Environment URLs

- **Frontend**: https://staging-hackathon1.vercel.app
- **Backend API**: https://hackathon1-staging.up.railway.app
- **API Docs**: https://hackathon1-staging.up.railway.app/docs
- **Database**: Neon staging instance (ep-xxx.neon.tech)

## Test Accounts

- **Google OAuth**: Use personal Google accounts for testing
- **GitHub OAuth**: Use personal GitHub accounts for testing
```

### 6.2 Share UAT Test Plan

Send UAT testers the following:
- Staging frontend URL
- UAT test plan: `specs/1-personalization-engine/UAT_TEST_PLAN.md`
- Test scenario scripts
- Feedback form or issue tracker link

### 6.3 Monitor During UAT

```bash
# Watch Railway logs in real-time
railway logs --environment staging --follow

# Monitor error rates
railway metrics --environment staging

# Check Neon database performance
# Neon Console → Metrics → Query Performance
```

---

## Rollback Procedure

### If Critical Issues Found During UAT

**Time**: 5 minutes

### Option 1: Rollback Railway Deployment

```bash
# Railway: Redeploy previous version
railway environment staging
railway redeploy <previous-deployment-id>

# Check deployment history:
railway deployments --environment staging
```

### Option 2: Rollback Database Migration

```bash
# Restore from Neon snapshot
# 1. Go to Neon Console → Database → Backups
# 2. Select "pre-phase4b-migration" snapshot
# 3. Click "Restore"

# Or run manual rollback SQL
psql "$DATABASE_URL" << EOF
DROP TABLE IF EXISTS chapter_recommendations;
DROP TABLE IF EXISTS skill_level_classifications;
DROP TABLE IF EXISTS chapter_progress;
DROP TABLE IF EXISTS chapter_metadata;
EOF
```

### Option 3: Full Environment Reset

```bash
# Delete staging environment and recreate
railway environment delete staging
# Then re-run Phase 1-4 from this runbook
```

---

## Production Deployment

**ONLY PROCEED AFTER**:
- [ ] UAT completed successfully with no critical issues
- [ ] All smoke tests passing
- [ ] Stakeholder sign-off received
- [ ] Production deployment window confirmed

### Production Deployment Steps

1. **Merge to Main**
   ```bash
   git checkout main
   git merge 1-personalization-engine
   git push origin main
   ```

2. **Run Migration on Production Database**
   ```bash
   export DATABASE_URL="<production-neon-url>"
   psql "$DATABASE_URL" -f backend/app/migrations/008_personalization_schema.sql
   ```

3. **Deploy Backend to Production Railway**
   ```bash
   railway environment production
   railway up --detach
   ```

4. **Deploy Frontend to Production**
   ```bash
   npm run build
   npm run deploy  # GitHub Pages deployment
   ```

5. **Verify Production**
   ```bash
   # Run smoke tests against production
   export STAGING_API="https://hackathon1repeat-production.up.railway.app/api/v1"
   bash specs/1-personalization-engine/scripts/smoke_test.sh
   ```

6. **Monitor Production for 24 Hours**
   - Watch error logs
   - Check API response times
   - Monitor database performance
   - Validate user feedback

---

## Deployment Contacts

- **DevOps Lead**: [Your Name]
- **Backend Engineer**: [Your Name]
- **Frontend Engineer**: [Your Name]
- **QA Lead**: [UAT Tester Name]
- **Stakeholder**: [Product Owner]

---

## Deployment Checklist Summary

**Pre-Deployment**:
- [ ] All tasks T001-T089 complete
- [ ] Tests passing
- [ ] .env.staging configured

**Phase 1: Infrastructure**:
- [ ] Neon staging database created
- [ ] Railway staging environment created
- [ ] OAuth staging apps created

**Phase 2: Database**:
- [ ] Migration 008 executed
- [ ] Migration verified
- [ ] Rollback snapshot created

**Phase 3: Backend**:
- [ ] Backend deployed to Railway staging
- [ ] Health check passing
- [ ] API docs accessible

**Phase 4: Frontend**:
- [ ] Frontend built with staging API URL
- [ ] Frontend deployed to staging
- [ ] Frontend loads successfully

**Phase 5: Smoke Tests**:
- [ ] Automated smoke tests pass
- [ ] Manual smoke tests complete

**Phase 6: UAT**:
- [ ] UAT testers notified
- [ ] UAT test plan distributed
- [ ] Monitoring enabled

**Post-UAT**:
- [ ] UAT feedback collected
- [ ] Critical issues resolved OR rollback executed
- [ ] Stakeholder sign-off received
- [ ] Ready for production deployment

---

**Document Version**: 1.0
**Last Updated**: 2025-12-23
**Next Review**: After production deployment
