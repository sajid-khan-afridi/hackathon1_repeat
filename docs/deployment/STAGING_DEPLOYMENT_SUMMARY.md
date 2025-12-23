# Phase 4B Staging Deployment - Summary & Quick Start

**Feature**: Personalization Engine
**Branch**: `1-personalization-engine`
**Status**: Ready for Staging Deployment
**Date**: 2025-12-23

---

## Quick Start Guide

### Prerequisites Checklist

Before starting deployment, ensure you have:

- [ ] Railway account with project admin access
- [ ] Neon PostgreSQL account
- [ ] Vercel/Netlify account (or GitHub Pages access)
- [ ] OpenAI API key with sufficient quota
- [ ] Qdrant cluster URL and API key
- [ ] Google OAuth staging app credentials
- [ ] GitHub OAuth staging app credentials
- [ ] PostgreSQL client (`psql`) installed
- [ ] Railway CLI installed (optional): `npm install -g @railway/cli`

---

## Deployment Steps (60-90 minutes)

### Step 1: Database Setup (15-20 min)

```bash
# 1. Create Neon staging database
# Visit: https://console.neon.tech
# Create database: hackathon1_staging

# 2. Export connection string
export STAGING_DB="postgresql://user:password@ep-staging-*.neon.tech/hackathon1_staging?sslmode=require"

# 3. Run migration script
cd "D:\GitHub Connected\hackathon1_repeat"
./backend/scripts/run_migration_staging.sh

# Expected: ✅ Migration completed successfully!
```

**Documentation**: `NEON_STAGING_DATABASE_SETUP.md`

---

### Step 2: Railway Backend Deployment (20-30 min)

```bash
# 1. Create Railway staging environment
# Visit: https://railway.app/dashboard
# Create environment: "staging"

# 2. Configure environment variables
# Use backend/.env.staging as template
# Update all placeholders with actual values

# 3. Deploy backend
railway environment staging
railway up

# Or use Railway Dashboard → Deploy from branch: 1-personalization-engine

# 4. Get staging URL
export STAGING_BACKEND_URL="https://hackathon1-staging.up.railway.app"

# 5. Verify health
curl $STAGING_BACKEND_URL/health
# Expected: {"status":"ok"}
```

**Documentation**: `RAILWAY_STAGING_SETUP.md`

---

### Step 3: Frontend Deployment (15-20 min)

```bash
# 1. Deploy to Vercel (recommended)
npm install -g vercel
vercel login
vercel

# 2. Configure environment variable
vercel env add API_URL
# Value: https://hackathon1-staging.up.railway.app

# 3. Deploy from feature branch
git checkout 1-personalization-engine
vercel --prod

# 4. Get staging URL
export STAGING_FRONTEND_URL="https://hackathon1-staging-abc123.vercel.app"

# 5. Update CORS in Railway
railway variables set CORS_ORIGINS="$STAGING_FRONTEND_URL,http://localhost:3000"
railway up
```

**Documentation**: `FRONTEND_STAGING_DEPLOYMENT.md`

---

### Step 4: Smoke Testing (15-20 min)

```bash
# Run automated smoke tests
./backend/scripts/smoke_test_staging.sh $STAGING_BACKEND_URL

# Expected: ✅ All smoke tests PASSED

# Manual verification
open $STAGING_FRONTEND_URL
# - Test login (Google/GitHub OAuth)
# - Create user profile
# - View recommendations
# - Start a chapter
# - Test chatbot
```

**Documentation**: `SMOKE_TEST_CHECKLIST.md`

---

### Step 5: UAT Handoff (10-15 min)

```bash
# Document staging URLs for UAT team
echo "Frontend: $STAGING_FRONTEND_URL" > staging_urls.txt
echo "Backend: $STAGING_BACKEND_URL" >> staging_urls.txt
echo "API Docs: $STAGING_BACKEND_URL/api/docs" >> staging_urls.txt

# Create test accounts (via SQL or OAuth login)
# - test.beginner@staging.test
# - test.intermediate@staging.test
# - test.advanced@staging.test
```

**Documentation**: `UAT_TEST_PLAN.md`

---

## Deployment Artifacts Created

### Configuration Files

1. **`backend/.env.staging`**
   - Complete staging environment configuration
   - All Phase 4B variables documented
   - Placeholder values for credentials

### Documentation

2. **`docs/deployment/RAILWAY_STAGING_SETUP.md`**
   - Step-by-step Railway configuration
   - Environment variable setup
   - Troubleshooting guide

3. **`docs/deployment/NEON_STAGING_DATABASE_SETUP.md`**
   - Database creation instructions
   - Migration execution steps
   - Rollback procedures

4. **`docs/deployment/FRONTEND_STAGING_DEPLOYMENT.md`**
   - Vercel/Netlify deployment guide
   - Environment variable configuration
   - CORS setup

5. **`docs/deployment/STAGING_DEPLOYMENT_RUNBOOK.md`**
   - Comprehensive deployment workflow
   - Pre-deployment checklist
   - Post-deployment validation
   - Rollback procedures
   - Production deployment steps

6. **`docs/deployment/SMOKE_TEST_CHECKLIST.md`**
   - 43 manual test cases
   - Automated test validation
   - Results tracking template

7. **`docs/deployment/UAT_TEST_PLAN.md`**
   - 26 comprehensive UAT test cases
   - 3 user personas
   - Bug reporting template
   - Sign-off procedures

### Scripts

8. **`backend/scripts/run_migration_staging.sh`**
   - Automated migration execution
   - Pre-flight validation
   - Post-migration verification
   - Error handling

9. **`backend/scripts/rollback_migration_staging.sh`**
   - Emergency rollback procedure
   - Data backup before rollback
   - Verification steps

10. **`backend/scripts/smoke_test_staging.sh`**
    - 30+ automated health checks
    - Performance testing
    - Security validation
    - Results logging

---

## Staging Environment URLs

After deployment, you will have:

```
Frontend Staging:    https://hackathon1-staging-abc123.vercel.app
Backend API:         https://hackathon1-staging.up.railway.app
API Documentation:   https://hackathon1-staging.up.railway.app/api/docs
Health Endpoint:     https://hackathon1-staging.up.railway.app/health
```

---

## Success Criteria

Deployment is successful when:

✅ **Infrastructure**
- Railway staging environment active
- Neon staging database accessible
- Frontend deployed and accessible

✅ **Functionality**
- All Phase 4B endpoints responding
- OAuth login working (Google + GitHub)
- Skill classification calculating correctly
- Recommendations personalized to skill level
- Progress tracking persisting to database
- RAG chatbot personalization working

✅ **Performance**
- API response times < 1000ms
- Page load times < 3 seconds
- No critical errors in logs

✅ **Quality**
- Automated smoke tests pass rate ≥ 80%
- Manual smoke tests pass
- Ready for UAT

---

## Next Steps After Staging Deployment

1. **Execute Smoke Tests** (15-20 min)
   - Run automated script: `smoke_test_staging.sh`
   - Complete manual checklist: `SMOKE_TEST_CHECKLIST.md`
   - Document any issues found

2. **UAT Execution** (2-3 days)
   - Provide staging URLs to UAT team
   - Create test accounts for 3 personas
   - Execute 26 UAT test cases
   - Collect feedback and bug reports

3. **Bug Fixes** (as needed)
   - Fix blocker and critical bugs
   - Redeploy to staging
   - Regression test

4. **UAT Sign-off**
   - Obtain Product Owner approval
   - Document UAT results
   - Approve for production

5. **Production Deployment**
   - Follow `STAGING_DEPLOYMENT_RUNBOOK.md` → Production section
   - Merge `1-personalization-engine` → `main`
   - Run migration on production database
   - Deploy backend and frontend to production
   - Run production smoke tests

---

## Rollback Procedures

### If Critical Issues Found in Staging

**Backend Rollback**:
```bash
railway environment staging
railway rollback
```

**Database Rollback**:
```bash
export STAGING_DB="postgresql://..."
./backend/scripts/rollback_migration_staging.sh
```

**Frontend Rollback**:
- Vercel Dashboard → Deployments → Rollback to previous

---

## Troubleshooting Common Issues

### Issue: Migration Fails

**Symptom**: `run_migration_staging.sh` exits with error

**Fix**:
```bash
# Check prerequisite tables
psql "$STAGING_DB" -c "\dt"

# Verify users and user_profiles exist
# If missing, apply base migrations first (001-007)
```

### Issue: Backend Deployment Fails

**Symptom**: Railway deployment stuck or errors

**Fix**:
1. Check Railway build logs
2. Verify environment variables set
3. Test locally: `cd backend && uvicorn app.main:app`

### Issue: CORS Errors

**Symptom**: Browser console shows CORS errors

**Fix**:
```bash
# Update CORS_ORIGINS in Railway
railway variables set CORS_ORIGINS="https://hackathon1-staging.vercel.app,http://localhost:3000"
railway up
```

### Issue: OAuth Login Fails

**Symptom**: Redirect URI mismatch

**Fix**:
1. Verify Railway URL matches OAuth app redirect URI
2. Update Google/GitHub OAuth apps
3. Update Railway environment variables

---

## Support & Escalation

**Deployment Issues**: DevOps Team
**Database Issues**: Database Administrator
**UAT Questions**: Product Owner
**Bug Reports**: QA Lead

---

## Document Index

All deployment documentation located in: `docs/deployment/`

1. `STAGING_DEPLOYMENT_SUMMARY.md` (this file) - Quick start guide
2. `STAGING_DEPLOYMENT_RUNBOOK.md` - Comprehensive deployment workflow
3. `RAILWAY_STAGING_SETUP.md` - Railway configuration
4. `NEON_STAGING_DATABASE_SETUP.md` - Database setup
5. `FRONTEND_STAGING_DEPLOYMENT.md` - Frontend deployment
6. `SMOKE_TEST_CHECKLIST.md` - Smoke test procedures
7. `UAT_TEST_PLAN.md` - User acceptance testing

---

## Version History

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0 | 2025-12-23 | Initial staging deployment package | DevOps Team |

---

**Ready to Deploy?**

Follow the Quick Start Guide above or reference the comprehensive runbook for detailed steps.

**Questions?** Review the troubleshooting section or contact the DevOps team.

**Good luck with your staging deployment! 🚀**
