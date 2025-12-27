# Deployment Plan: Phase 4A & 4B to Production

**Date**: 2025-12-23
**Status**: Ready for Execution
**Objective**: Deploy Authentication (Phase 4A) and Personalization Engine (Phase 4B) to production

---

## Executive Summary

All phases (1-4B) code exists in the `main` branch locally. However, the production deployments (Railway backend, GitHub Pages frontend) are running an older version **before** Phase 4A/4B were merged.

**Root Cause**: The Railway deployment and GitHub Pages build need to be manually triggered/rebuilt after the merge commits.

---

## Current State Analysis

| Component | Local Code | Production Status | Required Action |
|-----------|------------|-------------------|-----------------|
| Frontend (GitHub Pages) | ✅ All phases merged | ❌ Old build | Trigger redeploy |
| Backend (Railway) | ✅ All phases merged | ❌ Old build | Trigger redeploy + add env vars |
| Phase 1-3 | ✅ Complete | ✅ Working | No action needed |
| Phase 4A (Auth) | ✅ Complete | ❌ Not deployed | Deploy + configure env vars |
| Phase 4B (Personalization) | ✅ Complete | ❌ Not deployed | Deploy + run migrations |

---

## Deployment Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         USER FLOW                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────────────┐      ┌──────────────────┐                │
│  │  GitHub Pages    │      │   Railway        │                │
│  │  Frontend        │◄────►│   Backend API    │                │
│  │  (Docusaurus)    │      │   (FastAPI)      │                │
│  └──────────────────┘      └────────┬─────────┘                │
│                                     │                           │
│                                     ▼                           │
│                        ┌────────────────────────┐              │
│                        │  External Services     │              │
│                        ├────────────────────────┤              │
│                        │ • Neon PostgreSQL      │              │
│                        │ • Qdrant Vector DB     │              │
│                        │ • OpenAI API           │              │
│                        │ • GitHub OAuth         │              │
│                        └────────────────────────┘              │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

---

## Step-by-Step Execution Plan

### STEP 1: Generate RSA Keys for JWT

**Required for**: Phase 4A Authentication

```bash
# Generate RSA key pair (2048-bit)
openssl genrsa -out jwt_private_production.pem 2048
openssl rsa -in jwt_private_production.pem -pubout -out jwt_public_production.pem

# Convert to single-line format for environment variables
# Linux/Mac:
awk 'NF {sub(/\r/, ""); printf "%s\\n",$0;}' jwt_private_production.pem
awk 'NF {sub(/\r/, ""); printf "%s\\n",$0;}' jwt_public_production.pem

# Windows PowerShell:
(Get-Content jwt_private_production.pem -Raw) -replace "`r`n", "\n"
(Get-Content jwt_public_production.pem -Raw) -replace "`r`n", "\n"
```

**Security Notes**:
- Store these securely in Railway Secrets (not in code)
- NEVER commit actual keys to git
- Use different keys for dev/staging/production

---

### STEP 2: Configure GitHub OAuth App

**Required for**: Phase 4A Google/GitHub OAuth login

1. **Create GitHub OAuth App** (Recommended - easier setup)
   - Go to: https://github.com/settings/developers
   - Click "New OAuth App"
   - Fill in:
     - Application name: `Robotics Textbook Production`
     - Homepage URL: `https://sajid-khan-afridi.github.io/hackathon1_repeat`
     - Authorization callback URL: `https://hackathon1repeat-production.up.railway.app/auth/github/callback`
   - Save the `Client ID` and generate a new `Client Secret`

2. **(Optional) Create Google OAuth App**
   - Go to: https://console.cloud.google.com/apis/credentials
   - Create OAuth 2.0 Client ID (Web application)
   - Add authorized redirect URI: `https://hackathon1repeat-production.up.railway.app/auth/google/callback`

---

### STEP 3: Configure Railway Environment Variables

Go to: Railway Dashboard → Your Project → Variables

#### Critical Variables (Already Configured)
```bash
DATABASE_URL=postgres://user:password@ep-xxx.neon.tech/dbname?sslmode=require
OPENAI_API_KEY=sk-your-key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-key
QDRANT_COLLECTION=robotics_textbook
```

#### Add These Variables for Phase 4A/4B:

```bash
# Environment
ENVIRONMENT=production

# CORS - Allow frontend to call backend
CORS_ORIGINS=https://sajid-khan-afridi.github.io,https://hackathon1repeat-production.up.railway.app

# JWT Authentication (from Step 1)
JWT_PRIVATE_KEY="-----BEGIN PRIVATE KEY-----\nMIIE...\n-----END PRIVATE KEY-----"
JWT_PUBLIC_KEY="-----BEGIN PUBLIC KEY-----\nMIIB...\n-----END PUBLIC KEY-----"
JWT_ALGORITHM=RS256
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=1440
JWT_REFRESH_TOKEN_EXPIRE_DAYS=30

# GitHub OAuth (from Step 2)
GITHUB_CLIENT_ID=your-github-client-id
GITHUB_CLIENT_SECRET=your-github-client-secret
GITHUB_REDIRECT_URI=https://hackathon1repeat-production.up.railway.app/auth/github/callback

# Frontend URL for OAuth redirects
FRONTEND_URL=https://sajid-khan-afridi.github.io/hackathon1_repeat

# Security
CSRF_SECRET_KEY=your-32-char-random-string
# Generate with: python -c "import secrets; print(secrets.token_urlsafe(32))"
```

---

### STEP 4: Trigger Railway Backend Redeploy

**Option A: Automatic (Recommended)**
- Push a trivial commit to trigger redeploy:
```bash
git commit --allow-empty -m "chore: trigger Railway redeploy for Phase 4A/4B"
git push origin main
```

**Option B: Manual via Railway Dashboard**
1. Go to Railway Dashboard → Your Project
2. Click "Redeploy" button
3. Monitor deployment logs

**What to watch in logs**:
```
✅ "Running database migrations..."
✅ "Connected successfully"
✅ "All migrations completed successfully!"
✅ "Starting uvicorn on port XXXX"
✅ "Including auth router"
✅ "Including personalization router"
```

---

### STEP 5: Run Database Migrations for New Tables

The Phase 4A/4B features require new database tables. After Railway redeploys:

```bash
# Via Railway Console or SSH:
cd backend && python run_migrations.py
```

Or migrations should run automatically on startup if configured in `start.sh`.

**New tables required**:
- `users` - User accounts
- `user_profiles` - 5-question profile data
- `skill_level_classifications` - Computed skill tiers
- `chapter_progress` - Started/completed/bookmarked chapters
- `chapter_metadata` - Chapter recommendations metadata

---

### STEP 6: Trigger GitHub Pages Frontend Redeploy

**Option A: Automatic (Already Configured)**
- The `.github/workflows/deploy.yml` triggers on push to `main`
- If the merge commit already triggered, check Actions tab

**Option B: Manual Trigger**
```bash
git commit --allow-empty -m "chore: trigger GitHub Pages rebuild"
git push origin main
```

**Verify at**: https://github.com/sajid-khan-afridi/hackathon1_repeat/actions

---

### STEP 7: Smoke Tests

After both deployments complete, run these tests:

#### Backend Health Check
```bash
# Simple health
curl https://hackathon1repeat-production.up.railway.app/health
# Expected: {"status":"ok"}

# Detailed health
curl https://hackathon1repeat-production.up.railway.app/api/v1/health
# Expected: {"status":"healthy","database":"healthy","vector_store":"healthy","llm":"healthy"}
```

#### Auth Endpoints
```bash
# CSRF token
curl https://hackathon1repeat-production.up.railway.app/auth/csrf
# Expected: Message about CSRF token

# Signup (test)
curl -X POST https://hackathon1repeat-production.up.railway.app/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"TestPass123"}'
# Expected: 201 Created or validation error
```

#### Frontend Pages
```bash
# Login page
curl -I https://sajid-khan-afridi.github.io/hackathon1_repeat/login
# Expected: 200 OK

# Signup page
curl -I https://sajid-khan-afridi.github.io/hackathon1_repeat/signup
# Expected: 200 OK

# Profile page
curl -I https://sajid-khan-afridi.github.io/hackathon1_repeat/profile
# Expected: 200 OK
```

---

## Verification Checklist

### Backend (Railway)
- [ ] Health check returns 200
- [ ] `/auth/csrf` endpoint returns 200
- [ ] `/auth/signup` endpoint exists (may return validation on empty POST)
- [ ] `/api/v1/recommendations` endpoint exists (may require auth)
- [ ] `/api/v1/progress/*` endpoints exist (may require auth)

### Frontend (GitHub Pages)
- [ ] `/login` page loads
- [ ] `/signup` page loads
- [ ] `/profile` page loads
- [ ] Chatbot widget shows on docs pages
- [ ] Theme toggle works

### Integration Tests
- [ ] Can create user account via `/auth/signup`
- [ ] Can login via `/auth/login`
- [ ] Profile wizard appears after signup
- [ ] Profile data saves to database
- [ ] Skill level classification computes
- [ ] Chapter recommendations return

---

## Rollback Plan (If Something Fails)

### If Backend Fails
1. Revert Railway to previous deploy
2. Check Railway logs for specific error
3. Fix issue (usually missing env var)
4. Redeploy

### If Frontend Fails
1. Revert GitHub Pages to previous build
2. Check build logs in Actions tab
3. Fix issue
4. Trigger new build

---

## Troubleshooting

### Issue: Auth endpoints return 404
**Cause**: Railway didn't redeploy with new code
**Fix**: Manually trigger redeploy in Railway dashboard

### Issue: "JWT_PRIVATE_KEY not configured"
**Cause**: Missing environment variable
**Fix**: Add JWT keys to Railway Secrets

### Issue: "Database connection failed"
**Cause**: Missing migration or wrong DATABASE_URL
**Fix**: Run migrations, verify DATABASE_URL format

### Issue: CORS errors from frontend
**Cause**: CORS_ORIGINS doesn't include GitHub Pages URL
**Fix**: Add `https://sajid-khan-afridi.github.io` to CORS_ORIGINS

### Issue: OAuth callback fails
**Cause**: Wrong redirect URI configured in OAuth app
**Fix**: Update OAuth app with correct Railway callback URL

---

## Post-Deployment Tasks

1. **Monitor logs** for first 24 hours
2. **Test authentication flow** end-to-end
3. **Test personalization** with sample user
4. **Verify rate limiting** works correctly
5. **Create admin user** for content management (if needed)

---

## Contact & Support

- Railway Dashboard: https://railway.app/
- GitHub Actions: https://github.com/sajid-khan-afridi/hackathon1_repeat/actions
- Neon Database: https://console.neon.tech/
- Qdrant Console: https://cloud.qdrant.io/

---

**Last Updated**: 2025-12-23
**Status**: Ready for Execution
