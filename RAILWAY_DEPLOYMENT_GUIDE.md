# Railway Deployment Guide
## Feature: 001-user-auth - Authentication System
**Last Updated:** 2025-12-20

---

## Quick Deployment Steps

### 1. Prepare Production Environment Variables

Copy all variables from `.env.production` to your Railway project:

**Railway Dashboard → Your Project → Variables Tab**

### 2. Critical Environment Variables (MUST SET)

```bash
# Environment
ENVIRONMENT=production
LOG_LEVEL=info
LOG_FORMAT=json

# API Configuration
API_HOST=0.0.0.0
API_PORT=8000
API_URL=https://your-project.up.railway.app

# Frontend
FRONTEND_URL=https://your-docs-site.github.io

# CORS (comma-separated, NO wildcards)
CORS_ORIGINS=https://your-docs-site.github.io,https://your-project.up.railway.app

# Database (from Railway Postgres plugin)
DATABASE_URL=${DATABASE_URL}?sslmode=require

# Vector Database
QDRANT_URL=https://your-cluster.cloud.qdrant.io
QDRANT_API_KEY=<your-qdrant-key>
QDRANT_COLLECTION=robotics_textbook

# OpenAI
OPENAI_API_KEY=sk-<your-key>
OPENAI_MODEL=gpt-4o-mini

# JWT Configuration
JWT_ALGORITHM=RS256
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=60
JWT_REFRESH_TOKEN_EXPIRE_DAYS=30

# JWT Keys (COPY EXACT STRINGS - including quotes!)
JWT_PRIVATE_KEY="-----BEGIN PRIVATE KEY-----\nMIIEvAIBADANBgkqhkiG9w0BAQEFAASCBKYwggSiAgEAAoIBAQCWn9tbNON3i9/i\nUSgJF3S6UYaliADgjF5KaUQxwg8mDEv5f+2hY2G6zKmA4OVErbCYuucasedFZ9Du\nxiGWoZ4EBPcdwlmEMsLmk4PR6EPITpkpqQRuNT5x9HZAl0ZFomVUhDWnARg6kH19\nLBN88lIUbpcZcMkYgK8CeL1qnZT3ZJsWpVHOAdm1BmONwGYwbPhUfCUiLObPmLf6\n5kXQmabM+X3dcHtknDfxn+GLkCdoBCjr0GMRIfMzhV2lzcUU0mBH7deSS4IIXXs1\n695y1hKSp1Lm5WLegR+Nb/bVDzXvH3pe24yWVeoUM/XPDWMLlh82UqgN06gmILkf\nvZXp1cW1AgMBAAECggEAF9oUmGrVonap62LCnj3ypBaI5emViflwbQQiYmbCk1nb\nJhVWAfqE3VnPdbMnWyrODkF6m0WUw+7erRD+/KV3KlfG8hV2q3KGXpD5jOHk6fFY\nW7qCExmnW9BEcZIC9L/0n9+Xx2xq88sxtAKoy15sFQQ2NTchARpPnGqWRC3UCyDK\n90+Z7z9wcWqnxzMIbqfST8pKshVcJuMefxWP4cwDOgpTzhnhrjzX7e7q2WxoY73t\nVaGlvGJxLX4wjbYOLsX1PFo+DO1sooaZ7FMacDUP2HhOV+RbYHj3A6vs1Q/zKFDL\n0svMHgM4QMhBbXGWmkkrX/O9ZqZB4r+nDpe+pP59wQKBgQDT3x5LAkL/CmAfyK5h\n7kLPIYTyHj1Grwu/heQAbMfwpTu7Acufi2wekKDn+Dej1jVgDcsvz2WH0DNzQ1kI\nLMl5yLWlI0zWlwiHx115CT739xzJ6TRTKXm1lRpnxVOAcWPg3K6W1fpQHLTHmwVv\n/lnzAreYw9e7xIGNGlaT6K/MjQKBgQC1/xL1p9hRl618zgeEcFYLm4qNJDMo1CYF\n+EnIfTq4haMTHZzfM0LLA/p3UomoBke6UwTe1/UEhpw2j3vk5ZCFvju9O9jjyN8H\nY8aejz7K8WFrPKziyq28Ps+Z/U/bX7z6Q8aqa9zzm9HPyIE2yrymU+SJIx3Nq4mE\nxNoswoOXyQKBgAydVjJlGhQRTBPYYPmXtAadGktIHaHdvjdTA3Y7A9SCIgJKYxNG\nu9M+DTaCgt2QXXEwoQ5hrMvfS5DrS5u+/ufvWVFRvrtwFqPu7NPOBx/AaMhkyCbD\na2myTPYsu4IEPkwfF44Hg3XNn73arwIyBK0BVmkUD/4v/v6luM3ppr3pAoGAA7lV\nbPtjPMAqx5v7oWUfDZvAKQUtms9Ee+jJsbWGRxJM9O4tMI/+9OJ+9IQW1v5R52kd\nd2XZ2J7cgW1MLrHX1DTBSiz4UijhD2imPsZVlZd809HpA7b7+Rcb7l21jZqW5lQO\ns5wDaGSu1V6X7kQVuh2LVpcGXBf/d10DzdipEqkCgYBRHv12zCzaHq9kxvhaLYrv\nthbBJYzQWw1cbEgngfzQXgHJ4LYkljLAAomy1ElurLVwmVQeL/SrtP3oBn16Mdso\ncjD4DIWWaN7M9tjtA0sO9Bqh3QMqQi/CfIBIxVakRuEb0Ogbe9zWuNe5nj5MvEql\nO7N+ribDKQ5GBKNSIvu7cg==\n-----END PRIVATE KEY-----"

JWT_PUBLIC_KEY="-----BEGIN PUBLIC KEY-----\nMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAlp/bWzTjd4vf4lEoCRd0\nulGGpYgA4IxeSmlEMcIPJgxL+X/toWNhusypgODlRK2wmLrnGrHnRWfQ7sYhlqGe\nBAT3HcJZhDLC5pOD0ehDyE6ZKakEbjU+cfR2QJdGRaJlVIQ1pwEYOpB9fSwTfPJS\nFG6XGXDJGICvAni9ap2U92SbFqVRzgHZtQZjjcBmMGz4VHwlIizmz5i3+uZF0Jmm\nzPl93XB7ZJw38Z/hi5AnaAQo69BjESHzM4Vdpc3FFNJgR+3XkkuCCF17NevectYS\nkqdS5uVi3oEfjW/21Q817x96XtuMllXqFDP1zw1jC5YfNlKoDdOoJiC5H72V6dXF\ntQIDAQAB\n-----END PUBLIC KEY-----"

# GitHub OAuth (PRODUCTION APP)
GITHUB_CLIENT_ID=<production-client-id>
GITHUB_CLIENT_SECRET=<production-client-secret>
GITHUB_REDIRECT_URI=https://your-project.up.railway.app/auth/github/callback

# Security
CSRF_SECRET_KEY=<generate-with-openssl-rand-hex-32>

# Rate Limiting
RATE_LIMIT_ANONYMOUS=50
RATE_LIMIT_AUTHENTICATED=200
```

---

## Step-by-Step Deployment

### Step 1: Create Production GitHub OAuth App

1. Go to: https://github.com/settings/developers
2. Click **New OAuth App**
3. Fill in:
   - **Application name:** `Your App Name - Production`
   - **Homepage URL:** `https://your-docs-site.github.io`
   - **Authorization callback URL:** `https://your-project.up.railway.app/auth/github/callback`
4. Click **Register application**
5. Copy **Client ID** and generate **Client Secret**
6. Save both securely

### Step 2: Generate CSRF Secret

```bash
# Run this command to generate a strong CSRF secret
openssl rand -hex 32
```

Copy the output (64 characters).

### Step 3: Configure Railway Variables

**Method 1: Railway Dashboard (Recommended)**

1. Log in to Railway: https://railway.app
2. Select your project
3. Click **Variables** tab
4. Click **Raw Editor**
5. Paste all variables from `.env.production`
6. Replace placeholder values:
   - `your-api-domain.com` → Your Railway URL
   - `your-frontend-domain.com` → Your GitHub Pages URL
   - OAuth credentials → From Step 1
   - CSRF secret → From Step 2
   - Database URL → Use Railway's `${DATABASE_URL}?sslmode=require`
   - Qdrant credentials → From your Qdrant Cloud dashboard
   - OpenAI key → From OpenAI dashboard

**Method 2: Railway CLI**

```bash
# Install Railway CLI
npm i -g @railway/cli

# Login
railway login

# Link to your project
railway link

# Set variables one by one
railway variables set ENVIRONMENT=production
railway variables set JWT_ALGORITHM=RS256
# ... (repeat for all variables)
```

### Step 4: Verify Database Connection

```bash
# SSH into Railway container
railway run bash

# Test database connection
python -c "import psycopg2; conn = psycopg2.connect('$DATABASE_URL'); print('✅ Database connected')"
```

### Step 5: Run Database Migrations

**Option A: Via Railway CLI (Recommended)**

```bash
# Run migrations from local machine
railway run python backend/app/migrations/run_migrations.py --dry-run

# If dry-run looks good, execute
railway run python backend/app/migrations/run_migrations.py
```

**Option B: Via Railway Dashboard**

1. Go to **Settings** → **Deploy**
2. Add build command:
   ```
   pip install -r backend/requirements.txt && python backend/app/migrations/run_migrations.py
   ```

### Step 6: Deploy Application

```bash
# Deploy via Railway CLI
railway up

# OR push to GitHub (if Railway is watching your repo)
git push origin 001-user-auth:main
```

### Step 7: Verify Deployment

```bash
# Health check
curl https://your-project.up.railway.app/health

# Expected response:
# {"status":"ok","timestamp":"2025-12-20T..."}
```

---

## Post-Deployment Testing

### Test 1: Health Endpoint

```bash
curl https://your-project.up.railway.app/health
```

Expected: `{"status":"ok"}`

### Test 2: User Signup

```bash
curl -X POST https://your-project.up.railway.app/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "SecurePassword123!",
    "name": "Test User"
  }'
```

Expected: `{"access_token": "...", "refresh_token": "..."}`

### Test 3: User Login

```bash
curl -X POST https://your-project.up.railway.app/auth/login \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "SecurePassword123!"
  }'
```

Expected: `{"access_token": "...", "refresh_token": "..."}`

### Test 4: GitHub OAuth Flow

1. Visit: `https://your-project.up.railway.app/auth/github`
2. Should redirect to GitHub authorization
3. After authorization, should redirect back with tokens

### Test 5: Protected Endpoint

```bash
# Get a token first (from signup/login)
ACCESS_TOKEN="<your-token>"

curl https://your-project.up.railway.app/users/me \
  -H "Authorization: Bearer $ACCESS_TOKEN"
```

Expected: User profile data

### Test 6: CORS Validation

```bash
curl -X OPTIONS https://your-project.up.railway.app/auth/login \
  -H "Origin: https://your-docs-site.github.io" \
  -H "Access-Control-Request-Method: POST" \
  -v
```

Expected: `Access-Control-Allow-Origin: https://your-docs-site.github.io`

---

## Environment Variable Checklist

- [ ] `ENVIRONMENT=production`
- [ ] `LOG_FORMAT=json`
- [ ] `API_URL` set to Railway URL
- [ ] `FRONTEND_URL` set to GitHub Pages URL
- [ ] `CORS_ORIGINS` contains both URLs (comma-separated, HTTPS only)
- [ ] `DATABASE_URL` includes `?sslmode=require`
- [ ] `QDRANT_URL` uses HTTPS
- [ ] `QDRANT_API_KEY` set
- [ ] `QDRANT_COLLECTION` set
- [ ] `OPENAI_API_KEY` starts with `sk-`
- [ ] `JWT_ALGORITHM=RS256`
- [ ] `JWT_PRIVATE_KEY` set (with quotes, single-line format)
- [ ] `JWT_PUBLIC_KEY` set (with quotes, single-line format)
- [ ] `GITHUB_CLIENT_ID` set (production app)
- [ ] `GITHUB_CLIENT_SECRET` set (production app)
- [ ] `GITHUB_REDIRECT_URI` points to Railway URL
- [ ] `CSRF_SECRET_KEY` set (64 chars, generated with openssl)
- [ ] `RATE_LIMIT_ANONYMOUS` set (50)
- [ ] `RATE_LIMIT_AUTHENTICATED` set (200)

---

## Troubleshooting

### Issue: "JWT decode error"

**Cause:** JWT keys not properly formatted

**Fix:**
1. Ensure JWT keys are wrapped in **double quotes**
2. Verify keys contain `\n` (literal backslash-n, not actual newlines)
3. Run: `python convert_keys.py` to regenerate

### Issue: "CORS error in browser"

**Cause:** CORS origins misconfigured

**Fix:**
1. Check `CORS_ORIGINS` has your frontend URL
2. Ensure URLs use HTTPS (not HTTP)
3. No trailing slashes in URLs
4. No wildcards (`*`)

### Issue: "Database connection failed"

**Cause:** DATABASE_URL missing `sslmode=require`

**Fix:**
```bash
railway variables set DATABASE_URL="${DATABASE_URL}?sslmode=require"
```

### Issue: "GitHub OAuth redirect fails"

**Cause:** OAuth callback URL mismatch

**Fix:**
1. Check GitHub OAuth app settings
2. Callback URL must EXACTLY match: `https://your-project.up.railway.app/auth/github/callback`
3. Update `GITHUB_REDIRECT_URI` environment variable to match

### Issue: "Rate limit errors"

**Cause:** Rate limits too restrictive

**Fix:**
```bash
railway variables set RATE_LIMIT_ANONYMOUS=100
railway variables set RATE_LIMIT_AUTHENTICATED=500
```

---

## Rollback Procedure

If deployment fails:

```bash
# 1. Revert to previous deployment
railway rollback

# 2. Check logs
railway logs

# 3. Fix issues and redeploy
railway up
```

---

## Monitoring

### View Logs

```bash
# Real-time logs
railway logs --tail 100

# Filter by level
railway logs --filter error
```

### Key Metrics to Monitor

- **Response time:** Should be < 500ms for most endpoints
- **Error rate:** Should be < 1%
- **Database connections:** Should not exceed pool limit
- **Rate limit hits:** Monitor for abuse
- **JWT token generation:** Should succeed > 99%

---

## Security Checklist

- [ ] All secrets stored in Railway variables (not in code)
- [ ] RSA keys never committed to git
- [ ] CORS restricted to specific origins (no wildcards)
- [ ] HTTPS enforced for all external URLs
- [ ] Database uses SSL (sslmode=require)
- [ ] CSRF protection enabled
- [ ] Rate limiting configured
- [ ] OAuth apps separated (dev vs production)
- [ ] Sensitive data not logged
- [ ] Error messages don't expose internals

---

## Additional Resources

- **Railway Documentation:** https://docs.railway.app
- **Deployment Checklist:** `DEPLOYMENT_CHECKLIST.md`
- **Production Deployment Guide:** `PRODUCTION_DEPLOYMENT.md`
- **RSA Key Conversion Guide:** `RSA_KEY_CONVERSION_GUIDE.md`
- **Security Checklist:** `backend/OWASP_SECURITY_CHECKLIST.md`

---

## Support

If you encounter issues:

1. Check Railway logs: `railway logs`
2. Verify environment variables: `railway variables`
3. Test database connection: `railway run python -c "import psycopg2; ..."`
4. Review this guide's troubleshooting section
5. Check Railway status: https://railway.statuspage.io/

---

**Deployment Status:** Ready for Production ✅

**Last Validated:** 2025-12-20

**Feature Branch:** 001-user-auth
