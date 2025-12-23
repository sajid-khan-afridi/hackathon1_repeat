# Railway Staging Environment Setup Guide

**Purpose**: Configure Railway staging environment for Phase 4B Personalization Engine UAT
**Target Environment**: Staging (separate from production)
**Branch**: `1-personalization-engine`
**Date**: 2025-12-23

---

## Prerequisites

- Railway account with admin access to project
- Railway CLI installed (optional but recommended): `npm install -g @railway/cli`
- Git repository connected to Railway
- Access to Neon PostgreSQL staging database

---

## Step 1: Create Staging Environment in Railway

### Via Railway Dashboard (Recommended)

1. **Navigate to Project**
   - Go to [Railway Dashboard](https://railway.app/dashboard)
   - Select your project: `hackathon1_repeat` or similar

2. **Create New Environment**
   - Click on environment dropdown (top-right)
   - Click **"New Environment"**
   - Name: `staging`
   - Description: `Staging environment for UAT testing`
   - Click **"Create Environment"**

3. **Switch to Staging Environment**
   - Use environment dropdown to switch to `staging`
   - All subsequent configuration will apply to staging only

### Via Railway CLI (Alternative)

```bash
# Login to Railway
railway login

# Link to your project
railway link

# Create staging environment
railway environment create staging

# Switch to staging environment
railway environment staging
```

---

## Step 2: Configure GitHub Branch Deployment

### Option A: Manual Deployment (Recommended for Initial Setup)

1. **Deployment Settings**
   - In Railway dashboard, select your backend service
   - Go to **Settings** → **Service**
   - Scroll to **Source**

2. **Configure Branch**
   - Branch: `1-personalization-engine`
   - Root Directory: `backend` (if monorepo structure)
   - Build Command: (leave default or use `pip install -r requirements.txt`)
   - Start Command: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

3. **Deployment Trigger**
   - Enable: **Manual Deployments** (for controlled staging releases)
   - Disable: **Auto-deploy on push** (to prevent accidental deployments)

### Option B: Auto-Deploy from Feature Branch

1. **Configure in Settings**
   - Check **"Deploy on Push"** for `1-personalization-engine` branch
   - Staging will auto-deploy on every push to feature branch

2. **Setup Branch Protection**
   - In GitHub repository settings
   - Protect `1-personalization-engine` to prevent force pushes
   - Require PR approvals before merge

---

## Step 3: Configure Environment Variables

### Method 1: Via Railway Dashboard (Recommended)

1. **Navigate to Variables**
   - In Railway dashboard (staging environment)
   - Select backend service
   - Go to **Variables** tab

2. **Add Variables in Groups**

   **Database Configuration:**
   ```
   DATABASE_URL=postgresql://user:password@ep-staging-endpoint.neon.tech/hackathon1_staging?sslmode=require
   DB_POOL_SIZE=5
   DB_MAX_OVERFLOW=10
   DB_POOL_TIMEOUT=30
   ```

   **Environment Settings:**
   ```
   ENVIRONMENT=staging
   DEBUG=false
   LOG_LEVEL=INFO
   LOG_FORMAT=json
   API_HOST=0.0.0.0
   API_V1_STR=/api/v1
   ```

   **OpenAI Configuration:**
   ```
   OPENAI_API_KEY=sk-proj-your-openai-api-key
   EMBEDDING_MODEL=text-embedding-3-small
   CHAT_MODEL=gpt-4-turbo-preview
   ```

   **Qdrant Vector Database:**
   ```
   QDRANT_URL=https://your-cluster.gcp.cloud.qdrant.io
   QDRANT_API_KEY=your-qdrant-api-key
   QDRANT_COLLECTION=robotics_textbook_staging
   ```

   **CORS & Frontend (UPDATE after frontend deployment):**
   ```
   FRONTEND_URL=https://staging-hackathon1.vercel.app
   CORS_ORIGINS=https://staging-hackathon1.vercel.app,http://localhost:3000
   ```

   **Authentication (Phase 4A):**
   ```
   JWT_ALGORITHM=RS256
   JWT_ACCESS_TOKEN_EXPIRE_MINUTES=1440
   JWT_REFRESH_TOKEN_EXPIRE_DAYS=30
   CSRF_SECRET_KEY=your-staging-csrf-secret-32-chars
   ```

   **JWT Keys (multiline - use "Raw Editor"):**
   - Click **"Raw Editor"** button
   - Add `JWT_PRIVATE_KEY` and `JWT_PUBLIC_KEY` with proper formatting:
   ```
   JWT_PRIVATE_KEY="-----BEGIN PRIVATE KEY-----
   YOUR_STAGING_PRIVATE_KEY_HERE
   -----END PRIVATE KEY-----"
   ```

   **OAuth Configuration:**
   ```
   GOOGLE_CLIENT_ID=your-staging-client-id.apps.googleusercontent.com
   GOOGLE_CLIENT_SECRET=GOCSPX-your-staging-client-secret
   GOOGLE_REDIRECT_URI=https://hackathon1-staging.up.railway.app/auth/google/callback

   GITHUB_CLIENT_ID=your-staging-github-client-id
   GITHUB_CLIENT_SECRET=your-staging-github-client-secret
   GITHUB_REDIRECT_URI=https://hackathon1-staging.up.railway.app/auth/github/callback
   ```

   **Rate Limiting:**
   ```
   RATE_LIMIT_ANONYMOUS=20
   RATE_LIMIT_AUTHENTICATED=100
   ```

   **RAG & Personalization:**
   ```
   CONFIDENCE_THRESHOLD=0.2
   TOP_K_CHUNKS=5
   MAX_CONVERSATION_HISTORY=5
   SESSION_RETENTION_DAYS=7
   MAX_CONTEXT_TOKENS=5000
   VECTOR_WEIGHT=0.7
   BM25_WEIGHT=0.3
   EMBEDDING_CACHE_TTL=3600
   QUERY_CACHE_TTL=300
   RECOMMENDATION_CACHE_TTL=1800
   RECOMMENDATION_CACHE_MAX_SIZE=500
   ```

3. **Verify Variables**
   - Click **"Save"** or variables auto-save
   - Review all variables in list view
   - Ensure no placeholders remain (`your-*`, `<staging-*>`)

### Method 2: Via Railway CLI

```bash
# Switch to staging environment
railway environment staging

# Set variables from .env.staging file
railway variables --set-from-file backend/.env.staging

# Or set individual variables
railway variables set DATABASE_URL="postgresql://..."
railway variables set OPENAI_API_KEY="sk-proj-..."

# Verify variables
railway variables
```

### Method 3: Bulk Import from .env.staging

1. **Copy Variables**
   - Open `backend/.env.staging`
   - Copy all non-comment lines

2. **Import in Railway**
   - Click **"Raw Editor"** in Variables tab
   - Paste all variables
   - Click **"Save"**

3. **Update Placeholders**
   - Replace all `your-*` placeholders with actual credentials
   - Replace URLs with actual staging endpoints

---

## Step 4: Configure Custom Domain (Optional)

### Using Railway's Provided Domain

1. **Generate Domain**
   - In Railway service settings
   - Go to **Settings** → **Networking**
   - Click **"Generate Domain"**
   - Railway provides: `hackathon1-staging.up.railway.app`

2. **Update Environment Variables**
   - Update `GOOGLE_REDIRECT_URI` with new domain
   - Update `GITHUB_REDIRECT_URI` with new domain
   - Update OAuth app settings in Google/GitHub consoles

### Using Custom Domain (Advanced)

1. **Add Custom Domain**
   - In Railway: **Settings** → **Networking** → **Custom Domain**
   - Enter: `staging-api.yourdomain.com`
   - Railway provides CNAME record

2. **Configure DNS**
   - In your DNS provider (Cloudflare, Route53, etc.)
   - Add CNAME record pointing to Railway's target

3. **Update Variables**
   - Update all `*_REDIRECT_URI` variables
   - Update `FRONTEND_URL` if using subdomain

---

## Step 5: Deploy to Staging

### Manual Deployment (Recommended)

1. **Trigger Deployment**
   - In Railway dashboard
   - Go to **Deployments** tab
   - Click **"Deploy"** button
   - Select branch: `1-personalization-engine`
   - Click **"Deploy Now"**

2. **Monitor Build**
   - Watch build logs in real-time
   - Verify dependencies install successfully
   - Check for any build errors

3. **Verify Health**
   - Once deployed, Railway shows service status as "Active"
   - Visit: `https://hackathon1-staging.up.railway.app/health`
   - Expected response: `{"status": "ok"}`

### Auto-Deploy from Branch

If auto-deploy is enabled:

```bash
# Make changes on feature branch
git add .
git commit -m "feat: add personalization endpoints"
git push origin 1-personalization-engine

# Railway automatically detects push and deploys
```

Monitor deployment in Railway dashboard.

---

## Step 6: Verify Deployment

### Health Check

```bash
# Simple health check
curl https://hackathon1-staging.up.railway.app/health

# Expected response:
# {"status":"ok"}

# Detailed health check
curl https://hackathon1-staging.up.railway.app/api/v1/health

# Expected response includes database, Qdrant, OpenAI status
```

### API Documentation

Visit: `https://hackathon1-staging.up.railway.app/api/docs`

**Note**: API docs are disabled in production but should be enabled for staging (set `DEBUG=true` temporarily if needed).

### Logs Monitoring

1. **View Real-time Logs**
   - Railway dashboard → **Logs** tab
   - Watch for startup messages
   - Verify no errors on boot

2. **Check Specific Errors**
   - Filter logs by level: `ERROR`, `WARNING`
   - Look for database connection issues
   - Check for missing environment variables

---

## Step 7: Configure Staging Database Access

### Allow Railway IP Addresses in Neon

1. **Get Railway Outbound IPs**
   - Railway uses dynamic IPs
   - Neon allows all connections by default with SSL

2. **Verify SSL Mode**
   - Ensure `DATABASE_URL` has `?sslmode=require` suffix
   - Neon enforces SSL for all connections

### Test Database Connection

```bash
# SSH into Railway container (if enabled)
railway run bash

# Or use Railway CLI locally
railway run python -c "
import psycopg2
conn = psycopg2.connect('$DATABASE_URL')
print('Database connected successfully!')
conn.close()
"
```

---

## Step 8: Set Up Monitoring & Alerts

### Railway Built-in Monitoring

1. **Resource Usage**
   - Railway dashboard → **Metrics** tab
   - Monitor CPU, Memory, Network usage
   - Set up alerts for high resource usage

2. **Deployment Alerts**
   - Settings → **Notifications**
   - Enable email/Slack alerts for:
     - Failed deployments
     - Service crashes
     - High error rates

### External Monitoring (Optional)

**Uptime Monitoring:**
- Use UptimeRobot or Better Uptime
- Monitor endpoint: `https://hackathon1-staging.up.railway.app/health`
- Check interval: 5 minutes
- Alert on downtime

**Log Aggregation:**
- Use Sentry for error tracking
- Configure in FastAPI app:
```python
import sentry_sdk
sentry_sdk.init(dsn="your-staging-sentry-dsn", environment="staging")
```

---

## Step 9: Security Hardening

### Environment-Specific Secrets

- ✅ Use different JWT keys for staging vs production
- ✅ Use different OAuth app credentials for staging
- ✅ Use different CSRF secrets
- ✅ Rotate all secrets every 6 months

### Access Control

1. **Limit Railway Access**
   - Only grant staging environment access to QA/UAT team
   - Production access limited to DevOps team

2. **Database Credentials**
   - Use Neon's role-based access control
   - Create read-only users for analysts

### CORS Restrictions

- Only allow staging frontend domains in `CORS_ORIGINS`
- Never use wildcards (`*`) in staging or production

---

## Troubleshooting

### Deployment Fails with "Module not found"

**Cause**: Missing dependencies in `requirements.txt`

**Fix**:
```bash
cd backend
pip freeze > requirements.txt
git add requirements.txt
git commit -m "fix: update requirements.txt"
git push
```

### Database Connection Timeout

**Cause**: Invalid `DATABASE_URL` or Neon database not accessible

**Fix**:
1. Verify `DATABASE_URL` format in Railway variables
2. Ensure `?sslmode=require` suffix is present
3. Test connection locally: `psql $DATABASE_URL`

### OAuth Redirect URI Mismatch

**Cause**: Railway domain changed or OAuth app not updated

**Fix**:
1. Get current Railway domain from dashboard
2. Update Google OAuth app: https://console.cloud.google.com/apis/credentials
3. Update GitHub OAuth app: https://github.com/settings/developers
4. Update `GOOGLE_REDIRECT_URI` and `GITHUB_REDIRECT_URI` in Railway variables

### 500 Internal Server Error on /health

**Cause**: Missing environment variables or startup failure

**Fix**:
1. Check Railway logs for specific error
2. Verify all required variables are set
3. Test locally: `railway run uvicorn app.main:app`

---

## Rollback Procedure

### Rollback to Previous Deployment

1. **Via Railway Dashboard**
   - Go to **Deployments** tab
   - Find last successful deployment
   - Click **"Redeploy"** button

2. **Via Railway CLI**
```bash
railway rollback
```

### Rollback Database Migration

See: `NEON_STAGING_DATABASE_SETUP.md` → Migration Rollback section

---

## Post-Deployment Checklist

After staging deployment is complete:

- ✅ Health endpoint responding: `/health`
- ✅ API docs accessible: `/api/docs`
- ✅ Database connection successful
- ✅ Qdrant vector search working
- ✅ OpenAI API integration functional
- ✅ OAuth login flows working (Google + GitHub)
- ✅ CORS allowing staging frontend
- ✅ Logs show no critical errors
- ✅ All Phase 4B endpoints deployed:
  - ✅ `GET /api/v1/skill-level`
  - ✅ `GET /api/v1/recommendations`
  - ✅ `POST /api/v1/progress/start`
  - ✅ `POST /api/v1/progress/complete`

Proceed to: **Smoke Testing** (see `SMOKE_TEST_CHECKLIST.md`)

---

## Next Steps

1. ✅ **Complete**: Railway staging environment setup
2. ⏭️ **Next**: Run database migration (see `NEON_STAGING_DATABASE_SETUP.md`)
3. ⏭️ **Next**: Deploy frontend to staging (see `FRONTEND_STAGING_DEPLOYMENT.md`)
4. ⏭️ **Next**: Execute smoke tests (see `SMOKE_TEST_CHECKLIST.md`)
5. ⏭️ **Next**: Begin UAT (see `UAT_TEST_PLAN.md`)

---

**Document Version**: 1.0
**Last Updated**: 2025-12-23
**Maintained By**: DevOps Team
