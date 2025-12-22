# Railway Deployment Guide

## Issue: Healthcheck Failures

If you're seeing healthcheck failures like:
```
Attempt #1 failed with service unavailable. Continuing to retry...
1/1 replicas never became healthy!
```

This means the application is crashing during startup before it can respond to health checks.

## Root Causes

1. **Missing Environment Variables** - Railway needs all required env vars configured
2. **Database Connection Issues** - Migrations run on startup and need valid DATABASE_URL
3. **Config File Issues** - Fixed: .env file is now optional for production

## Required Environment Variables

Configure these in Railway Dashboard → Your Project → Variables:

### Critical (Required)
```bash
# Database (from Neon PostgreSQL)
DATABASE_URL=postgres://user:password@ep-xxx.neon.tech/dbname?sslmode=require

# OpenAI API
OPENAI_API_KEY=sk-your-openai-api-key-here

# Qdrant Vector Database
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_COLLECTION=robotics_textbook
```

### Application Settings
```bash
# Environment
ENVIRONMENT=production

# CORS - Add your GitHub Pages URL
CORS_ORIGINS=https://sajid-khan-afridi.github.io,https://hackathon1repeat-production.up.railway.app

# Rate Limiting (optional - has defaults)
RATE_LIMIT_ANONYMOUS=10
RATE_LIMIT_AUTHENTICATED=50

# RAG Configuration (optional - has defaults)
CONFIDENCE_THRESHOLD=0.2
TOP_K_CHUNKS=5
MAX_CONVERSATION_HISTORY=5
SESSION_RETENTION_DAYS=30
```

### Authentication (Optional - for OAuth features)
```bash
# JWT Configuration
JWT_PRIVATE_KEY=your-private-key-here
JWT_PUBLIC_KEY=your-public-key-here
JWT_ALGORITHM=RS256

# GitHub OAuth
GITHUB_CLIENT_ID=your-github-client-id
GITHUB_CLIENT_SECRET=your-github-client-secret
GITHUB_REDIRECT_URI=https://your-frontend-url.com/auth/callback/github

# Frontend URL
FRONTEND_URL=https://sajid-khan-afridi.github.io/hackathon1_repeat
```

## How to Fix

### Step 1: Check Environment Variables in Railway

1. Go to Railway Dashboard → Your Project → Variables
2. Ensure ALL required variables are set (DATABASE_URL, OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY)
3. Verify DATABASE_URL format: `postgres://user:password@host:port/dbname?sslmode=require`

### Step 2: Deploy the Fix

The code has been updated to:
- Make .env file optional (uses environment variables in production)
- Handle missing .env gracefully in migrations

Commit and push these changes:
```bash
git add backend/app/config.py backend/run_migrations.py backend/RAILWAY_DEPLOYMENT.md
git commit -m "fix(backend): make .env optional for Railway deployment"
git push
```

Railway will automatically redeploy.

### Step 3: Monitor Deployment Logs

Watch the Railway deployment logs for:

```
====================
Running database migrations...
====================
RAG Chatbot - Database Migration Runner
```

If you see:
- `ERROR: DATABASE_URL environment variable not set` → Set DATABASE_URL in Railway
- `Failed to connect to database` → Check DATABASE_URL format and credentials
- `Connecting to database...` → Good! Migrations are running
- `Starting uvicorn on port XXXX` → Success! Server is starting

### Step 4: Verify Health Check

Once deployed, test the health endpoints:

```bash
# Simple health check (Railway uses this)
curl https://your-app.up.railway.app/health

# Expected response:
{"status":"ok"}

# Detailed health check (checks all services)
curl https://your-app.up.railway.app/api/v1/health

# Expected response:
{
  "status": "healthy",
  "database": "healthy",
  "vector_store": "healthy",
  "llm": "healthy",
  "details": {...}
}
```

## Common Issues

### Issue: "DATABASE_URL environment variable not set"
**Solution**: Add DATABASE_URL to Railway environment variables

### Issue: "Failed to connect to database"
**Solutions**:
- Check DATABASE_URL format (must include `?sslmode=require` for Neon)
- Verify database credentials
- Ensure Neon database is accessible (not suspended)

### Issue: "Qdrant connection failed"
**Solutions**:
- Verify QDRANT_URL format: `https://xxx.qdrant.io` (no trailing slash)
- Check QDRANT_API_KEY is correct
- Ensure Qdrant cluster is running

### Issue: "OpenAI API call failed"
**Solutions**:
- Verify OPENAI_API_KEY starts with `sk-`
- Check API key has credits available
- Ensure no rate limits hit

## Health Check Configuration

Railway's health check:
- Path: `/health`
- Retry window: 1m40s
- Expected: HTTP 200 with any response

The app provides:
- `/health` - Simple check, always returns 200 OK
- `/api/v1/health` - Detailed check, tests all dependencies

## Startup Sequence

1. **Dockerfile** → Builds the image
2. **start.sh** → Runs on container start
   - Runs database migrations (`run_migrations.py`)
   - Starts uvicorn server
3. **Railway** → Checks `/health` endpoint
   - Must respond within 1m40s
   - 7 retry attempts

If any step fails, the deployment is marked unhealthy.

## Debugging

### View Real-time Logs
Railway Dashboard → Deployments → View Logs

### Look for:
```
# Good signs:
✅ "Running database migrations..."
✅ "Connected successfully"
✅ "All migrations completed successfully!"
✅ "Starting uvicorn on port XXXX"
✅ "Application startup complete"

# Bad signs:
❌ "ERROR: DATABASE_URL environment variable not set"
❌ "Failed to connect to database"
❌ "Migration failed"
❌ "Import Error"
❌ "Validation Error" (missing env vars)
```

## Next Steps After Successful Deployment

1. Update frontend API URL to point to Railway backend
2. Test chatbot functionality
3. Monitor logs for errors
4. Set up Railway metrics and alerts

## Support

If issues persist:
1. Check Railway logs for specific error messages
2. Verify all environment variables are set correctly
3. Test database connection manually
4. Ensure Qdrant collection exists and has data
