# RAG Chatbot MVP - Production Deployment Guide

## Overview

This guide walks you through deploying the RAG Chatbot backend to production using Railway or Render, and connecting it to the existing GitHub Pages frontend.

## Prerequisites

✅ **Completed** (Already configured in your project):
- PostgreSQL database on Neon (with schema applied)
- Qdrant Cloud vector store (33 points indexed)
- OpenAI API key
- GitHub repository: https://github.com/sajid-khan-afridi/hackathon1_repeat

## Infrastructure Summary

### Current Status

| Component | Status | Details |
|-----------|--------|---------|
| **Database** | ✅ Ready | Neon PostgreSQL with tables: `chat_sessions`, `chat_messages`, `source_citations` |
| **Vector Store** | ✅ Ready | Qdrant Cloud collection `robotics_textbook` with 33 vectors (1536 dimensions) |
| **OpenAI API** | ✅ Ready | API key configured |
| **Frontend** | ✅ Deployed | GitHub Pages at https://sajid-khan-afridi.github.io/hackathon1_repeat/ |
| **Backend** | ⏳ Pending | Needs deployment to Railway or Render |

## Deployment Options

### Option A: Railway (Recommended - Simplest)

**Pros:**
- GitHub integration with auto-deploy on push
- Free tier with $5/month credit
- Automatic HTTPS and domains
- Built-in metrics and logs
- Easy environment variable management

**Cons:**
- Limited free tier hours
- Requires credit card after trial

#### Steps:

1. **Sign up and create project**
   - Go to https://railway.app
   - Sign in with GitHub
   - Click "New Project" → "Deploy from GitHub repo"
   - Select `sajid-khan-afridi/hackathon1_repeat`

2. **Configure build settings**
   - Root directory: `backend`
   - Builder: Dockerfile
   - Railway will auto-detect `backend/Dockerfile`

3. **Set environment variables**

   Click "Variables" and add these:

   ```bash
   ENVIRONMENT=production

   # Copy from your .env file:
   OPENAI_API_KEY=sk-proj-YOUR_OPENAI_API_KEY_HERE

   QDRANT_URL=https://YOUR_QDRANT_CLUSTER_ID.REGION.gcp.cloud.qdrant.io
   QDRANT_API_KEY=YOUR_QDRANT_API_KEY_HERE
   QDRANT_COLLECTION=robotics_textbook

   DATABASE_URL=postgresql://username:password@your-neon-host.region.aws.neon.tech/database?sslmode=require

   # Update CORS with your GitHub Pages URL
   CORS_ORIGINS=https://sajid-khan-afridi.github.io,http://localhost:3000

   # Rate limiting
   RATE_LIMIT_ANONYMOUS=10
   RATE_LIMIT_AUTHENTICATED=50

   # RAG settings
   CONFIDENCE_THRESHOLD=0.2
   TOP_K_CHUNKS=5
   MAX_CONVERSATION_HISTORY=5
   SESSION_RETENTION_DAYS=30
   ```

4. **Deploy**
   - Click "Deploy"
   - Railway will build the Docker image and deploy
   - Wait 3-5 minutes for first deployment
   - You'll get a URL like: `https://your-app.up.railway.app`

5. **Verify deployment**
   ```bash
   # Test health endpoint
   curl https://your-app.up.railway.app/api/v1/health

   # Should return:
   # {
   #   "status": "healthy",
   #   "database": "healthy",
   #   "vector_store": "healthy",
   #   "llm": "healthy"
   # }
   ```

6. **Update CORS** (if needed)
   - After getting your Railway URL, add it to `CORS_ORIGINS`:
   - `CORS_ORIGINS=https://sajid-khan-afridi.github.io,https://your-app.up.railway.app`

---

### Option B: Render

**Pros:**
- Generous free tier (750 hours/month)
- No credit card required for free tier
- Auto-deploy from GitHub
- Built-in PostgreSQL option

**Cons:**
- Free tier instances spin down after 15 min inactivity (cold starts)
- Slower build times

#### Steps:

1. **Sign up and create web service**
   - Go to https://render.com
   - Sign in with GitHub
   - Click "New +" → "Web Service"
   - Connect repository: `sajid-khan-afridi/hackathon1_repeat`

2. **Configure service**
   - Name: `rag-chatbot-backend`
   - Region: Choose closest to your users
   - Branch: `main`
   - Root Directory: `backend`
   - Runtime: Python 3
   - Build Command: `pip install -r requirements.txt`
   - Start Command: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

3. **Set environment variables**

   Click "Environment" and add the same variables as Railway (see above)

4. **Deploy**
   - Click "Create Web Service"
   - First build takes 5-10 minutes
   - You'll get a URL like: `https://rag-chatbot-backend.onrender.com`

5. **Verify deployment**
   ```bash
   curl https://rag-chatbot-backend.onrender.com/api/v1/health
   ```

---

### Option C: Local Docker (Development/Testing)

For local testing with Docker:

```bash
# From project root
cd backend

# Build image
docker build -t rag-chatbot-backend .

# Run container (using .env file)
docker run -p 8000:8000 --env-file ../.env rag-chatbot-backend

# Or run with explicit environment variables
docker run -p 8000:8000 \
  -e ENVIRONMENT=production \
  -e OPENAI_API_KEY=your-key \
  -e QDRANT_URL=your-url \
  -e QDRANT_API_KEY=your-key \
  -e DATABASE_URL=your-db-url \
  -e CORS_ORIGINS=http://localhost:3000 \
  rag-chatbot-backend

# Test
curl http://localhost:8000/api/v1/health
```

---

## Frontend Integration

After deploying the backend, update your frontend to use the production API:

### Option 1: Update Docusaurus Configuration

Edit `docusaurus.config.js`:

```javascript
module.exports = {
  // ... existing config
  customFields: {
    chatbotApiUrl: 'https://your-app.up.railway.app', // Your deployed backend URL
  },
};
```

Then in your ChatbotWidget, use:

```typescript
const API_BASE_URL = typeof window !== 'undefined'
  ? (window as any).CHATBOT_API_URL ||
    (window as any).docusaurus?.siteConfig?.customFields?.chatbotApiUrl ||
    'http://localhost:8000'
  : 'http://localhost:8000';
```

### Option 2: Inject via GitHub Pages HTML

Add to your `static/index.html` or create a custom HTML template:

```html
<script>
  window.CHATBOT_API_URL = 'https://your-app.up.railway.app';
</script>
```

### Option 3: Environment Variable at Build Time

Create `.env.production` in your frontend:

```bash
REACT_APP_API_URL=https://your-app.up.railway.app
```

Then use in code:

```typescript
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';
```

---

## Post-Deployment Testing

### 1. Health Check

```bash
# Replace with your actual URL
export BACKEND_URL="https://your-app.up.railway.app"

curl -X GET "$BACKEND_URL/api/v1/health"

# Expected response:
# {
#   "status": "healthy",
#   "database": "healthy",
#   "vector_store": "healthy",
#   "llm": "healthy",
#   "details": {
#     "database": "Connected successfully",
#     "vector_store": "Collection 'robotics_textbook' found",
#     "llm": "OpenAI API accessible"
#   }
# }
```

### 2. Test Query Endpoint

```bash
curl -X POST "$BACKEND_URL/api/v1/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is a robot?",
    "top_k": 3
  }'

# Expected response:
# {
#   "answer": "...",
#   "sources": [...],
#   "confidence": 0.85,
#   "session_id": "...",
#   "tokens_used": {...}
# }
```

### 3. Test from Frontend

1. Open your GitHub Pages site: https://sajid-khan-afridi.github.io/hackathon1_repeat/
2. Navigate to a page with the chatbot widget
3. Ask a question: "What is a robot?"
4. Verify:
   - Response appears within 3-5 seconds
   - Source citations are displayed
   - Confidence indicator shows
   - No CORS errors in browser console

### 4. Monitor Logs

**Railway:**
- Go to your project → "Deployments" → Click latest deployment
- View real-time logs

**Render:**
- Go to your service → "Logs" tab
- Filter by severity if needed

---

## Troubleshooting

### CORS Errors

**Symptom:** Browser console shows:
```
Access to fetch at 'https://backend.com/api/v1/query' from origin 'https://yoursite.github.io'
has been blocked by CORS policy
```

**Solution:**
1. Add your frontend URL to `CORS_ORIGINS` environment variable
2. Redeploy backend
3. Clear browser cache

### Health Check Fails

**Symptom:** `/api/v1/health` returns 503 or times out

**Solution:**
1. Check environment variables are set correctly
2. Verify database connectivity:
   ```bash
   # From your backend container/logs
   python backend/check_db_status.py
   ```
3. Verify Qdrant connectivity:
   ```bash
   python backend/test_qdrant.py
   ```

### Cold Start Delays (Render Free Tier)

**Symptom:** First request after 15 minutes takes 30+ seconds

**Solution:**
- This is expected on Render's free tier
- Upgrade to paid tier for always-on instances
- Or use Railway which has better cold start performance

### Database Connection Pool Exhausted

**Symptom:** Logs show "connection pool exhausted"

**Solution:**
- Increase `max_size` in `backend/app/services/chat_service.py`:
  ```python
  self.pool = await asyncpg.create_pool(
      dsn=settings.database_url,
      min_size=2,
      max_size=20,  # Increase from 10 to 20
      command_timeout=60,
  )
  ```

---

## Monitoring and Maintenance

### Metrics to Track

1. **Health Check Uptime**
   - Set up monitoring (e.g., UptimeRobot, Better Uptime)
   - Alert on health check failures

2. **Response Times**
   - Railway/Render provide built-in metrics
   - Aim for p95 < 3 seconds

3. **Error Rates**
   - Monitor logs for 500 errors
   - Set up Sentry or similar for error tracking

4. **Database Sessions**
   - Query Neon metrics for connection count
   - Purge old sessions regularly

### Scheduled Maintenance

1. **Purge old chat sessions (every week)**
   ```sql
   -- Run in Neon SQL editor
   SELECT purge_old_sessions();
   ```

2. **Refresh session statistics**
   ```sql
   REFRESH MATERIALIZED VIEW CONCURRENTLY session_stats;
   ```

3. **Check Qdrant collection health**
   ```bash
   python backend/test_qdrant.py
   ```

---

## Cost Estimates

### Current Setup (Free Tier)

| Service | Plan | Cost | Notes |
|---------|------|------|-------|
| Neon PostgreSQL | Free | $0/month | 0.5 GB storage, 1 compute unit |
| Qdrant Cloud | Free | $0/month | 1 GB RAM cluster |
| OpenAI API | Pay-as-you-go | ~$5-10/month | Based on usage |
| Railway | Free trial | $0 (then $5/month) | 500 hours/month, then $5 credit |
| Render | Free | $0/month | 750 hours/month |
| GitHub Pages | Free | $0/month | Unlimited bandwidth |

**Total: ~$5-10/month** (mostly OpenAI usage)

### Scaling Costs (1000 queries/day)

| Service | Plan | Cost |
|---------|------|------|
| Neon PostgreSQL | Pro | $19/month |
| Qdrant Cloud | 2GB cluster | $25/month |
| OpenAI API | Pay-as-you-go | ~$50-100/month |
| Railway | Starter | $20/month |
| **Total** | | **~$114-164/month** |

---

## Security Checklist

- [ ] All API keys in environment variables (not committed to git)
- [ ] CORS configured with specific origins (not `*`)
- [ ] Rate limiting enabled (10/hour anonymous, 50/hour authenticated)
- [ ] Database uses SSL (`sslmode=require` in connection string)
- [ ] Qdrant uses API key authentication
- [ ] Non-root user in Docker container
- [ ] Health check doesn't expose sensitive data
- [ ] Logs don't contain API keys or tokens

---

## Rollback Procedure

If deployment fails or causes issues:

1. **Railway:**
   - Go to "Deployments"
   - Click previous successful deployment
   - Click "Redeploy"

2. **Render:**
   - Go to "Manual Deploy"
   - Select previous commit
   - Click "Deploy"

3. **Verify rollback:**
   ```bash
   curl $BACKEND_URL/api/v1/health
   ```

4. **Check logs** for root cause

---

## Next Steps After Deployment

1. **Set up custom domain** (optional)
   - Railway: Settings → Domains → Add custom domain
   - Render: Settings → Custom Domains

2. **Enable monitoring**
   - UptimeRobot for health checks
   - Sentry for error tracking
   - LogRocket for session replay

3. **Performance optimization**
   - Enable Redis caching for vector search results
   - Implement query result caching
   - Optimize database indexes

4. **Add authentication** (future)
   - Implement user authentication
   - Link `user_id` in `chat_sessions` table
   - Increase rate limits for authenticated users

---

## Support and Resources

- **Railway Docs:** https://docs.railway.app
- **Render Docs:** https://render.com/docs
- **Neon Docs:** https://neon.tech/docs
- **Qdrant Docs:** https://qdrant.tech/documentation/
- **FastAPI Docs:** https://fastapi.tiangolo.com

---

## Deployment Checklist

Use this checklist to track your deployment progress:

### Backend Deployment
- [ ] Choose platform (Railway or Render)
- [ ] Create account and link GitHub repository
- [ ] Configure build settings (Dockerfile, root directory)
- [ ] Set all environment variables
- [ ] Deploy and wait for build to complete
- [ ] Test health endpoint
- [ ] Test query endpoint
- [ ] Check logs for errors

### Frontend Integration
- [ ] Get backend URL from deployment
- [ ] Update frontend configuration with backend URL
- [ ] Update CORS_ORIGINS on backend with GitHub Pages URL
- [ ] Redeploy backend with new CORS settings
- [ ] Test chatbot from GitHub Pages site
- [ ] Verify no CORS errors in browser console

### Post-Deployment
- [ ] Set up uptime monitoring
- [ ] Configure error tracking
- [ ] Document backend URL in team docs
- [ ] Schedule database maintenance tasks
- [ ] Monitor OpenAI API usage and costs

---

**Deployment Date:** 2025-12-15
**Backend URL:** _To be filled after deployment_
**Frontend URL:** https://sajid-khan-afridi.github.io/hackathon1_repeat/
**Status:** ✅ Ready to deploy
