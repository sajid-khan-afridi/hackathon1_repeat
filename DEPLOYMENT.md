# Railway Deployment Guide

This guide walks you through deploying the RAG Chatbot to Railway.

## Prerequisites

- Railway account (sign up at https://railway.app)
- GitHub repository connected
- Environment variables ready (see `.env.example`)

## Backend Deployment to Railway

### Step 1: Create New Project

1. Go to https://railway.app/new
2. Click "Deploy from GitHub repo"
3. Select this repository: `hackathon1_repeat`
4. Railway will auto-detect the Python backend

### Step 2: Configure Service

Railway will automatically detect the backend using:
- `backend/railway.json` - Railway configuration
- `backend/nixpacks.toml` - Build configuration
- `backend/Procfile` - Start command

**Root Directory**: Set to `backend/` in Railway settings

### Step 3: Add Environment Variables

In Railway dashboard, add these environment variables:

#### Required Variables

```bash
# OpenAI Configuration
OPENAI_API_KEY=sk-proj-xxxxxxxxxxxxx

# Qdrant Vector Database
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION=robotics_textbook

# Neon PostgreSQL Database
DATABASE_URL=postgresql://user:pass@your-db.neon.tech/dbname?sslmode=require

# API Configuration
ENVIRONMENT=production
API_HOST=0.0.0.0
API_PORT=$PORT  # Railway provides this automatically
CORS_ORIGINS=https://your-frontend-domain.vercel.app,https://your-docs-site.com
```

#### Optional Variables (with defaults)

```bash
# Rate Limiting
RATE_LIMIT_ANONYMOUS=10
RATE_LIMIT_AUTHENTICATED=50

# RAG Configuration
CONFIDENCE_THRESHOLD=0.2
TOP_K_CHUNKS=5
MAX_CONVERSATION_HISTORY=5
SESSION_RETENTION_DAYS=30

# Logging
LOG_LEVEL=INFO
LOG_FORMAT=json
```

### Step 4: Deploy

1. Railway will automatically deploy when you push to `main` branch
2. Monitor deployment logs in Railway dashboard
3. Wait for health check to pass at `/api/v1/health`
4. Copy the generated Railway URL (e.g., `https://your-app.up.railway.app`)

### Step 5: Verify Deployment

Test the deployed backend:

```bash
# Health check
curl https://your-app.up.railway.app/api/v1/health

# Query endpoint
curl -X POST https://your-app.up.railway.app/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS?", "top_k": 3}'
```

Expected response: 200 OK with health status or query results

## Frontend Deployment Options

### Option A: Vercel (Recommended for Docusaurus)

1. Go to https://vercel.com/new
2. Import your GitHub repository
3. Configure build settings:
   - **Framework**: Docusaurus
   - **Root Directory**: `.` (project root)
   - **Build Command**: `npm run build`
   - **Output Directory**: `build`
4. Add environment variable:
   ```
   REACT_APP_API_URL=https://your-app.up.railway.app
   ```
5. Deploy

### Option B: GitHub Pages

1. Update `docusaurus.config.js`:
   ```js
   url: 'https://your-username.github.io',
   baseUrl: '/hackathon1_repeat/',
   ```
2. Run deployment:
   ```bash
   npm run deploy
   ```

## Post-Deployment Checklist

- [ ] Backend health check returns 200 OK
- [ ] Database connection working (check health endpoint)
- [ ] Qdrant vector store accessible
- [ ] OpenAI API responding
- [ ] CORS configured correctly for frontend domain
- [ ] Frontend can connect to backend API
- [ ] Test end-to-end query flow
- [ ] Monitor logs for errors
- [ ] Set up alerts for service failures

## Railway-Specific Features

### Custom Domain

1. Go to Railway project settings
2. Click "Add Domain"
3. Add your custom domain (e.g., `api.yoursite.com`)
4. Update DNS records as instructed
5. Update CORS_ORIGINS to include new domain

### Auto-Deployments

Railway automatically deploys when you push to `main` branch.

To disable:
1. Go to project settings
2. Toggle "Auto-Deploy" off

### Logs & Monitoring

View logs in real-time:
```bash
railway logs
```

Or in Railway dashboard: Project → Deployments → View Logs

### Database Backups

If using Railway PostgreSQL (alternative to Neon):
1. Railway provides automatic backups
2. Access via Railway dashboard → Database → Backups

## Scaling & Performance

### Vertical Scaling

Railway offers various instance sizes:
- **Hobby**: 512MB RAM, 1 vCPU (free tier)
- **Pro**: Up to 32GB RAM, 8 vCPU

Adjust in: Settings → Service → Resources

### Horizontal Scaling

For high traffic:
1. Enable Railway autoscaling (Pro plan)
2. Or use load balancer with multiple instances

## Troubleshooting

### Health Check Failing

Check logs for:
- Database connection errors
- Qdrant API key issues
- OpenAI API accessibility

```bash
railway logs --tail 100
```

### Port Binding Issues

Ensure your app binds to `0.0.0.0:$PORT`:
```python
# In app/main.py
uvicorn.run(app, host="0.0.0.0", port=int(os.getenv("PORT", 8000)))
```

### Build Failures

Check:
- `requirements.txt` is complete
- Python version compatibility (Railway uses Python 3.11)
- Nixpacks configuration in `nixpacks.toml`

### Environment Variable Issues

Verify all required variables are set:
```bash
railway variables
```

## Cost Estimation

### Railway Pricing (as of 2024)

- **Hobby Plan**: $5/month
  - 500 hours execution time
  - 512MB RAM, 1 vCPU
  - 100GB outbound network

- **Pro Plan**: Pay-as-you-go
  - $10/month base + usage
  - Higher resource limits

### External Services

- **Qdrant Cloud**: Free tier (1GB) or paid plans
- **Neon PostgreSQL**: Free tier (0.5GB) or paid plans
- **OpenAI API**: Pay-per-token (~$0.01-0.02 per 1000 requests)

**Estimated Monthly Cost**: $5-15/month for MVP traffic

## Security Best Practices

1. **Never commit `.env` files**
2. **Use Railway's secret management** for sensitive variables
3. **Enable HTTPS only** (Railway provides free SSL)
4. **Implement rate limiting** (re-enable when fixed)
5. **Monitor API usage** to prevent abuse
6. **Set up alerts** for unusual traffic patterns

## Production Readiness

Based on TEST_REPORT.md, the system is production-ready with:

✅ All integration tests passing (8/8)
✅ Health checks operational
✅ Error handling and graceful degradation
✅ Structured logging with correlation IDs
✅ Input sanitization (XSS prevention)

⚠️ **Known Limitation**: Rate limiting temporarily disabled (see TEST_REPORT.md)

**Recommended**: Deploy behind Cloudflare or nginx for rate limiting until fixed.

## Support

- Railway Docs: https://docs.railway.app
- Railway Discord: https://discord.gg/railway
- Project Issues: https://github.com/your-username/hackathon1_repeat/issues
