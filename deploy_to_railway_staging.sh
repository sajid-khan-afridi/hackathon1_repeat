#!/bin/bash
# =============================================================================
# Railway Staging Deployment Script - Auto-generated
# =============================================================================
# Usage: bash deploy_to_railway_staging.sh
# Prerequisites: Railway CLI installed and authenticated
# =============================================================================

set -e  # Exit on error

echo '============================================================'
echo 'Railway Staging Deployment - Phase 4B'
echo '============================================================'

# Step 1: Link to Railway project
echo '[*] Linking to Railway project...'
railway link

# Step 2: Switch to staging environment
echo '[*] Switching to staging environment...'
railway environment staging

# Step 3: Set environment variables
echo '[*] Setting environment variables...'
railway variables set API_V1_STR="/api/v1"
railway variables set BM25_WEIGHT="0.3"
railway variables set CHAT_MODEL="gpt-4-turbo-preview"
railway variables set CONFIDENCE_THRESHOLD="0.2"
railway variables set CORS_ORIGINS="http://localhost:3000,https://sajid-khan-afridi.github.io,https://staging-hackathon1.vercel.app"
railway variables set CSRF_SECRET_KEY="your-staging-csrf-secret-32-chars"
railway variables set DATABASE_URL="postgresql://neondb_owner:npg_WxAZ9pqHKYm1@ep-autumn-truth-a1650jur-pooler.ap-southeast-1.aws.neon.tech/hackathon1_staging?sslmode=require&channel_binding=require"
railway variables set DB_MAX_OVERFLOW="10"
railway variables set DB_POOL_SIZE="5"
railway variables set DB_POOL_TIMEOUT="30"
railway variables set DEBUG="false"
railway variables set DEPLOYMENT_VERSION="phase-4b-personalization"
railway variables set EMBEDDING_CACHE_TTL="3600"
railway variables set EMBEDDING_MODEL="text-embedding-3-small"
railway variables set ENVIRONMENT="staging"
railway variables set FRONTEND_URL="https://staging-hackathon1.vercel.app"
railway variables set GITHUB_CLIENT_ID="your-staging-github-client-id"
railway variables set GITHUB_CLIENT_SECRET="your-staging-github-client-secret"
railway variables set GITHUB_REDIRECT_URI="https://hackathon1-staging.up.railway.app/auth/github/callback"
railway variables set GOOGLE_CLIENT_ID="your-staging-client-id.apps.googleusercontent.com"
railway variables set GOOGLE_CLIENT_SECRET="GOCSPX-your-staging-client-secret"
railway variables set GOOGLE_REDIRECT_URI="https://hackathon1-staging.up.railway.app/auth/google/callback"
railway variables set JWT_ACCESS_TOKEN_EXPIRE_MINUTES="1440"
railway variables set JWT_ALGORITHM="RS256"
railway variables set JWT_PRIVATE_KEY="-----BEGIN PRIVATE KEY-----\nYOUR_STAGING_PRIVATE_KEY_HERE\n-----END PRIVATE KEY-----"
railway variables set JWT_PUBLIC_KEY="-----BEGIN PUBLIC KEY-----\nYOUR_STAGING_PUBLIC_KEY_HERE\n-----END PUBLIC KEY-----"
railway variables set JWT_REFRESH_TOKEN_EXPIRE_DAYS="30"
railway variables set LOG_FORMAT="json"
railway variables set LOG_LEVEL="INFO"
railway variables set MAX_CONTEXT_TOKENS="5000"
railway variables set MAX_CONVERSATION_HISTORY="5"
railway variables set OPENAI_API_KEY="$OPENAI_API_KEY"  # Set from environment or copy from production
railway variables set QDRANT_API_KEY="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.awQ4TEDfm-AjaZJXBz_gwYF8rwbXbqadEYNERwhVKZE"
railway variables set QDRANT_COLLECTION="robotics_textbook_staging"
railway variables set QDRANT_URL="https://f143926d-3704-4829-ba33-fe1e3586ca47.europe-west3-0.gcp.cloud.qdrant.io"
railway variables set QUERY_CACHE_TTL="300"
railway variables set RATE_LIMIT_ANONYMOUS="20"
railway variables set RATE_LIMIT_AUTHENTICATED="100"
railway variables set RECOMMENDATION_CACHE_MAX_SIZE="500"
railway variables set RECOMMENDATION_CACHE_TTL="1800"
railway variables set SESSION_RETENTION_DAYS="7"
railway variables set TOP_K_CHUNKS="5"
railway variables set VECTOR_WEIGHT="0.7"

# Step 4: Deploy to Railway
echo '[*] Deploying to Railway staging...'
cd backend
railway up

echo '============================================================'
echo '[OK] Deployment Complete!'
echo '============================================================'
echo 'Next steps:'
echo '  1. Check deployment logs: railway logs'
echo '  2. Get deployment URL: railway status'
echo '  3. Test health endpoint: curl https://your-app.railway.app/health'
