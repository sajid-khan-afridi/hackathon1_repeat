# 🚀 ONE-CLICK RAILWAY DEPLOYMENT

**Status**: ✅ Ready to deploy (GitHub OAuth already configured)

---

## Deploy to Railway (5 minutes)

### Option 1: Railway Dashboard (Recommended)

1. **Go to Railway**:
   ```
   https://railway.app/new
   ```

2. **Deploy from GitHub**:
   - Click "Deploy from GitHub repo"
   - Select: `hackathon1_repeat`
   - Branch: `1-personalization-engine`
   - Root directory: `backend`

3. **Configure Service**:
   - Service name: `hackathon1-backend-staging`
   - Environment: Create new → `staging`

4. **Add Environment Variables**:
   Click "Variables" → "Raw Editor" → Paste this:

```env
ENVIRONMENT=staging
DEBUG=false
LOG_LEVEL=INFO
LOG_FORMAT=json
API_V1_STR=/api/v1
DATABASE_URL=postgresql://neondb_owner:npg_WxAZ9pqHKYm1@ep-autumn-truth-a1650jur-pooler.ap-southeast-1.aws.neon.tech/hackathon1_staging?sslmode=require&channel_binding=require
DB_POOL_SIZE=5
DB_MAX_OVERFLOW=10
DB_POOL_TIMEOUT=30
QDRANT_URL=https://f143926d-3704-4829-ba33-fe1e3586ca47.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.awQ4TEDfm-AjaZJXBz_gwYF8rwbXbqadEYNERwhVKZE
QDRANT_COLLECTION=robotics_textbook_staging
OPENAI_API_KEY=<YOUR_OPENAI_API_KEY>
EMBEDDING_MODEL=text-embedding-3-small
CHAT_MODEL=gpt-4-turbo-preview
CORS_ORIGINS=http://localhost:3000,https://sajid-khan-afridi.github.io,https://staging-hackathon1.vercel.app
FRONTEND_URL=https://staging-hackathon1.vercel.app
JWT_PRIVATE_KEY=-----BEGIN PRIVATE KEY-----
MIIEvQIBADANBgkqhkiG9w0BAQEFAASCBKcwggSjAgEAAoIBAQCizyWLHQYX34/M
AZqWzBN2rpKjf/WF7eZcZmqZtNuEBOl0f+AcEln8snahmY8ZKcW9jkB/uMqGHHcw
esOGFFCTTSPPyrbcdNPxKWdJgm/ZgRynKIrlLBVencpFQljkBRlhlPcdw+UTZ3MC
Jh0kwq2nrpv8WvKZ9xJu/qH9fkfp1oEaQaYNqUdAlsPE4qreHwlgkAIBGZE6e6Sl
u6cujeZ29BbDApPQHnyB9e6FA1TBuvWE+Hqvep5Tu/Zot7k7ygaoAeoGEfTIM4uo
fR4TfMcZkNa+yu7k6WxBBXZgv2pLQItwlYm7234ULlsSpH3XkL5fIbSyhg6x4AsT
TzVDgGSfAgMBAAECggEAEUnh+Lsk8dVhqw44i5mG2kGA0i/ns0h8qoSNqV4qjq20
L2os3Y5CqVziJW2YY9mrD+Kn4r2NdAmty84dg9gVvllLW5NxMPPkVbr0fDqHr2x7
Ix0Lss5S7ps0PCqReeqOCW4M18enmb87LUKS3FvNSP3NE/0saevHX1YBfwOxQJlv
3lOaFs1m8sbiUtCudpBDq/P8JLY5xvJnjPrpyXviBEjHOvhtgXoVjxRihOXI6agx
91YJ5A2ax0z7UhvWvofxVF7zDRH7DvREeQnqdYEG5BQCPlaXyCW2HWFaMePvIrZ/
beak90X7WJuw7AJjtiasz6RsKv1HGtjU0Wf/9yo5UQKBgQDbu9cO1AziHtilkbtd
hDgsIGfX4MXSw5wfBi8rDUG48EA4gGqBWNCYzrASBtf2a0IeD7LKPpFKM60EHXP1
k9hWQTV0BiMRhp1/CLe2NBdgVbGp2sqWgcpYI0mos9oRVIshQEWp7PiUusrYydRX
Uu4T1JRxnYe7Uk/N+bDp5Ika0wKBgQC9riMQrLfSR2qyprIIfh7kQZ3Mw7aOLwNI
iJPpapXwKZUb8h7orn891IETWoaT/bSee5+lIeQZW7uRiJRj87iBbXu6AKGC96X2
KWHb8FYPTGjczxOdwuEltVUdbcrZWDPmwMbawmTDyDiPqSRTOpwIsxFzRIQGHrOK
SoMx0uGXhQKBgQCA8VegEfuFbciAUGZ82CwGaPXHDtXHepZQ9lYTk3HvzrxA68qU
IMQ4B4H28iElL4fbECdA2zpzKqPq+EWl5J5c72MotLk7fZ1KgJkdyK+3DlJ1dDjR
3VQaKIr+/puG8++5EXhP4Ql2ba6/TKDYpYekB2kcyu5tlD3UhyOz/TIflQKBgDQG
56dTOLh8zk9iWcLqlR6WwOnBZPlq+8ORcsC7c5UEUtnvd3rrIgublivw0KkXFcvC
rO9mdGJgPvqUM9+Woi0eEqnaMnLx6puyjhz9tx1LIBgw1HiZU5g8PLRtwQ4Mm2/X
wLOQToNFD/6XrlcF5HaeZ1LuoCftk+GCvqmT5jzhAoGAWxUyaXu77axdxUpnibAI
lprxMxYfa7g9eMbHbwC6bYqGGV+T2Uc4nVGHLyQDwwo+yM/zLhMc7FxCwFI1zRhK
zdn35ThQ09bBPY+VgECXMN+gag9MhpXxidfmUX8QmJZ3pSVYXwf3rqzExH78M5pS
XqT+S2CRWGu1+kB9ou5tNU4=
-----END PRIVATE KEY-----
JWT_PUBLIC_KEY=-----BEGIN PUBLIC KEY-----
MIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAos8lix0GF9+PzAGalswT
dq6So3/1he3mXGZqmbTbhATpdH/gHBJZ/LJ2oZmPGSnFvY5Af7jKhhx3MHrDhhRQ
k00jz8q23HTT8SlnSYJv2YEcpyiK5SwVXp3KRUJY5AUZYZT3HcPlE2dzAiYdJMKt
p66b/FrymfcSbv6h/X5H6daBGkGmDalHQJbDxOKq3h8JYJACARmROnukpbunLo3m
dvQWwwKT0B58gfXuhQNUwbr1hPh6r3qeU7v2aLe5O8oGqAHqBhH0yDOLqH0eE3zH
GZDWvsru5OlsQQV2YL9qS0CLcJWJu9t+FC5bEqR915C+XyG0soYOseALE081Q4Bk
nwIDAQAB
-----END PUBLIC KEY-----
JWT_ALGORITHM=RS256
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=1440
JWT_REFRESH_TOKEN_EXPIRE_DAYS=30
CSRF_SECRET_KEY=dGNBWMQZtkcYIfnnwFbYly0OtZCAAhlqmVOF3-Zbt9s
GOOGLE_CLIENT_ID=your-staging-client-id.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=GOCSPX-your-staging-client-secret
GOOGLE_REDIRECT_URI=https://hackathon1-staging.up.railway.app/auth/google/callback
GITHUB_CLIENT_ID=your-staging-github-client-id
GITHUB_CLIENT_SECRET=your-staging-github-client-secret
GITHUB_REDIRECT_URI=https://hackathon1-staging.up.railway.app/auth/github/callback
RATE_LIMIT_ANONYMOUS=20
RATE_LIMIT_AUTHENTICATED=100
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
DEPLOYMENT_VERSION=phase-4b-personalization
```

5. **Deploy**:
   - Click "Deploy"
   - Wait 2-3 minutes for build
   - Get your URL: `https://hackathon1-staging.up.railway.app`

6. **Verify**:
   ```bash
   curl https://your-app.up.railway.app/health
   ```

---

## After Deployment

Update OAuth redirect URIs with actual Railway URL:
1. Replace `hackathon1-staging.up.railway.app` with your actual Railway domain
2. Update Google/GitHub OAuth apps with new callback URLs
3. Update Railway environment variables with actual OAuth credentials

---

**Next**: Deploy → Get URL → Run smoke tests
