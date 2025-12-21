# Production Deployment Guide - Authentication System

**Feature:** 001-user-auth
**Branch:** 001-user-auth
**Date:** 2025-12-20
**Environment:** Production

## Overview

This guide provides step-by-step instructions for deploying the authentication system to production with proper security configurations.

---

## Table of Contents

1. [Pre-Deployment Checklist](#pre-deployment-checklist)
2. [RSA Key Generation for JWT](#rsa-key-generation-for-jwt)
3. [OAuth Configuration](#oauth-configuration)
4. [CORS Configuration](#cors-configuration)
5. [Database Migrations](#database-migrations)
6. [Environment Variables](#environment-variables)
7. [Security Validation](#security-validation)
8. [Deployment Steps](#deployment-steps)
9. [Post-Deployment Verification](#post-deployment-verification)
10. [Rollback Procedures](#rollback-procedures)

---

## Pre-Deployment Checklist

Before deploying to production, ensure the following are completed:

- [ ] Production RSA keys generated and stored securely
- [ ] Separate GitHub OAuth app created for production
- [ ] Production CORS origins configured (no wildcards)
- [ ] All database migrations tested in staging
- [ ] Environment variables validated
- [ ] Security audit completed (OWASP checklist)
- [ ] Backup strategy confirmed
- [ ] Monitoring and alerting configured

---

## RSA Key Generation for JWT

### Step 1: Generate RSA Key Pair

Production requires a 2048-bit RSA key pair (4096-bit for high-security environments):

```bash
# Generate private key (2048-bit)
openssl genrsa -out jwt_private_production.pem 2048

# Generate public key from private key
openssl rsa -in jwt_private_production.pem -pubout -out jwt_public_production.pem
```

**Generated files (INCLUDED in this deployment):**
- `jwt_private_production.pem` - RSA private key (2048-bit)
- `jwt_public_production.pem` - RSA public key

### Step 2: Convert to Single-Line Format

Environment variables require single-line format with `\n` as newline separator.

**Windows PowerShell:**
```powershell
# Private key
$private = Get-Content jwt_private_production.pem -Raw
$private -replace "`r`n", "\n" -replace "`n", "\n"

# Public key
$public = Get-Content jwt_public_production.pem -Raw
$public -replace "`r`n", "\n" -replace "`n", "\n"
```

**Linux/Mac:**
```bash
# Private key
awk 'NF {sub(/\r/, ""); printf "%s\\n",$0;}' jwt_private_production.pem

# Public key
awk 'NF {sub(/\r/, ""); printf "%s\\n",$0;}' jwt_public_production.pem
```

### Step 3: Store in Secret Manager

**CRITICAL: Never commit production keys to version control!**

Store the single-line keys in your deployment platform's secret manager:

**Railway:**
```bash
railway variables set JWT_PRIVATE_KEY="-----BEGIN PRIVATE KEY-----\n...\n-----END PRIVATE KEY-----"
railway variables set JWT_PUBLIC_KEY="-----BEGIN PUBLIC KEY-----\n...\n-----END PUBLIC KEY-----"
```

**GitHub Secrets (for GitHub Actions):**
1. Go to repository Settings → Secrets and variables → Actions
2. Add secrets: `JWT_PRIVATE_KEY` and `JWT_PUBLIC_KEY`

### Security Best Practices

- ✅ Use separate key pairs for dev/staging/production
- ✅ Rotate keys every 6 months
- ✅ Store keys in encrypted secret management systems
- ✅ Use 2048-bit minimum (4096-bit for high security)
- ❌ NEVER commit keys to version control
- ❌ NEVER share keys via email or chat
- ❌ NEVER reuse development keys in production

---

## OAuth Configuration

### GitHub OAuth - Production Setup

**Step 1: Create Production OAuth App**

1. Go to https://github.com/settings/developers
2. Click "New OAuth App"
3. Fill in production details:
   - **Application name:** RAG Chatbot Production
   - **Homepage URL:** `https://your-production-domain.com`
   - **Authorization callback URL:** `https://your-api-domain.com/auth/github/callback`

**Step 2: Generate Client Secret**

1. After creating the app, click "Generate a new client secret"
2. Copy the secret immediately (shown only once)
3. Store in your secret manager

**Step 3: Configure Environment Variables**

```bash
# Railway
railway variables set GITHUB_CLIENT_ID="your-production-client-id"
railway variables set GITHUB_CLIENT_SECRET="your-production-client-secret"
railway variables set GITHUB_REDIRECT_URI="https://your-api-domain.com/auth/github/callback"
railway variables set FRONTEND_URL="https://your-production-domain.com"
```

### Production URLs to Update

Replace the following placeholder URLs with your actual production domains:

| Variable | Development | Production |
|----------|-------------|------------|
| `GITHUB_REDIRECT_URI` | `http://localhost:8000/auth/github/callback` | `https://your-api-domain.com/auth/github/callback` |
| `FRONTEND_URL` | `http://localhost:3000` | `https://your-production-domain.com` |
| `CORS_ORIGINS` | `http://localhost:3000,http://localhost:8000` | `https://your-production-domain.com,https://your-api-domain.com` |

### Google OAuth (Optional)

If using Google OAuth, follow similar steps:

1. Go to https://console.cloud.google.com/apis/credentials
2. Create new OAuth 2.0 Client ID (Web application)
3. Add authorized redirect URI: `https://your-api-domain.com/auth/google/callback`
4. Store credentials in secret manager

---

## CORS Configuration

### Development CORS

```python
CORS_ORIGINS = "http://localhost:3000,http://localhost:8000"
```

### Production CORS

**CRITICAL: Only include actual production domains!**

```python
# Example for Railway/Custom Domain deployment
CORS_ORIGINS = "https://your-frontend.com,https://www.your-frontend.com,https://your-api.com"

# Example for GitHub Pages frontend + Railway backend
CORS_ORIGINS = "https://yourusername.github.io,https://your-railway-app.railway.app"
```

### Security Requirements

- ✅ Use HTTPS for all production origins
- ✅ List exact domains (no wildcards)
- ✅ Include both www and non-www if needed
- ✅ Test CORS preflight requests
- ❌ NEVER use wildcard `*` in production
- ❌ NEVER include `http://` origins in production
- ❌ NEVER include localhost in production

### CORS Validation

The current CORS middleware (`backend/app/middleware/cors.py`) is production-ready with:
- Explicit origin whitelisting
- Credentials support
- Proper headers configuration
- No wildcard usage

**No code changes required** - just update `CORS_ORIGINS` environment variable.

---

## Database Migrations

### Migration Files (In Order)

The following migrations must be executed in sequence:

1. `000_drop_old_tables.sql` - Clean slate (if needed)
2. `001_create_chat_tables.sql` - Chat sessions and messages
3. `002_add_auto_purge.sql` - Automatic cleanup functions
4. `003_add_confidence_column.sql` - RAG confidence tracking
5. **`004_create_users_table.sql`** - Users, refresh tokens, login attempts ⚠️ NEW
6. **`005_create_profiles_table.sql`** - User profiles and preferences ⚠️ NEW
7. **`006_link_sessions_to_users.sql`** - Link chat sessions to users ⚠️ NEW
8. **`007_add_github_oauth.sql`** - GitHub OAuth support ⚠️ NEW

### Migration Execution

**Step 1: Backup Production Database**

```bash
# Railway example
railway run pg_dump $DATABASE_URL > backup_$(date +%Y%m%d_%H%M%S).sql
```

**Step 2: Execute Migrations**

```bash
# Connect to production database
psql $DATABASE_URL

# Execute each migration in order
\i backend/app/migrations/004_create_users_table.sql
\i backend/app/migrations/005_create_profiles_table.sql
\i backend/app/migrations/006_link_sessions_to_users.sql
\i backend/app/migrations/007_add_github_oauth.sql
```

**Step 3: Verify Migration Success**

```sql
-- Check tables exist
\dt

-- Verify users table structure
\d users

-- Verify foreign key constraints
SELECT conname, conrelid::regclass, confrelid::regclass
FROM pg_constraint
WHERE contype = 'f' AND conrelid::regclass::text IN ('users', 'user_profiles', 'chat_sessions', 'refresh_tokens');

-- Check indexes
SELECT schemaname, tablename, indexname
FROM pg_indexes
WHERE tablename IN ('users', 'user_profiles', 'refresh_tokens', 'login_attempts')
ORDER BY tablename, indexname;
```

### Migration Idempotency

All migration files use `IF NOT EXISTS` and conditional logic to ensure:
- ✅ Safe to re-run if interrupted
- ✅ No duplicate tables/columns/constraints
- ✅ Graceful handling of existing schema

### Rollback Procedure

If migrations fail, restore from backup:

```bash
# Stop application
railway down

# Restore database
psql $DATABASE_URL < backup_YYYYMMDD_HHMMSS.sql

# Restart application
railway up
```

---

## Environment Variables

### Complete Production Configuration

```bash
# =============================================================================
# OpenAI Configuration
# =============================================================================
OPENAI_API_KEY=sk-your-production-key

# =============================================================================
# Qdrant Vector Database
# =============================================================================
QDRANT_URL=https://your-production-cluster.qdrant.io
QDRANT_API_KEY=your-production-qdrant-key
QDRANT_COLLECTION=robotics_textbook

# =============================================================================
# Neon PostgreSQL Database
# =============================================================================
DATABASE_URL=postgres://user:password@ep-production.neon.tech/dbname?sslmode=require

# =============================================================================
# Backend Configuration
# =============================================================================
ENVIRONMENT=production
API_HOST=0.0.0.0
API_PORT=8000
CORS_ORIGINS=https://your-frontend.com,https://your-api.com

# =============================================================================
# Rate Limiting
# =============================================================================
RATE_LIMIT_ANONYMOUS=10
RATE_LIMIT_AUTHENTICATED=50

# =============================================================================
# RAG Configuration
# =============================================================================
CONFIDENCE_THRESHOLD=0.2
TOP_K_CHUNKS=5
MAX_CONVERSATION_HISTORY=5
SESSION_RETENTION_DAYS=30

# =============================================================================
# Logging Configuration
# =============================================================================
LOG_LEVEL=INFO
LOG_FORMAT=json

# =============================================================================
# JWT Configuration
# =============================================================================
JWT_PRIVATE_KEY="-----BEGIN PRIVATE KEY-----\nMIIE...\n-----END PRIVATE KEY-----"
JWT_PUBLIC_KEY="-----BEGIN PUBLIC KEY-----\nMIIB...\n-----END PUBLIC KEY-----"
JWT_ALGORITHM=RS256
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=1440
JWT_REFRESH_TOKEN_EXPIRE_DAYS=30

# =============================================================================
# GitHub OAuth Configuration
# =============================================================================
GITHUB_CLIENT_ID=your-production-client-id
GITHUB_CLIENT_SECRET=your-production-client-secret
GITHUB_REDIRECT_URI=https://your-api-domain.com/auth/github/callback

# =============================================================================
# Security Configuration
# =============================================================================
CSRF_SECRET_KEY=your-production-csrf-secret
FRONTEND_URL=https://your-production-domain.com
```

### Railway Deployment

```bash
# Set all variables at once
railway variables set ENVIRONMENT=production
railway variables set CORS_ORIGINS="https://your-frontend.com,https://your-api.com"
railway variables set JWT_PRIVATE_KEY="-----BEGIN PRIVATE KEY-----\n...\n-----END PRIVATE KEY-----"
railway variables set JWT_PUBLIC_KEY="-----BEGIN PUBLIC KEY-----\n...\n-----END PUBLIC KEY-----"
railway variables set GITHUB_CLIENT_ID="your-production-client-id"
railway variables set GITHUB_CLIENT_SECRET="your-production-client-secret"
railway variables set GITHUB_REDIRECT_URI="https://your-api-domain.com/auth/github/callback"
railway variables set FRONTEND_URL="https://your-production-domain.com"
railway variables set CSRF_SECRET_KEY="your-production-csrf-secret"
```

---

## Security Validation

### OWASP Security Checklist

Refer to `backend/OWASP_SECURITY_CHECKLIST.md` for complete security audit.

**Key Security Validations:**

- [ ] JWT tokens use RS256 algorithm (asymmetric encryption)
- [ ] Access tokens expire in 24 hours
- [ ] Refresh tokens expire in 30 days
- [ ] Password hashing uses Argon2id
- [ ] CSRF protection enabled with secure secret
- [ ] Rate limiting enforced (10 anon, 50 auth per hour)
- [ ] CORS restricted to production domains only
- [ ] OAuth redirect URIs whitelisted
- [ ] Database uses SSL/TLS (Neon enforces this)
- [ ] No secrets in version control
- [ ] Login attempts logged for audit
- [ ] Failed login lockout after 5 attempts
- [ ] SQL injection prevention (parameterized queries)
- [ ] XSS prevention (FastAPI auto-escaping)

### Security Testing Commands

```bash
# Test JWT key configuration
curl https://your-api-domain.com/auth/status

# Test CORS preflight
curl -X OPTIONS https://your-api-domain.com/auth/login \
  -H "Origin: https://your-frontend.com" \
  -H "Access-Control-Request-Method: POST"

# Test rate limiting
for i in {1..15}; do curl https://your-api-domain.com/query; done

# Validate database encryption
psql $DATABASE_URL -c "SHOW ssl;"
```

---

## Deployment Steps

### Step-by-Step Deployment Procedure

**1. Pre-Deployment Validation (Staging)**

```bash
# Run all tests
pytest backend/tests/

# Run security scan
bandit -r backend/app/

# Validate configuration
python -c "from backend.app.config import settings; print(settings.jwt_configured)"
```

**2. Database Migration (Production)**

```bash
# Backup database
railway run pg_dump $DATABASE_URL > backup_$(date +%Y%m%d_%H%M%S).sql

# Execute migrations
psql $DATABASE_URL -f backend/app/migrations/004_create_users_table.sql
psql $DATABASE_URL -f backend/app/migrations/005_create_profiles_table.sql
psql $DATABASE_URL -f backend/app/migrations/006_link_sessions_to_users.sql
psql $DATABASE_URL -f backend/app/migrations/007_add_github_oauth.sql

# Verify success
psql $DATABASE_URL -c "\dt"
```

**3. Update Environment Variables**

```bash
# Railway example (set all production variables)
railway variables set ENVIRONMENT=production
railway variables set JWT_PRIVATE_KEY="..."
railway variables set JWT_PUBLIC_KEY="..."
railway variables set GITHUB_CLIENT_ID="..."
railway variables set GITHUB_CLIENT_SECRET="..."
railway variables set GITHUB_REDIRECT_URI="https://your-api.com/auth/github/callback"
railway variables set FRONTEND_URL="https://your-frontend.com"
railway variables set CORS_ORIGINS="https://your-frontend.com,https://your-api.com"
railway variables set CSRF_SECRET_KEY="..."
```

**4. Deploy Application**

```bash
# Railway
railway up

# GitHub Actions (automatically triggered on push to main)
git push origin 001-user-auth:main
```

**5. Health Check Monitoring**

Monitor for first 15 minutes post-deployment:

```bash
# Check application health
curl https://your-api-domain.com/health

# Check authentication endpoints
curl https://your-api-domain.com/auth/status

# Monitor logs
railway logs
```

**6. Post-Deployment Verification**

See [Post-Deployment Verification](#post-deployment-verification) section below.

---

## Post-Deployment Verification

### Functional Testing

**1. Health Endpoint**
```bash
curl https://your-api-domain.com/health
# Expected: {"status": "healthy"}
```

**2. Auth Status**
```bash
curl https://your-api-domain.com/auth/status
# Expected: {"jwt_configured": true, "github_oauth_configured": true}
```

**3. User Registration**
```bash
curl -X POST https://your-api-domain.com/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "SecurePassword123!",
    "confirm_password": "SecurePassword123!"
  }'
# Expected: 201 Created with access_token and refresh_token
```

**4. User Login**
```bash
curl -X POST https://your-api-domain.com/auth/login \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "SecurePassword123!"
  }'
# Expected: 200 OK with access_token and refresh_token
```

**5. GitHub OAuth Flow**
```bash
# Step 1: Get authorization URL
curl https://your-api-domain.com/oauth/github/authorize
# Expected: {"authorization_url": "https://github.com/login/oauth/authorize?..."}

# Step 2: Visit URL in browser, authorize, verify callback
# Step 3: Check user created in database
```

**6. Protected Endpoint (with JWT)**
```bash
curl https://your-api-domain.com/users/me \
  -H "Authorization: Bearer eyJhbGciOiJSUzI1NiIs..."
# Expected: User profile data
```

**7. Rate Limiting**
```bash
# Test anonymous rate limit (should fail after 10 requests)
for i in {1..15}; do
  curl -w "\n%{http_code}\n" https://your-api-domain.com/query
done
# Expected: First 10 succeed (200), next 5 fail (429)
```

**8. CORS Validation**
```bash
curl -X OPTIONS https://your-api-domain.com/auth/login \
  -H "Origin: https://your-frontend.com" \
  -H "Access-Control-Request-Method: POST" \
  -v
# Expected: Access-Control-Allow-Origin header with your-frontend.com
```

### Database Verification

```sql
-- Connect to production database
psql $DATABASE_URL

-- Check tables exist
\dt

-- Verify users table structure
\d users

-- Check constraints
SELECT conname, contype, conrelid::regclass
FROM pg_constraint
WHERE conrelid::regclass::text IN ('users', 'user_profiles', 'refresh_tokens');

-- Verify indexes
SELECT tablename, indexname
FROM pg_indexes
WHERE tablename IN ('users', 'user_profiles', 'refresh_tokens', 'login_attempts');

-- Test cleanup functions exist
SELECT proname FROM pg_proc WHERE proname LIKE 'purge%';
```

### Security Verification

```bash
# 1. Check SSL/TLS enforcement
curl -I https://your-api-domain.com/health
# Expected: HTTP/2 200 (HTTPS working)

# 2. Verify CORS blocks unauthorized origins
curl -X OPTIONS https://your-api-domain.com/auth/login \
  -H "Origin: https://malicious-site.com" \
  -H "Access-Control-Request-Method: POST"
# Expected: No Access-Control-Allow-Origin header

# 3. Test JWT validation
curl https://your-api-domain.com/users/me \
  -H "Authorization: Bearer invalid-token"
# Expected: 401 Unauthorized

# 4. Test password requirements
curl -X POST https://your-api-domain.com/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "weak"}'
# Expected: 400 Bad Request (password too weak)
```

---

## Rollback Procedures

### Immediate Rollback (Critical Issues)

**1. Stop Application**
```bash
railway down
```

**2. Restore Database from Backup**
```bash
psql $DATABASE_URL < backup_YYYYMMDD_HHMMSS.sql
```

**3. Revert to Previous Deployment**
```bash
# Railway
railway rollback

# GitHub Actions
git revert HEAD
git push origin main
```

**4. Verify Rollback Success**
```bash
curl https://your-api-domain.com/health
```

### Partial Rollback (OAuth Issues Only)

If only OAuth is failing, disable OAuth without full rollback:

```bash
# Remove OAuth credentials (forces email/password only)
railway variables set GITHUB_CLIENT_ID=""
railway variables set GITHUB_CLIENT_SECRET=""
railway restart
```

### Database-Only Rollback

If application is fine but database migration failed:

```bash
# Restore database only
psql $DATABASE_URL < backup_YYYYMMDD_HHMMSS.sql

# Keep application running
```

---

## Monitoring and Alerts

### Key Metrics to Monitor

1. **Authentication Success Rate**
   - Target: >95% for valid credentials
   - Alert: <90% for 5 minutes

2. **JWT Validation Errors**
   - Target: <1% of requests
   - Alert: >5% for 5 minutes

3. **OAuth Flow Completion Rate**
   - Target: >80% (users who start OAuth complete it)
   - Alert: <60% for 15 minutes

4. **Rate Limit Triggers**
   - Target: <10% of users hit rate limit
   - Alert: >20% of users hit rate limit

5. **Database Connection Errors**
   - Target: 0 errors
   - Alert: Any error

6. **Failed Login Attempts**
   - Target: <10% of login attempts
   - Alert: >30% for 10 minutes (potential attack)

### Log Queries (Railway/CloudWatch)

```bash
# Authentication errors
railway logs | grep "authentication failed"

# OAuth failures
railway logs | grep "oauth error"

# Rate limit triggers
railway logs | grep "rate limit exceeded"

# Database errors
railway logs | grep "database error"
```

---

## Troubleshooting

### Common Issues and Resolutions

**Issue: JWT validation fails**
```
Error: "Invalid token signature"
```
**Resolution:**
1. Verify JWT_PRIVATE_KEY and JWT_PUBLIC_KEY match
2. Check keys are in single-line format with `\n` separators
3. Ensure keys are from same RSA pair

**Issue: OAuth redirect fails**
```
Error: "redirect_uri_mismatch"
```
**Resolution:**
1. Check GITHUB_REDIRECT_URI matches GitHub OAuth app configuration exactly
2. Verify HTTPS vs HTTP (production must use HTTPS)
3. Ensure no trailing slashes in URLs

**Issue: CORS errors in browser**
```
Error: "CORS policy: No 'Access-Control-Allow-Origin' header"
```
**Resolution:**
1. Add frontend domain to CORS_ORIGINS
2. Ensure HTTPS for production domains
3. Check CORS middleware is loaded in main.py

**Issue: Database migration fails**
```
Error: "relation already exists"
```
**Resolution:**
1. Migrations are idempotent; safe to re-run
2. Check for partial migration (some tables created)
3. Manually verify schema matches expected state

**Issue: Rate limiting too aggressive**
```
Error: "Rate limit exceeded" for legitimate users
```
**Resolution:**
1. Increase RATE_LIMIT_AUTHENTICATED in environment variables
2. Check user authentication is working (authenticated users get higher limit)
3. Monitor IP addresses triggering rate limits (potential bot traffic)

---

## Support and Escalation

### Issue Severity Levels

**P0 - Critical (Immediate Response Required)**
- Authentication completely broken
- Database unavailable
- Security breach detected

**P1 - High (Response within 1 hour)**
- OAuth flow failing for all users
- High rate of JWT validation errors
- Database migration failed in production

**P2 - Medium (Response within 4 hours)**
- OAuth flow failing for some users
- CORS errors for specific domains
- Rate limiting issues

**P3 - Low (Response within 24 hours)**
- Documentation updates needed
- Non-critical configuration tweaks
- Performance optimization requests

### Contact Information

- **On-Call Engineer:** [Your Contact]
- **Escalation Path:** [Your Escalation Process]
- **Documentation:** This file + `backend/OWASP_SECURITY_CHECKLIST.md`

---

## Appendix

### A. Generated Production Keys

**Location:** `D:\GitHub Connected\hackathon1_repeat\`
- `jwt_private_production.pem` - RSA private key (2048-bit)
- `jwt_public_production.pem` - RSA public key

**Security Note:** These keys are for demonstration purposes. Generate fresh keys for actual production deployment.

### B. Migration Checksums

Verify migration file integrity before execution:

```bash
# Generate checksums
sha256sum backend/app/migrations/*.sql
```

Expected checksums (for verification):
```
<hash>  backend/app/migrations/004_create_users_table.sql
<hash>  backend/app/migrations/005_create_profiles_table.sql
<hash>  backend/app/migrations/006_link_sessions_to_users.sql
<hash>  backend/app/migrations/007_add_github_oauth.sql
```

### C. Environment Variable Validation Script

```python
# validate_production_config.py
from backend.app.config import settings

def validate_production_config():
    """Validate all required production environment variables."""
    errors = []

    # JWT Configuration
    if not settings.jwt_configured:
        errors.append("JWT keys not configured")
    if settings.jwt_algorithm != "RS256":
        errors.append(f"JWT algorithm should be RS256, got {settings.jwt_algorithm}")

    # OAuth Configuration
    if not settings.github_oauth_configured:
        errors.append("GitHub OAuth not configured")

    # CORS Configuration
    if "localhost" in settings.cors_origins.lower():
        errors.append("CORS includes localhost (production should not)")
    if "*" in settings.cors_origins:
        errors.append("CORS uses wildcard (security risk)")

    # Environment Check
    if not settings.is_production:
        errors.append(f"ENVIRONMENT should be 'production', got '{settings.environment}'")

    # Logging Configuration
    if settings.log_format != "json":
        errors.append("LOG_FORMAT should be 'json' for production")

    if errors:
        print("❌ Production configuration validation FAILED:")
        for error in errors:
            print(f"  - {error}")
        return False
    else:
        print("✅ Production configuration validation PASSED")
        return True

if __name__ == "__main__":
    validate_production_config()
```

### D. Quick Reference Commands

```bash
# Health check
curl https://your-api-domain.com/health

# View logs
railway logs --tail 100

# Database console
railway run psql

# Restart service
railway restart

# View environment variables
railway variables

# Generate CSRF secret
python -c "import secrets; print(secrets.token_urlsafe(32))"

# Convert RSA key to single line (Linux/Mac)
awk 'NF {sub(/\r/, ""); printf "%s\\n",$0;}' jwt_private_production.pem
```

---

**Deployment Completed:** [Date/Time]
**Deployed By:** [Your Name]
**Version:** 001-user-auth
**Status:** ✅ Ready for Production
