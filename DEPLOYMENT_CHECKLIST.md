# Production Deployment Checklist - Authentication System

**Feature:** 001-user-auth
**Branch:** 001-user-auth
**Date:** 2025-12-20
**Status:** Pre-Deployment Validation

---

## Pre-Deployment Tasks

### 1. Security Configuration

- [ ] **Generate production RSA keys** (COMPLETED)
  - Generated: `jwt_private_production.pem` and `jwt_public_production.pem`
  - Algorithm: RS256 (2048-bit)
  - Location: Project root (NOT committed to version control)
  - Action Required: Convert to single-line format and store in secret manager

- [ ] **Create separate GitHub OAuth app for production**
  - Development OAuth app: Use for localhost only
  - Production OAuth app: Create new app with production URLs
  - Callback URL: `https://your-api-domain.com/auth/github/callback`
  - Homepage URL: `https://your-production-domain.com`

- [ ] **Generate production CSRF secret**
  ```bash
  python -c "import secrets; print(secrets.token_urlsafe(32))"
  ```

### 2. Environment Variables

- [ ] **Set ENVIRONMENT to production**
  ```bash
  railway variables set ENVIRONMENT=production
  ```

- [ ] **Configure JWT keys**
  ```bash
  # Convert keys to single-line format first (see PRODUCTION_DEPLOYMENT.md)
  railway variables set JWT_PRIVATE_KEY="-----BEGIN PRIVATE KEY-----\n...\n-----END PRIVATE KEY-----"
  railway variables set JWT_PUBLIC_KEY="-----BEGIN PUBLIC KEY-----\n...\n-----END PUBLIC KEY-----"
  railway variables set JWT_ALGORITHM=RS256
  railway variables set JWT_ACCESS_TOKEN_EXPIRE_MINUTES=1440
  railway variables set JWT_REFRESH_TOKEN_EXPIRE_DAYS=30
  ```

- [ ] **Configure GitHub OAuth**
  ```bash
  railway variables set GITHUB_CLIENT_ID="your-production-client-id"
  railway variables set GITHUB_CLIENT_SECRET="your-production-client-secret"
  railway variables set GITHUB_REDIRECT_URI="https://your-api-domain.com/auth/github/callback"
  ```

- [ ] **Configure CORS origins**
  ```bash
  railway variables set CORS_ORIGINS="https://your-frontend.com,https://your-api.com"
  ```

- [ ] **Configure frontend URL**
  ```bash
  railway variables set FRONTEND_URL="https://your-production-domain.com"
  ```

- [ ] **Configure CSRF secret**
  ```bash
  railway variables set CSRF_SECRET_KEY="your-production-csrf-secret"
  ```

- [ ] **Set logging to JSON format**
  ```bash
  railway variables set LOG_FORMAT=json
  railway variables set LOG_LEVEL=INFO
  ```

### 3. Database Preparation

- [ ] **Backup production database**
  ```bash
  railway run pg_dump $DATABASE_URL > backup_$(date +%Y%m%d_%H%M%S).sql
  ```

- [ ] **Test migrations in staging first**
  ```bash
  # Staging environment
  python backend/app/migrations/run_migrations.py --dry-run
  python backend/app/migrations/run_migrations.py
  ```

- [ ] **Verify migration idempotency**
  - All migrations use `IF NOT EXISTS`
  - Safe to re-run if interrupted
  - Tested in dev/staging environments

### 4. Code Validation

- [ ] **Run all tests**
  ```bash
  pytest backend/tests/ -v
  ```

- [ ] **Security scan**
  ```bash
  bandit -r backend/app/
  ```

- [ ] **Validate production configuration**
  ```bash
  python backend/validate_production_config.py
  ```
  Expected output: All checks PASSED

- [ ] **Review OWASP security checklist**
  - See: `backend/OWASP_SECURITY_CHECKLIST.md`
  - Verify all critical items addressed

### 5. Documentation Review

- [ ] **Production deployment guide**
  - Location: `PRODUCTION_DEPLOYMENT.md`
  - Reviewed: Yes/No
  - Updated: Yes/No

- [ ] **Environment variable documentation**
  - Location: `.env.example`
  - All variables documented: Yes/No
  - Production instructions clear: Yes/No

- [ ] **Migration documentation**
  - All migrations have description headers: Yes/No
  - Rollback procedures documented: Yes/No

---

## Deployment Execution

### Step 1: Final Validation

- [ ] **Confirm target environment**
  - Target: Production
  - Confirmed by: [Name]
  - Date/Time: [Timestamp]

- [ ] **Review all changes since last deployment**
  ```bash
  git diff main...001-user-auth
  git log main...001-user-auth --oneline
  ```

- [ ] **Pre-deployment smoke tests passed**
  - Health endpoint: Working
  - Auth endpoints: Working
  - Database connection: Working

### Step 2: Database Migration

- [ ] **Execute migrations on production database**
  ```bash
  # Connect to production database
  psql $DATABASE_URL

  # Execute migrations in order
  \i backend/app/migrations/004_create_users_table.sql
  \i backend/app/migrations/005_create_profiles_table.sql
  \i backend/app/migrations/006_link_sessions_to_users.sql
  \i backend/app/migrations/007_add_github_oauth.sql
  ```

- [ ] **Verify migration success**
  ```sql
  -- Check tables exist
  \dt

  -- Verify users table structure
  \d users

  -- Check foreign key constraints
  SELECT conname, conrelid::regclass, confrelid::regclass
  FROM pg_constraint
  WHERE contype = 'f' AND conrelid::regclass::text IN ('users', 'user_profiles', 'chat_sessions');
  ```

- [ ] **Migration execution time**
  - Started: [Timestamp]
  - Completed: [Timestamp]
  - Duration: [Duration]
  - Status: Success/Failed

### Step 3: Application Deployment

- [ ] **Deploy application**
  ```bash
  # Railway deployment
  railway up

  # OR: Push to main branch (triggers GitHub Actions)
  git push origin 001-user-auth:main
  ```

- [ ] **Monitor deployment progress**
  - Build status: Success/Failed
  - Deployment status: Success/Failed
  - Health checks: Passing/Failing

- [ ] **Deployment completion**
  - Started: [Timestamp]
  - Completed: [Timestamp]
  - Duration: [Duration]

### Step 4: Post-Deployment Verification

- [ ] **Health endpoint check**
  ```bash
  curl https://your-api-domain.com/health
  # Expected: {"status": "ok"}
  ```

- [ ] **Auth status check**
  ```bash
  curl https://your-api-domain.com/auth/status
  # Expected: {"jwt_configured": true, "github_oauth_configured": true}
  ```

- [ ] **User registration test**
  ```bash
  curl -X POST https://your-api-domain.com/auth/signup \
    -H "Content-Type: application/json" \
    -d '{
      "email": "test@example.com",
      "password": "SecurePassword123!",
      "confirm_password": "SecurePassword123!"
    }'
  # Expected: 201 Created with tokens
  ```

- [ ] **User login test**
  ```bash
  curl -X POST https://your-api-domain.com/auth/login \
    -H "Content-Type: application/json" \
    -d '{
      "email": "test@example.com",
      "password": "SecurePassword123!"
    }'
  # Expected: 200 OK with tokens
  ```

- [ ] **GitHub OAuth flow test**
  1. Visit: `https://your-api-domain.com/oauth/github/authorize`
  2. Authorize with GitHub account
  3. Verify callback redirect to frontend
  4. Check user created in database

- [ ] **Protected endpoint test**
  ```bash
  curl https://your-api-domain.com/users/me \
    -H "Authorization: Bearer <access_token>"
  # Expected: User profile data
  ```

- [ ] **CORS validation**
  ```bash
  curl -X OPTIONS https://your-api-domain.com/auth/login \
    -H "Origin: https://your-frontend.com" \
    -H "Access-Control-Request-Method: POST" \
    -v
  # Expected: Access-Control-Allow-Origin header present
  ```

- [ ] **Rate limiting test**
  ```bash
  # Test anonymous rate limit (should block after 10 requests)
  for i in {1..15}; do
    curl -w "\n%{http_code}\n" https://your-api-domain.com/query
  done
  # Expected: First 10 succeed, next 5 return 429
  ```

### Step 5: Monitoring Setup

- [ ] **Configure application monitoring**
  - Service: [Railway/DataDog/New Relic]
  - Dashboard URL: [URL]
  - Alerts configured: Yes/No

- [ ] **Monitor key metrics (first 15 minutes)**
  - Authentication success rate: ____%
  - JWT validation errors: ____
  - OAuth flow completion: ____%
  - Rate limit triggers: ____
  - Database connection errors: ____

- [ ] **Log monitoring**
  ```bash
  railway logs --tail 100
  ```
  - Authentication errors: None/Found
  - OAuth failures: None/Found
  - Database errors: None/Found

---

## Post-Deployment Tasks

### 1. Documentation Updates

- [ ] **Update production URLs in documentation**
  - README.md updated: Yes/No
  - API documentation updated: Yes/No
  - OAuth setup guide updated: Yes/No

- [ ] **Create deployment record**
  - Deployment date: [Date]
  - Deployed by: [Name]
  - Version/commit: [Git SHA]
  - Status: Success/Failed

- [ ] **Update change log**
  - Feature: 001-user-auth deployed
  - Version: [Version number]
  - Release notes created: Yes/No

### 2. User Communication

- [ ] **Internal team notification**
  - Slack/Teams message sent: Yes/No
  - Stakeholders notified: Yes/No
  - Documentation shared: Yes/No

- [ ] **External user notification (if applicable)**
  - Release announcement: Yes/No
  - Migration guide: Yes/No
  - Support documentation: Yes/No

### 3. Cleanup

- [ ] **Remove development/test data**
  ```sql
  DELETE FROM users WHERE email LIKE '%test%' OR email LIKE '%example.com';
  DELETE FROM login_attempts WHERE created_at < NOW() - INTERVAL '7 days';
  ```

- [ ] **Archive deployment artifacts**
  - Database backup stored: Yes/No
  - Deployment logs archived: Yes/No
  - Configuration snapshot saved: Yes/No

### 4. Long-term Monitoring

- [ ] **Set up recurring health checks**
  - Frequency: [e.g., every 5 minutes]
  - Uptime monitoring: [Service/URL]
  - Alert recipients: [Email/Slack]

- [ ] **Schedule key rotation**
  - JWT keys rotation: [Date in 6 months]
  - CSRF secret rotation: [Date in 6 months]
  - OAuth credentials review: [Date in 1 year]

- [ ] **Database maintenance schedule**
  - Auto-purge functions: Enabled
  - Backup frequency: [Daily/Weekly]
  - Performance monitoring: [Service]

---

## Rollback Procedures

### If Deployment Fails

- [ ] **Stop application**
  ```bash
  railway down
  ```

- [ ] **Restore database from backup**
  ```bash
  psql $DATABASE_URL < backup_YYYYMMDD_HHMMSS.sql
  ```

- [ ] **Revert to previous deployment**
  ```bash
  railway rollback
  # OR
  git revert HEAD && git push origin main
  ```

- [ ] **Verify rollback success**
  ```bash
  curl https://your-api-domain.com/health
  ```

- [ ] **Post-rollback communication**
  - Team notified: Yes/No
  - Incident report created: Yes/No
  - Root cause analysis scheduled: Yes/No

---

## Sign-Off

### Pre-Deployment Approval

- [ ] **Technical Lead:** [Name] | [Date]
- [ ] **Security Review:** [Name] | [Date]
- [ ] **Product Owner:** [Name] | [Date]

### Post-Deployment Verification

- [ ] **Deployment Engineer:** [Name] | [Date]
- [ ] **QA Verification:** [Name] | [Date]
- [ ] **Production Sign-Off:** [Name] | [Date]

---

## Notes and Issues

### Deployment Notes
```
[Record any notable observations, configuration changes, or decisions made during deployment]
```

### Issues Encountered
```
[Document any issues encountered during deployment and their resolutions]
```

### Follow-Up Actions
```
[List any follow-up actions required post-deployment]
```

---

## Quick Reference

### Emergency Contacts
- On-Call Engineer: [Contact]
- Database Admin: [Contact]
- Security Team: [Contact]

### Important URLs
- Production API: https://your-api-domain.com
- Production Frontend: https://your-production-domain.com
- API Documentation: https://your-api-domain.com/api/docs
- Monitoring Dashboard: [URL]
- Deployment Platform: [Railway/etc URL]

### Key Commands
```bash
# View logs
railway logs --tail 100

# Database console
railway run psql

# Restart service
railway restart

# View environment variables
railway variables

# Health check
curl https://your-api-domain.com/health
```

---

**Deployment Status:** [ ] Not Started | [ ] In Progress | [ ] Completed | [ ] Failed | [ ] Rolled Back

**Final Notes:**
