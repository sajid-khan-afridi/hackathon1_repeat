# Research: User Authentication System

**Feature**: 004-user-auth | **Date**: 2025-12-17 | **Phase**: 0 (Research)

## Overview

This document captures research findings and technology decisions for implementing user authentication in the Physical AI & Humanoid Robotics Textbook. The authentication system must integrate with the existing FastAPI backend on Railway and Docusaurus/React frontend on GitHub Pages.

---

## Research Tasks

### 1. Authentication Library Selection for FastAPI

**Question**: Should we use Better Auth (designed for Next.js), FastAPI-Users, or custom implementation?

**Findings**:

| Option | Pros | Cons |
|--------|------|------|
| **Better Auth** | Full-featured, modern, TypeScript-first | Designed for Next.js/Node.js, would require significant adaptation for Python/FastAPI |
| **FastAPI-Users** | Native FastAPI integration, well-documented | Additional dependency, opinionated structure may conflict with existing services |
| **Custom PyJWT Implementation** | Full control, minimal dependencies, fits existing asyncpg pattern | More code to write and maintain, security responsibility on us |

**Decision**: Custom PyJWT Implementation
**Rationale**:
1. The existing codebase uses direct asyncpg queries with async/await patterns - custom auth fits this style
2. Better Auth requires Node.js runtime which conflicts with our Python backend
3. FastAPI-Users adds ORM dependencies (SQLAlchemy) which conflicts with our raw SQL migration approach
4. PyJWT is battle-tested (5M+ monthly downloads) with RS256 support
5. Constitution principle: "Smallest Viable Change" - custom auth is ~300 lines vs adding new framework

**Alternatives Considered**:
- Better Auth: Rejected due to runtime mismatch (Node.js vs Python)
- FastAPI-Users: Rejected due to ORM dependency conflict

---

### 2. Password Hashing Algorithm

**Question**: Which password hashing algorithm and parameters should we use?

**Findings**:

| Algorithm | Security | Performance | Recommendation |
|-----------|----------|-------------|----------------|
| **bcrypt** | Excellent, adaptive cost factor | ~300ms at cost=12 | Recommended |
| **Argon2** | Best (PHC winner), memory-hard | ~500ms default | Overkill for our scale |
| **scrypt** | Good, memory-hard | Variable | Less common in Python ecosystem |

**Decision**: bcrypt with cost factor 12
**Rationale**:
1. Constitution specifies bcrypt (cost factor 12) in Security Principles (line 118)
2. passlib[bcrypt] is well-supported in Python ecosystem
3. Cost factor 12 provides ~300ms hash time (protects against brute force while meeting <500ms p95 target)
4. Same algorithm used by Better Auth, ensuring future migration path if needed

**Implementation**:
```python
from passlib.context import CryptContext
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto", bcrypt__rounds=12)
```

---

### 3. JWT Token Strategy

**Question**: How should we structure and sign JWT tokens?

**Findings**:

| Aspect | Decision | Rationale |
|--------|----------|-----------|
| **Signing Algorithm** | RS256 (RSA + SHA-256) | Constitution requires RS256 (line 119); asymmetric allows public key verification |
| **Access Token Expiry** | 24 hours | Per FR-011 specification |
| **Refresh Token Expiry** | 30 days | Per FR-012 specification |
| **Token Storage (Frontend)** | httpOnly secure cookies | Per FR-025; prevents XSS access to tokens |
| **Token Refresh** | Silent refresh via /auth/refresh endpoint | Per FR-010; no user interaction required |

**Token Payload Structure**:
```json
{
  "sub": "user_uuid",
  "email": "user@example.com",
  "type": "access|refresh",
  "iat": 1702800000,
  "exp": 1702886400
}
```

**Key Management**:
- RSA key pair generated and stored in environment variables
- Private key: `JWT_PRIVATE_KEY` (PEM format, for signing)
- Public key: `JWT_PUBLIC_KEY` (PEM format, for verification)

---

### 4. Google OAuth 2.0 Integration

**Question**: How to implement Google OAuth for FastAPI with Docusaurus frontend?

**Findings**:

**Flow Architecture**:
1. Frontend redirects to Google OAuth consent screen
2. Google redirects back to backend callback URL with authorization code
3. Backend exchanges code for tokens, extracts user info
4. Backend creates/updates user, issues JWT tokens
5. Backend redirects to frontend with tokens in secure cookies

**Required Google Cloud Configuration**:
- OAuth 2.0 Client ID (Web application type)
- Authorized redirect URI: `https://api.domain.com/auth/google/callback`
- Required scopes: `openid`, `email`, `profile`

**Environment Variables**:
```
GOOGLE_CLIENT_ID=xxx.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=GOCSPX-xxx
GOOGLE_REDIRECT_URI=https://api.domain.com/auth/google/callback
```

**Library**: `httpx` for async HTTP requests to Google APIs (already used in codebase)

---

### 5. Account Linking Strategy

**Question**: How to handle when Google OAuth email matches existing email account? (FR-008)

**Findings**:

**Scenario Analysis**:

| Scenario | Behavior |
|----------|----------|
| New Google user, email not in system | Create new user with Google ID |
| New Google user, email exists (email signup) | Auto-link: add Google ID to existing user |
| Existing Google user | Authenticate normally via Google ID |
| User tries email signup with Google-linked email | Allow - user can have both auth methods |

**Decision**: Auto-link accounts by email
**Rationale**:
1. FR-008 requires auto-linking when Google email matches existing account
2. User convenience - no manual account merge needed
3. Security: Google email verification provides trust in email ownership

**Implementation**:
```python
async def handle_google_oauth(google_user_info):
    existing_user = await get_user_by_email(google_user_info['email'])
    if existing_user:
        # Auto-link: add Google ID to existing user
        await update_user_google_id(existing_user.id, google_user_info['sub'])
        return existing_user
    else:
        # Create new user with Google info
        return await create_user_from_google(google_user_info)
```

---

### 6. Rate Limiting Integration

**Question**: How to integrate auth-based rate limiting with existing rate limiter?

**Findings**:

**Existing Infrastructure** (from `backend/app/services/rate_limiter.py`):
- SlowAPI-based rate limiting already configured
- Config supports `rate_limit_anonymous: 10` and `rate_limit_authenticated: 50`

**Integration Points**:
1. `UserIdentificationMiddleware` extracts user ID from JWT token
2. Rate limiter checks user ID to determine limit tier
3. Response headers include `X-RateLimit-Limit`, `X-RateLimit-Remaining`, `X-RateLimit-Reset`

**Decision**: Extend existing rate limiter with JWT user extraction
**Implementation**:
```python
# In middleware/rate_limit.py
def get_user_id_from_request(request: Request) -> Optional[str]:
    token = request.cookies.get("access_token")
    if token:
        payload = verify_jwt_token(token)
        return payload.get("sub")  # user_id
    return None  # Anonymous user
```

---

### 7. Session Management Approach

**Question**: How to manage user sessions (stateless JWT vs stateful sessions)?

**Findings**:

| Approach | Pros | Cons |
|----------|------|------|
| **Stateless JWT** | Scalable, no DB lookup per request | Can't revoke tokens instantly |
| **Stateful Sessions (DB)** | Instant revocation, session tracking | DB lookup per request, more complex |
| **Hybrid (JWT + DB blacklist)** | Best of both | Additional complexity |

**Decision**: Stateless JWT with optional refresh token revocation
**Rationale**:
1. FR-014 requires concurrent sessions on multiple devices - stateless JWT simplifies this
2. 24-hour access token expiry limits damage if token leaked
3. Logout invalidates refresh token (stored in DB), not access token
4. Constitution principle: "Smallest Viable Change" - stateless is simpler

**Logout Implementation**:
```python
async def logout(refresh_token: str):
    # Delete refresh token from database (invalidates it)
    await delete_refresh_token(refresh_token)
    # Access token still valid until expiry, but short-lived (24h)
    # Frontend clears cookies
```

---

### 8. Password Validation Rules

**Question**: What password validation rules should we implement? (FR-003)

**Findings**:

**Specification Requirements** (from spec.md):
- Minimum 8 characters
- At least 1 uppercase letter
- At least 1 number

**Implementation**:
```python
import re

def validate_password(password: str) -> tuple[bool, str]:
    if len(password) < 8:
        return False, "Password must be at least 8 characters"
    if not re.search(r'[A-Z]', password):
        return False, "Password must contain at least one uppercase letter"
    if not re.search(r'[0-9]', password):
        return False, "Password must contain at least one number"
    return True, ""
```

**Frontend Validation**: Same rules applied on form input (real-time feedback per FR-031)

---

### 9. CSRF Protection Strategy

**Question**: How to implement CSRF protection for auth endpoints? (FR-027)

**Findings**:

**Options**:
1. **Double Submit Cookie**: Generate CSRF token, store in cookie and header
2. **Synchronizer Token**: Server-side token storage, validate on requests
3. **SameSite Cookie Attribute**: Modern browser protection

**Decision**: SameSite=Strict cookies + CSRF header for state-changing requests
**Rationale**:
1. `SameSite=Strict` prevents most CSRF attacks in modern browsers
2. Additional `X-CSRF-Token` header for POST/PUT/DELETE requests
3. Token stored in httpOnly cookie, required in header (double-submit pattern)

**Implementation**:
```python
# Set cookies with SameSite=Strict
response.set_cookie(
    key="access_token",
    value=token,
    httponly=True,
    secure=True,  # HTTPS only
    samesite="strict",
    max_age=86400  # 24 hours
)
```

---

### 10. Frontend Auth State Management

**Question**: How to manage auth state in Docusaurus/React frontend?

**Findings**:

**Existing Pattern** (from `src/context/UserContext.tsx`):
- React Context for user profile preferences
- localStorage for persistence

**Auth State Requirements**:
1. Track logged-in status across page navigations
2. Store user info (email, profile completion status)
3. Trigger token refresh when needed
4. Integrate with existing UserContext for profile data

**Decision**: New AuthContext that wraps/extends UserContext
**Implementation**:
```typescript
// src/context/AuthContext.tsx
interface AuthState {
  isAuthenticated: boolean;
  user: User | null;
  isLoading: boolean;
}

// Check auth status on app load by calling /auth/me endpoint
// Store user in React state (not localStorage - cookies handle persistence)
// Expose login(), logout(), refresh() methods
```

---

## Summary of Decisions

| Decision | Choice | Key Rationale |
|----------|--------|---------------|
| Auth Library | Custom PyJWT | Fits existing asyncpg pattern, no framework lock-in |
| Password Hashing | bcrypt (cost=12) | Constitution requirement, ~300ms hash time |
| JWT Signing | RS256 | Constitution requirement, asymmetric keys |
| Token Storage | httpOnly cookies | XSS protection per FR-025 |
| OAuth Provider | Google only | Spec scope (others out of scope) |
| Account Linking | Auto-link by email | FR-008 requirement |
| Rate Limiting | Extend existing | Leverage SlowAPI infrastructure |
| Sessions | Stateless JWT | Simplicity, concurrent device support |
| CSRF | SameSite + Double Submit | Modern browser protection + fallback |
| Frontend State | React AuthContext | Extends existing context pattern |

---

## Dependencies to Add

### Backend (requirements.txt)
```
passlib[bcrypt]==1.7.4    # Password hashing
PyJWT==2.8.0              # JWT token handling
cryptography==41.0.7      # RSA key support for PyJWT
```

### Frontend (package.json)
```json
{
  "dependencies": {
    "jwt-decode": "^4.0.0"  # Decode JWT for user info display (not verification)
  }
}
```

---

## Environment Variables to Add

```bash
# JWT Configuration
JWT_PRIVATE_KEY="-----BEGIN RSA PRIVATE KEY-----\n...\n-----END RSA PRIVATE KEY-----"
JWT_PUBLIC_KEY="-----BEGIN PUBLIC KEY-----\n...\n-----END PUBLIC KEY-----"
JWT_ALGORITHM="RS256"
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=1440    # 24 hours
JWT_REFRESH_TOKEN_EXPIRE_DAYS=30

# Google OAuth
GOOGLE_CLIENT_ID="xxx.apps.googleusercontent.com"
GOOGLE_CLIENT_SECRET="GOCSPX-xxx"
GOOGLE_REDIRECT_URI="https://api.domain.com/auth/google/callback"

# Security
CSRF_SECRET_KEY="random-32-char-string"
CORS_ORIGINS="https://your-frontend.github.io"
```

---

## Risks and Mitigations

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| JWT token leaked | High | Low | Short expiry (24h), httpOnly cookies, HTTPS only |
| Brute force login | Medium | Medium | Rate limiting (10 attempts/15min), bcrypt cost factor |
| Google OAuth unavailable | Medium | Low | Fallback to email/password login suggested |
| CORS misconfiguration | High | Low | Explicit origin whitelist, no wildcards |

---

## References

- [PyJWT Documentation](https://pyjwt.readthedocs.io/)
- [passlib Documentation](https://passlib.readthedocs.io/)
- [Google OAuth 2.0 for Web](https://developers.google.com/identity/protocols/oauth2/web-server)
- [OWASP Authentication Cheat Sheet](https://cheatsheetseries.owasp.org/cheatsheets/Authentication_Cheat_Sheet.html)
- [Constitution Security Principles](../../.specify/memory/constitution.md#security-principles)
