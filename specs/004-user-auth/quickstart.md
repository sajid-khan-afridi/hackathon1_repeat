# Quickstart: User Authentication System

**Feature**: 004-user-auth | **Date**: 2025-12-17

This guide covers setting up the development environment for implementing user authentication.

---

## Prerequisites

- Python 3.11+ installed
- Node.js 18+ installed
- Access to Neon PostgreSQL database (existing)
- Google Cloud Console account (for OAuth)
- Existing backend and frontend codebase checked out

---

## Step 1: Generate RSA Key Pair for JWT

Generate an RSA key pair for signing JWT tokens (RS256 algorithm):

```bash
# Generate private key (keep secure!)
openssl genrsa -out jwt_private.pem 2048

# Extract public key from private key
openssl rsa -in jwt_private.pem -pubout -out jwt_public.pem

# Convert to single-line format for environment variables
# Private key
cat jwt_private.pem | tr '\n' '|' | sed 's/|/\\n/g'

# Public key
cat jwt_public.pem | tr '\n' '|' | sed 's/|/\\n/g'
```

Store the keys securely. Never commit these files to git.

---

## Step 2: Configure Google OAuth 2.0

### Create OAuth Credentials

1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project or select existing
3. Navigate to **APIs & Services** > **Credentials**
4. Click **Create Credentials** > **OAuth client ID**
5. Select **Web application**
6. Configure:
   - **Name**: Robotics Textbook Auth
   - **Authorized JavaScript origins**:
     - `http://localhost:3000` (dev)
     - `https://your-frontend.github.io` (prod)
   - **Authorized redirect URIs**:
     - `http://localhost:8000/auth/google/callback` (dev)
     - `https://api.robotics-textbook.railway.app/auth/google/callback` (prod)
7. Click **Create** and note the Client ID and Client Secret

### Enable Google+ API

1. Navigate to **APIs & Services** > **Library**
2. Search for "Google+ API" or "Google People API"
3. Enable it for your project

---

## Step 3: Update Backend Environment Variables

Add the following to `backend/.env`:

```bash
# ==========================================
# JWT Configuration (Phase 4A: Authentication)
# ==========================================
JWT_PRIVATE_KEY="-----BEGIN RSA PRIVATE KEY-----\nMIIE...\n-----END RSA PRIVATE KEY-----"
JWT_PUBLIC_KEY="-----BEGIN PUBLIC KEY-----\nMIIB...\n-----END PUBLIC KEY-----"
JWT_ALGORITHM="RS256"
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=1440
JWT_REFRESH_TOKEN_EXPIRE_DAYS=30

# ==========================================
# Google OAuth Configuration
# ==========================================
GOOGLE_CLIENT_ID="your-client-id.apps.googleusercontent.com"
GOOGLE_CLIENT_SECRET="GOCSPX-your-secret"
GOOGLE_REDIRECT_URI="http://localhost:8000/auth/google/callback"

# ==========================================
# Security Configuration
# ==========================================
CSRF_SECRET_KEY="generate-a-32-char-random-string"

# ==========================================
# CORS Origins (add frontend URL)
# ==========================================
CORS_ORIGINS="http://localhost:3000,https://your-frontend.github.io"
```

**Generate CSRF Secret**:
```bash
python -c "import secrets; print(secrets.token_urlsafe(32))"
```

---

## Step 4: Install Backend Dependencies

Add to `backend/requirements.txt`:

```
# Authentication (Phase 4A)
passlib[bcrypt]==1.7.4
PyJWT==2.8.0
cryptography==41.0.7
```

Install dependencies:

```bash
cd backend
pip install -r requirements.txt
```

---

## Step 5: Run Database Migrations

Create the authentication tables:

```bash
cd backend
python run_migrations.py
```

Expected migrations to run:
- `004_create_users_table.sql` - Users and refresh_tokens tables
- `005_create_profiles_table.sql` - User profiles table
- `006_link_sessions_to_users.sql` - Link chat_sessions to users

Verify tables exist:

```bash
# Connect to Neon database
psql $DATABASE_URL

# Check tables
\dt

# Expected output should include:
# users
# refresh_tokens
# user_profiles
# login_attempts
```

---

## Step 6: Update Frontend Environment Variables

Add to root `.env` or `docusaurus.config.ts`:

```bash
# Frontend environment variables
REACT_APP_API_URL="http://localhost:8000"

# For production (docusaurus.config.ts customFields)
customFields: {
  apiUrl: process.env.API_URL || 'http://localhost:8000',
}
```

---

## Step 7: Install Frontend Dependencies

Add to `package.json`:

```json
{
  "dependencies": {
    "jwt-decode": "^4.0.0"
  }
}
```

Install:

```bash
npm install
```

---

## Step 8: Start Development Servers

### Terminal 1: Backend

```bash
cd backend
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

### Terminal 2: Frontend

```bash
npm start
```

---

## Step 9: Verify Setup

### Test API Health

```bash
curl http://localhost:8000/health
# Expected: {"status": "healthy", ...}
```

### Test Signup Endpoint (after implementation)

```bash
curl -X POST http://localhost:8000/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "TestPass123"}'
```

### Test Google OAuth (after implementation)

1. Navigate to `http://localhost:8000/auth/google`
2. Should redirect to Google consent screen
3. After authorization, should redirect back with cookies set

---

## Environment Variable Reference

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `JWT_PRIVATE_KEY` | Yes | - | RSA private key (PEM format) |
| `JWT_PUBLIC_KEY` | Yes | - | RSA public key (PEM format) |
| `JWT_ALGORITHM` | No | `RS256` | JWT signing algorithm |
| `JWT_ACCESS_TOKEN_EXPIRE_MINUTES` | No | `1440` | Access token expiry (24h) |
| `JWT_REFRESH_TOKEN_EXPIRE_DAYS` | No | `30` | Refresh token expiry |
| `GOOGLE_CLIENT_ID` | Yes | - | Google OAuth client ID |
| `GOOGLE_CLIENT_SECRET` | Yes | - | Google OAuth client secret |
| `GOOGLE_REDIRECT_URI` | Yes | - | OAuth callback URL |
| `CSRF_SECRET_KEY` | Yes | - | Secret for CSRF token generation |
| `CORS_ORIGINS` | Yes | - | Comma-separated allowed origins |

---

## File Structure After Setup

```
backend/
├── .env                          # Updated with auth variables
├── requirements.txt              # Updated with auth dependencies
├── app/
│   ├── config.py                 # MODIFY: Add auth config
│   ├── models/
│   │   ├── user.py               # NEW
│   │   └── profile.py            # NEW
│   ├── services/
│   │   ├── auth_service.py       # NEW
│   │   ├── user_service.py       # NEW
│   │   └── profile_service.py    # NEW
│   ├── routers/
│   │   ├── auth.py               # NEW
│   │   ├── users.py              # NEW
│   │   └── oauth.py              # NEW
│   ├── middleware/
│   │   └── auth.py               # NEW
│   └── migrations/
│       ├── 004_create_users_table.sql      # NEW
│       ├── 005_create_profiles_table.sql   # NEW
│       └── 006_link_sessions_to_users.sql  # NEW

src/
├── components/
│   ├── Auth/                     # NEW directory
│   └── Profile/                  # NEW directory
├── context/
│   └── AuthContext.tsx           # NEW
├── hooks/
│   └── useAuth.ts                # NEW
├── pages/
│   ├── login.tsx                 # NEW
│   ├── signup.tsx                # NEW
│   └── profile.tsx               # NEW
└── services/
    └── authApi.ts                # NEW
```

---

## Troubleshooting

### JWT Key Format Issues

If you get "Invalid key format" errors:
- Ensure newlines in PEM are properly escaped (`\n`)
- Verify key begins with `-----BEGIN RSA PRIVATE KEY-----`
- Test key parsing: `python -c "from cryptography.hazmat.primitives import serialization; serialization.load_pem_private_key(open('jwt_private.pem', 'rb').read(), password=None)"`

### Google OAuth Redirect Errors

Common issues:
- **redirect_uri_mismatch**: Ensure exact match in Google Console
- **invalid_client**: Check client ID/secret are correct
- **access_denied**: User denied consent or app not verified

### Database Connection Issues

- Verify `DATABASE_URL` is correct in `.env`
- Check Neon database is active (free tier may sleep)
- Run `python -c "import asyncpg; print('OK')"` to verify driver

### CORS Errors

- Add frontend origin to `CORS_ORIGINS`
- Ensure `credentials: 'include'` in fetch requests
- Check `Access-Control-Allow-Credentials: true` in response

---

## Next Steps

After completing setup:

1. Implement auth service (`backend/app/services/auth_service.py`)
2. Implement auth router (`backend/app/routers/auth.py`)
3. Implement auth middleware (`backend/app/middleware/auth.py`)
4. Implement frontend AuthContext (`src/context/AuthContext.tsx`)
5. Implement login/signup forms (`src/components/Auth/`)
6. Implement profile wizard (`src/components/Profile/`)
7. Run tests and verify quality gates

See `tasks.md` for detailed implementation tasks (generated by `/sp.tasks`).
