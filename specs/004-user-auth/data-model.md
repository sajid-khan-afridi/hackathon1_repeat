# Data Model: User Authentication System

**Feature**: 004-user-auth | **Date**: 2025-12-17 | **Phase**: 1 (Design)

## Overview

This document defines the database schema and entity relationships for the user authentication system. The schema integrates with the existing Neon PostgreSQL database used by the RAG chatbot.

---

## Entity Relationship Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                         AUTHENTICATION                          │
└─────────────────────────────────────────────────────────────────┘

┌──────────────────┐       ┌──────────────────┐       ┌──────────────────┐
│      users       │       │  refresh_tokens  │       │  user_profiles   │
├──────────────────┤       ├──────────────────┤       ├──────────────────┤
│ id (PK)          │───┬──>│ id (PK)          │       │ id (PK)          │
│ email (UNIQUE)   │   │   │ user_id (FK)     │<──────│ user_id (FK)     │<───┐
│ password_hash    │   │   │ token_hash       │       │ experience_level │    │
│ google_id        │   │   │ expires_at       │       │ ros_familiarity  │    │
│ created_at       │   │   │ created_at       │       │ hardware_access  │    │
│ updated_at       │   │   │ revoked_at       │       │ learning_goal    │    │
│ last_login_at    │   │   └──────────────────┘       │ preferred_lang   │    │
│ is_active        │   │                              │ is_complete      │    │
│ failed_attempts  │   │                              │ created_at       │    │
│ locked_until     │   │                              │ updated_at       │    │
└──────────────────┘   │                              └──────────────────┘    │
                       │                                                       │
                       │   ┌──────────────────┐                               │
                       │   │  chat_sessions   │  (existing table)             │
                       │   ├──────────────────┤                               │
                       └──>│ user_id (FK)     │───────────────────────────────┘
                           │ session_token    │
                           │ ...              │
                           └──────────────────┘
```

---

## Entities

### 1. Users

**Purpose**: Store user authentication credentials and account status.

**Table**: `users`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique user identifier |
| `email` | VARCHAR(255) | NOT NULL, UNIQUE | User email address (login identifier) |
| `password_hash` | VARCHAR(255) | NULL | bcrypt hash (null for OAuth-only users) |
| `google_id` | VARCHAR(255) | NULL, UNIQUE | Google OAuth subject ID (for linked accounts) |
| `created_at` | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Account creation timestamp |
| `updated_at` | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Last update timestamp |
| `last_login_at` | TIMESTAMPTZ | NULL | Most recent login timestamp |
| `is_active` | BOOLEAN | NOT NULL, DEFAULT TRUE | Account enabled status |
| `failed_login_attempts` | INTEGER | NOT NULL, DEFAULT 0 | Failed login counter (for lockout) |
| `locked_until` | TIMESTAMPTZ | NULL | Account lockout expiry (null = not locked) |

**Indexes**:
- `idx_users_email` ON `email` (for login lookup)
- `idx_users_google_id` ON `google_id` WHERE `google_id IS NOT NULL` (for OAuth lookup)

**Validation Rules** (per FR-002, FR-003):
- Email: Valid email format (regex validation)
- Password: Min 8 chars, 1 uppercase, 1 number (validated before hashing)

**SQL Migration** (`004_create_users_table.sql`):
```sql
CREATE TABLE IF NOT EXISTS users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) NOT NULL UNIQUE,
    password_hash VARCHAR(255),
    google_id VARCHAR(255) UNIQUE,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    last_login_at TIMESTAMPTZ,
    is_active BOOLEAN NOT NULL DEFAULT TRUE,
    failed_login_attempts INTEGER NOT NULL DEFAULT 0,
    locked_until TIMESTAMPTZ,

    -- Ensure at least one auth method exists
    CONSTRAINT users_auth_method_check CHECK (
        password_hash IS NOT NULL OR google_id IS NOT NULL
    )
);

CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);
CREATE INDEX IF NOT EXISTS idx_users_google_id ON users(google_id) WHERE google_id IS NOT NULL;

-- Trigger to update updated_at on row changes
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ language 'plpgsql';

CREATE TRIGGER update_users_updated_at
    BEFORE UPDATE ON users
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();
```

---

### 2. Refresh Tokens

**Purpose**: Store refresh tokens for session management and token revocation.

**Table**: `refresh_tokens`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique token identifier |
| `user_id` | UUID | NOT NULL, FOREIGN KEY → users(id) ON DELETE CASCADE | Token owner |
| `token_hash` | VARCHAR(255) | NOT NULL | SHA-256 hash of refresh token |
| `expires_at` | TIMESTAMPTZ | NOT NULL | Token expiration (30 days from creation) |
| `created_at` | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Token creation timestamp |
| `revoked_at` | TIMESTAMPTZ | NULL | Revocation timestamp (null = active) |
| `user_agent` | VARCHAR(500) | NULL | Browser/client info for session display |
| `ip_address` | INET | NULL | Client IP for security auditing |

**Indexes**:
- `idx_refresh_tokens_user_id` ON `user_id` (for user session listing)
- `idx_refresh_tokens_token_hash` ON `token_hash` (for token lookup)
- `idx_refresh_tokens_expires_at` ON `expires_at` (for cleanup job)

**SQL Migration** (`004_create_users_table.sql` - continued):
```sql
CREATE TABLE IF NOT EXISTS refresh_tokens (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token_hash VARCHAR(255) NOT NULL,
    expires_at TIMESTAMPTZ NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    revoked_at TIMESTAMPTZ,
    user_agent VARCHAR(500),
    ip_address INET
);

CREATE INDEX IF NOT EXISTS idx_refresh_tokens_user_id ON refresh_tokens(user_id);
CREATE INDEX IF NOT EXISTS idx_refresh_tokens_token_hash ON refresh_tokens(token_hash);
CREATE INDEX IF NOT EXISTS idx_refresh_tokens_expires_at ON refresh_tokens(expires_at);

-- Auto-purge expired/revoked tokens (runs daily via pg_cron or application)
-- Tokens are kept for 7 days after expiry for audit purposes
```

---

### 3. User Profiles

**Purpose**: Store user learning preferences for personalization (Phase 4B).

**Table**: `user_profiles`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique profile identifier |
| `user_id` | UUID | NOT NULL, UNIQUE, FOREIGN KEY → users(id) ON DELETE CASCADE | Profile owner (1:1 with users) |
| `experience_level` | VARCHAR(50) | NULL | Programming experience: beginner/intermediate/advanced |
| `ros_familiarity` | VARCHAR(50) | NULL | ROS knowledge: none/basic/proficient |
| `hardware_access` | VARCHAR(50) | NULL | Hardware: simulation_only/jetson_kit/full_robot_lab |
| `learning_goal` | VARCHAR(50) | NULL | Goal: career_transition/academic_research/hobby |
| `preferred_language` | VARCHAR(20) | NULL | Code examples: python/cpp/both |
| `is_complete` | BOOLEAN | NOT NULL, DEFAULT FALSE | All 5 questions answered |
| `created_at` | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Profile creation timestamp |
| `updated_at` | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Last update timestamp |

**Enum Values** (per spec Profile Questions):

| Field | Valid Values |
|-------|--------------|
| `experience_level` | `beginner`, `intermediate`, `advanced` |
| `ros_familiarity` | `none`, `basic`, `proficient` |
| `hardware_access` | `simulation_only`, `jetson_kit`, `full_robot_lab` |
| `learning_goal` | `career_transition`, `academic_research`, `hobby` |
| `preferred_language` | `python`, `cpp`, `both` |

**SQL Migration** (`005_create_profiles_table.sql`):
```sql
CREATE TABLE IF NOT EXISTS user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL UNIQUE REFERENCES users(id) ON DELETE CASCADE,
    experience_level VARCHAR(50) CHECK (
        experience_level IS NULL OR
        experience_level IN ('beginner', 'intermediate', 'advanced')
    ),
    ros_familiarity VARCHAR(50) CHECK (
        ros_familiarity IS NULL OR
        ros_familiarity IN ('none', 'basic', 'proficient')
    ),
    hardware_access VARCHAR(50) CHECK (
        hardware_access IS NULL OR
        hardware_access IN ('simulation_only', 'jetson_kit', 'full_robot_lab')
    ),
    learning_goal VARCHAR(50) CHECK (
        learning_goal IS NULL OR
        learning_goal IN ('career_transition', 'academic_research', 'hobby')
    ),
    preferred_language VARCHAR(20) CHECK (
        preferred_language IS NULL OR
        preferred_language IN ('python', 'cpp', 'both')
    ),
    is_complete BOOLEAN NOT NULL DEFAULT FALSE,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_user_profiles_user_id ON user_profiles(user_id);

-- Trigger for is_complete auto-calculation
CREATE OR REPLACE FUNCTION update_profile_completion()
RETURNS TRIGGER AS $$
BEGIN
    NEW.is_complete = (
        NEW.experience_level IS NOT NULL AND
        NEW.ros_familiarity IS NOT NULL AND
        NEW.hardware_access IS NOT NULL AND
        NEW.learning_goal IS NOT NULL AND
        NEW.preferred_language IS NOT NULL
    );
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ language 'plpgsql';

CREATE TRIGGER update_user_profiles_completion
    BEFORE INSERT OR UPDATE ON user_profiles
    FOR EACH ROW
    EXECUTE FUNCTION update_profile_completion();
```

---

### 4. Login Attempts (Security Audit)

**Purpose**: Track login attempts for security auditing and brute-force detection.

**Table**: `login_attempts`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique attempt identifier |
| `email` | VARCHAR(255) | NOT NULL | Attempted email (may not exist) |
| `ip_address` | INET | NOT NULL | Client IP address |
| `user_agent` | VARCHAR(500) | NULL | Browser/client info |
| `success` | BOOLEAN | NOT NULL | Whether login succeeded |
| `failure_reason` | VARCHAR(100) | NULL | Reason for failure (if failed) |
| `created_at` | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Attempt timestamp |

**Failure Reasons**: `invalid_email`, `invalid_password`, `account_locked`, `account_inactive`

**SQL Migration** (`004_create_users_table.sql` - continued):
```sql
CREATE TABLE IF NOT EXISTS login_attempts (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) NOT NULL,
    ip_address INET NOT NULL,
    user_agent VARCHAR(500),
    success BOOLEAN NOT NULL,
    failure_reason VARCHAR(100),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_login_attempts_email ON login_attempts(email);
CREATE INDEX IF NOT EXISTS idx_login_attempts_ip ON login_attempts(ip_address);
CREATE INDEX IF NOT EXISTS idx_login_attempts_created_at ON login_attempts(created_at);

-- Auto-purge old login attempts (keep 30 days for audit)
-- Can be implemented via pg_cron or application scheduled task
```

---

## Existing Table Modifications

### chat_sessions (Existing)

**Current State**: Has `user_id` column already defined but unused.

**Modification**: Add foreign key constraint to link with new `users` table.

**SQL Migration** (`006_link_sessions_to_users.sql`):
```sql
-- Add foreign key constraint to existing user_id column
ALTER TABLE chat_sessions
    ADD CONSTRAINT fk_chat_sessions_user_id
    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE SET NULL;

-- Index for user session lookup
CREATE INDEX IF NOT EXISTS idx_chat_sessions_user_id
    ON chat_sessions(user_id) WHERE user_id IS NOT NULL;
```

---

## Pydantic Models (Backend)

### User Models

```python
# backend/app/models/user.py
from pydantic import BaseModel, EmailStr, Field
from uuid import UUID
from datetime import datetime
from typing import Optional

class UserBase(BaseModel):
    email: EmailStr

class UserCreate(UserBase):
    password: str = Field(..., min_length=8)

class UserLogin(UserBase):
    password: str

class UserResponse(UserBase):
    id: UUID
    created_at: datetime
    is_active: bool
    has_google: bool
    profile_complete: bool

class UserInDB(UserBase):
    id: UUID
    password_hash: Optional[str]
    google_id: Optional[str]
    created_at: datetime
    updated_at: datetime
    last_login_at: Optional[datetime]
    is_active: bool
    failed_login_attempts: int
    locked_until: Optional[datetime]
```

### Profile Models

```python
# backend/app/models/profile.py
from pydantic import BaseModel, Field
from uuid import UUID
from datetime import datetime
from typing import Optional, Literal

ExperienceLevel = Literal["beginner", "intermediate", "advanced"]
ROSFamiliarity = Literal["none", "basic", "proficient"]
HardwareAccess = Literal["simulation_only", "jetson_kit", "full_robot_lab"]
LearningGoal = Literal["career_transition", "academic_research", "hobby"]
PreferredLanguage = Literal["python", "cpp", "both"]

class ProfileBase(BaseModel):
    experience_level: Optional[ExperienceLevel] = None
    ros_familiarity: Optional[ROSFamiliarity] = None
    hardware_access: Optional[HardwareAccess] = None
    learning_goal: Optional[LearningGoal] = None
    preferred_language: Optional[PreferredLanguage] = None

class ProfileCreate(ProfileBase):
    pass

class ProfileUpdate(ProfileBase):
    pass

class ProfileResponse(ProfileBase):
    id: UUID
    user_id: UUID
    is_complete: bool
    created_at: datetime
    updated_at: datetime
```

### Token Models

```python
# backend/app/models/token.py
from pydantic import BaseModel
from uuid import UUID
from datetime import datetime
from typing import Literal

class TokenPayload(BaseModel):
    sub: UUID  # user_id
    email: str
    type: Literal["access", "refresh"]
    iat: datetime
    exp: datetime

class TokenResponse(BaseModel):
    access_token: str
    refresh_token: str
    token_type: str = "bearer"
    expires_in: int  # seconds

class RefreshTokenRequest(BaseModel):
    refresh_token: str
```

---

## TypeScript Types (Frontend)

```typescript
// src/types/auth.ts

export interface User {
  id: string;
  email: string;
  createdAt: string;
  isActive: boolean;
  hasGoogle: boolean;
  profileComplete: boolean;
}

export interface UserProfile {
  id: string;
  userId: string;
  experienceLevel: 'beginner' | 'intermediate' | 'advanced' | null;
  rosFamiliarity: 'none' | 'basic' | 'proficient' | null;
  hardwareAccess: 'simulation_only' | 'jetson_kit' | 'full_robot_lab' | null;
  learningGoal: 'career_transition' | 'academic_research' | 'hobby' | null;
  preferredLanguage: 'python' | 'cpp' | 'both' | null;
  isComplete: boolean;
  createdAt: string;
  updatedAt: string;
}

export interface AuthState {
  isAuthenticated: boolean;
  user: User | null;
  profile: UserProfile | null;
  isLoading: boolean;
}

export interface LoginRequest {
  email: string;
  password: string;
}

export interface SignupRequest {
  email: string;
  password: string;
}

export interface AuthResponse {
  user: User;
  profile: UserProfile | null;
}
```

---

## Data Flow

### Signup Flow
```
1. User submits email + password
2. Validate email format and password rules
3. Check email not already registered
4. Hash password with bcrypt (cost=12)
5. INSERT into users table
6. INSERT empty profile into user_profiles table
7. Generate access + refresh tokens
8. Set httpOnly cookies
9. Return user + profile response
10. Frontend shows profile wizard
```

### Login Flow
```
1. User submits email + password
2. SELECT user by email
3. Check account not locked (locked_until < NOW())
4. Verify password with bcrypt
5. If failed: increment failed_login_attempts, check lockout threshold
6. If success: reset failed_login_attempts, update last_login_at
7. Generate access + refresh tokens
8. INSERT refresh token hash into refresh_tokens
9. Set httpOnly cookies
10. Return user + profile response
```

### Token Refresh Flow
```
1. Frontend calls /auth/refresh with refresh token (from cookie)
2. Hash received token, lookup in refresh_tokens table
3. Verify not expired and not revoked
4. Generate new access token
5. Optionally rotate refresh token
6. Set new cookies
7. Return success
```

### Logout Flow
```
1. Frontend calls /auth/logout
2. Get refresh token from cookie
3. UPDATE refresh_tokens SET revoked_at = NOW() WHERE token_hash = hash
4. Clear httpOnly cookies
5. Return success
```

---

## Storage Estimates

| Table | Row Size (avg) | Expected Rows (1yr) | Total Size |
|-------|---------------|---------------------|------------|
| users | ~500 bytes | 10,000 | ~5 MB |
| refresh_tokens | ~200 bytes | 50,000 (active sessions) | ~10 MB |
| user_profiles | ~300 bytes | 10,000 | ~3 MB |
| login_attempts | ~150 bytes | 100,000 (30-day retention) | ~15 MB |

**Total Estimated**: ~33 MB (well within Neon 0.5GB free tier)

---

## References

- Spec Requirements: FR-001 to FR-019 (Account, Authentication, Session, Profile)
- Constitution: Security Principles (lines 833-879)
- Existing Schema: `backend/app/migrations/001_create_chat_tables.sql`
