---
id: 001
title: Create User Authentication Specification
stage: spec
date: 2025-12-17
surface: agent
model: claude-opus-4-5-20251101
feature: 001-user-auth
branch: 001-user-auth
user: system
command: /sp.specify
labels: ["authentication", "user-auth", "phase-4a", "specification"]
links:
  spec: specs/001-user-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-user-auth/spec.md
  - specs/001-user-auth/checklists/requirements.md
tests:
  - N/A (specification phase)
---

## Prompt

Phase 4A: User Authentication System

## Feature Overview
Implement a complete authentication system for the Physical AI & Humanoid Robotics Textbook using Better Auth with email and Google OAuth. This enables user identity, session management, and profile collection to unlock Phase 4B personalization features.

## Dependencies
- Phase 1 (Docusaurus infrastructure) - Complete
- Phase 2 (MDX textbook chapters) - Complete
- Phase 3 (RAG chatbot backend on Railway) - Complete
- Backend: FastAPI on Railway (https://hackathon1repeat-production.up.railway.app)
- Database: Neon PostgreSQL (existing tables: chat_history, user_bookmarks)
- Frontend: Docusaurus v3 with React/TypeScript at GitHub Pages

## User Stories

### P1: Email Signup & Login
As a new user, I want to create an account with email/password so I can access personalized content.
- Signup form with email, password (min 8 chars, 1 uppercase, 1 number)
- Login form with email/password
- Password hashed with bcrypt (cost factor 12)
- Email verification optional for MVP

### P2: Google OAuth Login
As a returning user, I want to sign in with Google for faster access.
- Google OAuth 2.0 integration
- Auto-link if same email exists
- Handle OAuth callback and error states

### P3: Profile Collection Wizard
As a new user after signup, I want to answer 5 questions so the content adapts to my level.
Profile questions (from user-profiling skill):
1. Programming experience (Beginner/Intermediate/Advanced)
2. ROS familiarity (None/Basic/Proficient)
3. Hardware access (Simulation Only/Jetson Kit/Full Robot Lab)
4. Learning goal (Career/Research/Hobby)
5. Preferred code examples (Python/C++/Both)

### P4: Session Management
As a logged-in user, I want my session to persist and auto-refresh.
- JWT access tokens (24h expiry, RS256 signed)
- Refresh tokens (30d expiry)
- Logout clears session
- Protected routes redirect to login

### P5: Rate Limiting Tiers
As an authenticated user, I get higher chatbot query limits.
- Anonymous: 10 queries/hour
- Authenticated: 50 queries/hour
- Rate limit headers in API responses

## Database Schema (Neon PostgreSQL)
Tables to create:
- users (id, email, password_hash, google_id, created_at, last_login)
- sessions (id, user_id, token, refresh_token, expires_at, created_at)
- user_profiles (user_id FK, experience_level, ros_familiarity, hardware_access, learning_goal, preferred_language, profile_hash, created_at, updated_at)

## Security Requirements (OWASP Top 10 compliance)
- Bcrypt password hashing (cost 12)
- JWT RS256 signing (NOT HS256)
- CSRF tokens for state-changing requests
- Rate limiting: 10 failed logins -> 15min lockout per IP
- No secrets in code (use .env)
- Input sanitization (prevent XSS, SQL injection)
- Secure cookies (httpOnly, secure, sameSite=strict)
- HTTPS-only API calls

## Quality Gates (Exit Criteria from Constitution)
- Email signup, login, logout work (100% success on test scenarios)
- Google OAuth functional
- JWT tokens expire after 24h and refresh correctly
- OWASP Top 10 checklist passed (security audit)
- Rate limiting functional (tested with load tool)
- 80% test coverage on auth flows
- PHR created documenting auth implementation

## Skills to Use
- better-auth-setup - Authentication configuration
- user-profiling - Post-signup profile collection
- neon-postgres - Database schema and operations

## Out of Scope
- Email verification (defer to later)
- Password reset flow (defer to later)
- Account deletion (defer to later)
- Multi-factor authentication (defer to later)
- Social login beyond Google (GitHub, etc.)

## Response snapshot

Created complete specification for Phase 4A User Authentication System:

**Branch**: `001-user-auth`
**Spec File**: `specs/001-user-auth/spec.md`

### Key Deliverables:
1. **5 User Stories (P1-P5)**: Email signup/login, Google OAuth, Profile wizard, Session management, Rate limiting
2. **28 Functional Requirements**: Covering account management, authentication, sessions, profile, rate limiting, and security
3. **10 Success Criteria**: Measurable, technology-agnostic outcomes
4. **3 Key Entities**: User, Session, UserProfile
5. **Quality Checklist**: All 16 validation items passed

### Spec Highlights:
- Clear prioritization (P1 email auth is MVP, P5 rate limiting is last)
- OWASP security requirements embedded in functional requirements
- Technology-agnostic success criteria (e.g., "signup in under 60 seconds")
- Explicit out-of-scope to prevent feature creep

## Outcome

- Impact: Created comprehensive Phase 4A specification enabling authentication system development
- Tests: N/A (specification phase - tests defined in acceptance scenarios)
- Files: 2 files created (spec.md, checklists/requirements.md)
- Next prompts: `/sp.clarify` to refine or `/sp.plan` to create architecture plan
- Reflection: User-provided detailed input enabled complete spec with no clarification markers needed

## Evaluation notes (flywheel)

- Failure modes observed: None - user input was comprehensive
- Graders run and results (PASS/FAIL): Quality checklist 16/16 PASS
- Prompt variant (if applicable): Standard /sp.specify with detailed user input
- Next experiment (smallest change to try): Monitor if detailed user input consistently reduces clarification needs
