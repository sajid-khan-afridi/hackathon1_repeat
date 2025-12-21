# Implementation Plan: User Authentication System

**Branch**: `004-user-auth` | **Date**: 2025-12-17 | **Spec**: [specs/004-user-auth/spec.md](spec.md)
**Input**: Feature specification from `/specs/004-user-auth/spec.md`

<!--
IMPORTANT: Branch numbering follows phase-based numbering from the constitution:
- Phase 1 (Book Infrastructure): 001-*
- Phase 2 (Content Creation): 002-*
- Phase 3 (RAG Chatbot Core): 003-*
- Phase 4A (Authentication): 004-*
- Phase 4B (Personalization): 005-*
- Phase 5 (Translation): 006-*
- Phase 6 (Integration & Deployment): 007-*
-->

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement user authentication for the Physical AI & Humanoid Robotics Textbook using email/password and Google OAuth. The system will integrate Better Auth into the existing FastAPI backend (Railway) with Neon PostgreSQL for user storage. Frontend React components will be added to Docusaurus for login/signup forms and profile collection wizard. This enables user identity, session management, and differentiated rate limits for Phase 4B personalization.

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript 5.6 (frontend)
**Primary Dependencies**: FastAPI 0.115, Better Auth (adapted for FastAPI), React 19, Docusaurus 3.9
**Storage**: Neon PostgreSQL (existing), asyncpg for async database operations
**Testing**: pytest + pytest-asyncio (backend), Jest + Playwright (frontend)
**Target Platform**: Railway (backend), GitHub Pages (frontend static site)
**Project Type**: Web application (FastAPI backend + Docusaurus/React frontend)
**Performance Goals**: <500ms p95 latency for auth endpoints (FR-029)
**Constraints**: <500ms p95 auth endpoints, Neon free tier (0.5GB), secure cookies (httpOnly, secure flags)
**Scale/Scope**: 100 concurrent auth requests, 10k users capacity, 5 profile questions

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

<!--
  QUALITY GATES THREE-TIER HIERARCHY (per ADR-003):

  Tier 1: Constitution Detailed Exit Criteria (SOURCE OF TRUTH)
    Location: .specify/memory/constitution.md Section "QUALITY GATES"
    Reference: Lines 958-965 (Phase 4A Exit Criteria)

  Tier 2: Constitution Summary (INFORMATIONAL ONLY)
    Location: .specify/memory/constitution.md Section "Core Principles > Quality Over Speed"
    Reference: Lines 85-92
    Note: High-level summaries; NOT authoritative source

  Tier 3: Plan Phase Constitution Check (IMPLEMENTATION VIEW - THIS FILE)
    Rule: MUST exactly match Tier 1 (Constitution Detailed Exit Criteria)
    Any deviation is a conflict; Tier 1 always wins
-->

### Phase 4A Quality Gates (from Constitution)
<!-- Reference: .specify/memory/constitution.md lines 958-965 -->
- [ ] Email signup, login, logout work (100% success on test scenarios)
- [ ] Google OAuth functional
- [ ] JWT tokens expire after 24h and refresh correctly
- [ ] OWASP Top 10 checklist passed (security audit)
- [ ] Rate limiting functional (tested with load tool)
- [ ] PHR created documenting auth implementation

### Core Principles Alignment

**✅ Quality Over Speed**: 80% test coverage on all auth flows (SC-010); 100% success rate on valid credentials (SC-003); comprehensive security audit (OWASP Top 10)
**✅ Smallest Viable Change**: Email + Google OAuth only (no GitHub/Facebook per spec out-of-scope); skip profile wizard option; no email verification or password reset (Phase 1 MVP)
**✅ Security by Default**: Passwords hashed with bcrypt (cost 12); JWT signed with RS256; httpOnly secure cookies; CSRF protection; rate limiting (10 failed attempts → 15min lockout); input sanitization
**✅ Observability & Measurability**: Structured logging for auth events (FR-028); p95 latency < 500ms measured; rate limit headers in responses (FR-022); correlation IDs for request tracing
**✅ Accessibility & Inclusivity**: Login/signup forms keyboard navigable; visible focus indicators; WCAG 2.1 AA color contrast; inline error messages below fields (FR-031)
**✅ Free Tier Sustainability**: Users table in existing Neon PostgreSQL (0.5GB limit); minimal additional storage overhead; rate limiting protects API quotas

### Agent Ownership
**Primary**: AuthEngineer Agent (owns: `better-auth-setup`, `user-profiling`, `neon-postgres`)
**Support**: DocusaurusBuilder Agent (React auth components), RAGArchitect Agent (rate limiting integration)
**Coordinator**: Orchestrator Agent (quality gate approval)

### Complexity Violations
*None identified - Phase follows YAGNI principle:*
- No email verification workflow (out of scope)
- No password reset flow (out of scope)
- No MFA (out of scope)
- No additional OAuth providers beyond Google (out of scope)
- Simple JWT-based sessions (no complex session store)

## Project Structure

### Documentation (this feature)

```text
specs/004-user-auth/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── auth-api.yaml    # OpenAPI 3.1 specification
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Backend (FastAPI on Railway)
backend/
├── app/
│   ├── main.py                    # FastAPI app entry point (existing)
│   ├── config.py                  # Environment configuration (existing)
│   ├── models/
│   │   ├── __init__.py
│   │   ├── user.py                # NEW: User model (Pydantic)
│   │   └── profile.py             # NEW: UserProfile model
│   ├── services/
│   │   ├── auth_service.py        # NEW: Authentication business logic
│   │   ├── user_service.py        # NEW: User CRUD operations
│   │   ├── profile_service.py     # NEW: Profile management
│   │   └── chat_service.py        # MODIFY: Link sessions to users
│   ├── routers/
│   │   ├── auth.py                # NEW: Auth endpoints (signup, login, logout, refresh)
│   │   ├── users.py               # NEW: User profile endpoints
│   │   ├── oauth.py               # NEW: Google OAuth callback
│   │   └── query.py               # MODIFY: Add auth-based rate limiting
│   ├── middleware/
│   │   ├── auth.py                # NEW: JWT validation middleware
│   │   └── rate_limit.py          # MODIFY: Auth-aware rate limiting
│   └── migrations/
│       ├── 004_create_users_table.sql      # NEW: Users table
│       └── 005_create_profiles_table.sql   # NEW: User profiles table
└── tests/
    ├── unit/
    │   ├── test_auth_service.py   # NEW: Auth service unit tests
    │   └── test_user_service.py   # NEW: User service unit tests
    └── integration/
        └── test_auth_flow.py      # NEW: End-to-end auth flow tests

# Frontend (Docusaurus + React)
src/
├── components/
│   ├── Auth/
│   │   ├── LoginForm.tsx          # NEW: Email/password login
│   │   ├── SignupForm.tsx         # NEW: Email/password registration
│   │   ├── GoogleLoginButton.tsx  # NEW: Google OAuth button
│   │   ├── LogoutButton.tsx       # NEW: Logout functionality
│   │   └── AuthModal.tsx          # NEW: Modal wrapper for auth forms
│   └── Profile/
│       ├── ProfileWizard.tsx      # NEW: 5-question wizard
│       ├── ProfileBanner.tsx      # NEW: Incomplete profile reminder
│       └── ProfileSettings.tsx    # NEW: Edit profile page
├── context/
│   ├── AuthContext.tsx            # NEW: Authentication state management
│   └── UserContext.tsx            # MODIFY: Link to auth context
├── hooks/
│   └── useAuth.ts                 # NEW: Auth hook for components
├── pages/
│   ├── login.tsx                  # NEW: Login page
│   ├── signup.tsx                 # NEW: Signup page
│   └── profile.tsx                # NEW: Profile settings page
└── services/
    └── authApi.ts                 # NEW: API client for auth endpoints
```

**Structure Decision**: Web application structure selected. Extends existing FastAPI backend with auth routers/services and Docusaurus frontend with React auth components. Database migrations added to existing migration system.

## Complexity Tracking

> **No complexity violations identified.** Phase follows YAGNI principle.

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| *None* | - | - |

## Post-Design Constitution Check (Phase 4A Complete)

### ✅ All Quality Gates Remain Valid
<!-- Copy same checklist from Constitution Check above -->
- [ ] Email signup, login, logout work (100% success on test scenarios)
- [ ] Google OAuth functional
- [ ] JWT tokens expire after 24h and refresh correctly
- [ ] OWASP Top 10 checklist passed (security audit)
- [ ] Rate limiting functional (tested with load tool)
- [ ] PHR created documenting auth implementation

### ✅ Design Artifacts Generated
- ✅ `research.md` - Technology decisions: Better Auth adaptation for FastAPI, bcrypt password hashing, RS256 JWT signing
- ✅ `data-model.md` - Users, Sessions, UserProfiles entity schemas with relationships
- ✅ `quickstart.md` - Local development setup with Google OAuth credentials, database migrations
- ✅ `contracts/auth-api.yaml` - OpenAPI 3.1 spec for auth endpoints (signup, login, logout, refresh, OAuth callback, profile CRUD)

### ✅ No New Complexity Violations

**Analysis**:
- **YAGNI Compliance**: Direct asyncpg queries (no ORM abstraction); simple JWT-based sessions (no Redis session store); inline validation (no validation framework)
- **Security**: bcrypt cost factor 12; RS256 JWT signing; httpOnly secure cookies; CSRF tokens; rate limiting; input sanitization
- **Performance**: Connection pooling (2-10 connections); <500ms p95 target; efficient database indexes on email and user_id
- **Accessibility**: Keyboard-navigable forms; visible focus indicators; inline error messages; WCAG 2.1 AA color contrast
- **Free Tier**: Users and profiles fit within Neon 0.5GB limit; rate limiting protects OpenAI API quotas

### ✅ Recommended ADRs

**Decision Detected**: Custom FastAPI Authentication vs Better Auth Adaptation
- **Impact**: Long-term: Authentication patterns, token management, future OAuth provider additions
- **Alternatives**: (1) Better Auth with Next.js frontend, (2) Custom FastAPI auth with PyJWT, (3) FastAPI-Users library
- **Scope**: Cross-cutting - affects all authenticated endpoints, frontend auth state, rate limiting

📋 Architectural decision detected: **FastAPI Custom Authentication with PyJWT**
   Document reasoning and tradeoffs? Run `/sp.adr fastapi-custom-auth`

*Waiting for user consent before creating ADRs.*

---
