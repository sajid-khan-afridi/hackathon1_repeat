---
id: 003
title: User Auth Implementation Plan
stage: plan
date: 2025-12-17
surface: agent
model: claude-opus-4-5-20251101
feature: 001-user-auth
branch: 001-user-auth
user: architect
command: /sp.plan
labels: ["authentication", "planning", "phase-4a", "fastapi", "jwt", "oauth"]
links:
  spec: specs/001-user-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-user-auth/plan.md
  - specs/001-user-auth/research.md
  - specs/001-user-auth/data-model.md
  - specs/001-user-auth/quickstart.md
  - specs/001-user-auth/contracts/auth-api.yaml
tests:
  - null
---

## Prompt

```text
/sp.plan according to @specs\001-user-auth\spec.md
```

Execute the implementation planning workflow using the plan template to generate design artifacts for the User Authentication System (Phase 4A).

## Response snapshot

Executed the /sp.plan workflow for 001-user-auth feature, generating comprehensive design artifacts:

**Phase 0 (Research)**:
- Created `research.md` with technology decisions:
  - Custom PyJWT implementation (over Better Auth due to Python backend)
  - bcrypt password hashing (cost factor 12)
  - RS256 JWT signing algorithm
  - Stateless JWT sessions with refresh token revocation
  - Google OAuth 2.0 integration via httpx
  - Account auto-linking by email (FR-008)

**Phase 1 (Design)**:
- Created `data-model.md` with entity schemas:
  - Users table (id, email, password_hash, google_id, lockout fields)
  - Refresh tokens table (for session management)
  - User profiles table (5 profile questions per spec)
  - Login attempts table (security audit)
  - SQL migrations and Pydantic models

- Created `contracts/auth-api.yaml` (OpenAPI 3.1):
  - POST /auth/signup, /auth/login, /auth/logout, /auth/refresh
  - GET/POST /auth/google, /auth/google/callback
  - GET/PUT /users/profile
  - GET/DELETE /users/sessions

- Created `quickstart.md` setup guide:
  - RSA key generation for JWT
  - Google OAuth configuration
  - Environment variable reference
  - Database migration instructions

**Plan.md** updated with:
- Technical Context (Python 3.11, FastAPI, Neon PostgreSQL)
- Constitution Check (Phase 4A quality gates)
- Project Structure (backend/frontend file layout)
- Post-Design Constitution Check (compliance verified)

## Outcome

- ✅ Impact: High - Comprehensive Phase 4A design artifacts enabling implementation
- 🧪 Tests: N/A (planning phase - tests defined in contracts)
- 📁 Files: 5 files created/updated (plan.md, research.md, data-model.md, quickstart.md, auth-api.yaml)
- 🔁 Next prompts: /sp.tasks to generate implementation task list
- 🧠 Reflection: Planning identified need for custom auth vs Better Auth adaptation due to Python backend

## Evaluation notes (flywheel)

- Failure modes observed: None - workflow completed successfully
- Graders run and results (PASS/FAIL): Constitution check PASS, quality gates aligned
- Prompt variant (if applicable): Standard /sp.plan workflow
- Next experiment (smallest change to try): N/A - proceed to /sp.tasks for implementation tasks
