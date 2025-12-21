---
id: 004
title: Document FastAPI Custom Auth ADR
stage: plan
date: 2025-12-18
surface: agent
model: claude-opus-4-5-20251101
feature: 001-user-auth
branch: 001-user-auth
user: system
command: /sp.adr fastapi-custom-auth
labels: ["adr", "architecture", "authentication", "fastapi", "jwt"]
links:
  spec: specs/001-user-auth/spec.md
  ticket: null
  adr: history/adr/009-fastapi-custom-authentication-stack.md
  pr: null
files:
  - history/adr/009-fastapi-custom-authentication-stack.md
tests:
  - null
---

## Prompt

/sp.adr fastapi-custom-auth

The user invoked the ADR command to document the architecturally significant decision identified in plan.md regarding the choice of custom FastAPI authentication implementation over Better Auth or FastAPI-Users library alternatives.

## Response snapshot

Created ADR-009: FastAPI Custom Authentication Stack

The ADR documents the decision to implement custom authentication using:
- Custom PyJWT implementation (~300 lines)
- bcrypt with cost factor 12 for password hashing
- RS256 JWT signing with asymmetric keys
- 24h access tokens, 30-day refresh tokens
- httpOnly secure cookies with SameSite=Strict
- Google OAuth 2.0 with auto-link by email

Five alternatives were considered and rejected:
1. Better Auth - requires Node.js runtime (conflicts with Python backend)
2. FastAPI-Users - adds SQLAlchemy ORM dependency (conflicts with raw SQL pattern)
3. Stateful Redis sessions - over-engineering for expected scale
4. Argon2 hashing - constitution specifies bcrypt
5. localStorage tokens - XSS vulnerability, FR-025 requires httpOnly

## Outcome

- Impact: Created ADR-009 documenting authentication architecture decision for 001-user-auth feature
- Tests: N/A - documentation artifact, no code changes
- Files: 1 created (history/adr/009-fastapi-custom-authentication-stack.md)
- Next prompts: /sp.tasks to generate implementation tasks based on plan.md and ADR-009
- Reflection: Decision cluster approach properly grouped auth library + hashing + JWT + session model into single cohesive ADR

## Evaluation notes (flywheel)

- Failure modes observed: None - decision was properly clustered, alternatives documented with rationale
- Graders run and results (PASS/FAIL):
  - Clustering check: PASS (not over-granular, groups related auth decisions)
  - Alternatives check: PASS (5 alternatives with explicit rejection rationale)
  - Consequences check: PASS (8 positive, 6 negative outcomes documented)
  - References check: PASS (links to plan.md, research.md, data-model.md, contracts)
- Prompt variant (if applicable): Standard /sp.adr command execution
- Next experiment (smallest change to try): N/A - standard workflow executed successfully
