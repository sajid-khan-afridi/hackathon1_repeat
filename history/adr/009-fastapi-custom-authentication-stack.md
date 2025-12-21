# ADR-009: FastAPI Custom Authentication Stack

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-18
- **Feature:** 001-user-auth
- **Context:** Need to implement user authentication for the Physical AI & Humanoid Robotics Textbook to enable user identity, session management, and differentiated rate limits for Phase 4B personalization. Requirements include: email/password and Google OAuth sign-up/login, JWT-based sessions with 24h access tokens, secure token storage, CSRF protection, and integration with existing FastAPI backend on Railway and Neon PostgreSQL database. Must meet <500ms p95 latency requirement and OWASP Top 10 security compliance.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: YES - Long-term consequence for all authenticated endpoints, middleware patterns, frontend auth state, and security posture
     2) Alternatives: YES - Better Auth, FastAPI-Users, various session strategies explicitly considered with tradeoffs
     3) Scope: YES - Cross-cutting concern affecting backend services, React frontend, database schema, and rate limiting
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Adopt a custom FastAPI authentication implementation using PyJWT for token handling, bcrypt for password hashing, and stateless JWT with database-stored refresh tokens for session management.

- **Auth Library**: Custom PyJWT implementation (~300 lines)
- **Password Hashing**: bcrypt with cost factor 12 (via passlib)
- **JWT Signing**: RS256 (RSA + SHA-256) with asymmetric key pair
- **Access Tokens**: 24-hour expiry, stateless validation
- **Refresh Tokens**: 30-day expiry, stored as SHA-256 hashes in PostgreSQL
- **Token Storage**: httpOnly secure cookies with SameSite=Strict
- **CSRF Protection**: Double-submit pattern (cookie + header)
- **OAuth Integration**: Google OAuth 2.0 with auto-link by email
- **Session Model**: Stateless JWT for access, database-backed refresh token revocation

## Consequences

### Positive

- **Pattern Consistency**: Custom auth fits existing asyncpg/raw SQL patterns in the codebase
- **Minimal Dependencies**: Only 3 new packages (passlib[bcrypt], PyJWT, cryptography) vs framework overhead
- **Full Control**: Complete control over token structure, validation logic, and security behaviors
- **Performance**: Stateless access token validation avoids database lookup per request
- **Security**: RS256 asymmetric signing allows public key distribution for verification
- **Constitution Alignment**: Follows "Smallest Viable Change" principle (~300 lines vs new framework)
- **Migration Path**: Same bcrypt algorithm used by Better Auth if future migration needed
- **Concurrent Sessions**: Stateless JWT naturally supports multiple device logins (FR-014)

### Negative

- **Maintenance Burden**: Custom code requires ongoing security review and updates
- **No Community Patches**: Security vulnerabilities require manual discovery and patching
- **Limited Features**: No built-in email verification, password reset (out of scope but harder to add)
- **Testing Responsibility**: Must write comprehensive auth tests (no framework-provided test utilities)
- **Documentation**: Must document custom auth flows for future contributors
- **Token Revocation Delay**: Access tokens valid until expiry (24h) after logout/account disable

## Alternatives Considered

**Alternative Stack A: Better Auth with Next.js Frontend**
- Library: Better Auth (TypeScript-first auth library)
- Benefits: Modern, full-featured, handles email verification, password reset, multiple OAuth providers
- Rejected: Requires Node.js runtime which conflicts with Python FastAPI backend; would require architectural change to API-first pattern

**Alternative Stack B: FastAPI-Users Library**
- Library: FastAPI-Users with SQLAlchemy backend
- Benefits: Native FastAPI integration, pre-built routes, well-documented, community-maintained
- Rejected: Adds SQLAlchemy ORM dependency which conflicts with existing raw SQL migration approach and asyncpg patterns; opinionated structure may conflict with existing services

**Alternative Stack C: Stateful Session Store (Redis)**
- Pattern: Server-side sessions stored in Redis with session ID in cookie
- Benefits: Instant token revocation, session data storage, familiar pattern
- Rejected: Adds Redis dependency (complexity, cost), database lookup per request (latency), over-engineering for expected scale

**Alternative Stack D: Argon2 Password Hashing**
- Algorithm: Argon2id (PHC competition winner, memory-hard)
- Benefits: Best-in-class security, resistant to GPU attacks
- Rejected: Constitution specifies bcrypt (cost=12), ~500ms hash time may exceed performance budget, bcrypt is sufficient for expected threat model

**Alternative Token Storage: localStorage + Bearer Header**
- Pattern: Store JWT in localStorage, send via Authorization header
- Benefits: Simpler CORS, no cookie domain restrictions
- Rejected: XSS vulnerability (tokens accessible to JavaScript), FR-025 requires httpOnly cookies for security

## References

- Feature Spec: specs/001-user-auth/spec.md
- Implementation Plan: specs/001-user-auth/plan.md (lines 199-209)
- Research: specs/001-user-auth/research.md (Sections 1, 2, 3, 7)
- Data Model: specs/001-user-auth/data-model.md
- API Contract: specs/001-user-auth/contracts/auth-api.yaml
- Related ADRs: ADR-008 (Chat History Schema - shares Neon PostgreSQL database)
