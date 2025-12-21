# Tasks: User Authentication System

**Input**: Design documents from `/specs/004-user-auth/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/auth-api.yaml

**Tests**: Tests are NOT explicitly requested in the feature specification. Test tasks are OMITTED per template guidelines.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/app/` for FastAPI on Railway
- **Frontend**: `src/` for Docusaurus/React on GitHub Pages
- **Migrations**: `backend/app/migrations/`
- **Tests**: `backend/tests/` and frontend component tests

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, dependencies, and basic structure

- [X] T001 Add authentication dependencies to backend/requirements.txt (passlib[bcrypt], PyJWT, cryptography)
- [X] T002 [P] Add frontend dependency jwt-decode to package.json
- [X] T003 [P] Update backend/app/config.py to load JWT and OAuth environment variables
- [X] T004 [P] Create backend/app/models/__init__.py module structure
- [X] T005 [P] Create backend/app/services/__init__.py module structure
- [X] T006 [P] Create backend/app/routers/__init__.py module structure
- [X] T007 [P] Create backend/app/middleware/__init__.py module structure
- [X] T008 [P] Create src/components/Auth/ directory structure
- [X] T009 [P] Create src/components/Profile/ directory structure
- [X] T010 [P] Create src/types/auth.ts with TypeScript type definitions from data-model.md

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

### Database Schema

- [X] T011 Create backend/app/migrations/004_create_users_table.sql (users, refresh_tokens, login_attempts tables per data-model.md)
- [X] T012 Create backend/app/migrations/005_create_profiles_table.sql (user_profiles table per data-model.md)
- [X] T013 Create backend/app/migrations/006_link_sessions_to_users.sql (FK constraint on chat_sessions.user_id)
- [X] T014 Run migrations via backend/run_migrations.py and verify tables exist

### Core Models (Pydantic)

- [X] T015 [P] Create backend/app/models/user.py (UserBase, UserCreate, UserLogin, UserResponse, UserInDB per data-model.md)
- [X] T016 [P] Create backend/app/models/profile.py (ProfileBase, ProfileCreate, ProfileUpdate, ProfileResponse per data-model.md)
- [X] T017 [P] Create backend/app/models/token.py (TokenPayload, TokenResponse, RefreshTokenRequest per data-model.md)

### JWT Authentication Framework

- [X] T018 Create backend/app/services/jwt_service.py (RS256 token generation, verification, cookie helpers per research.md)
- [X] T019 Create backend/app/middleware/auth.py (JWT validation middleware, get_current_user dependency)

### Password Security

- [X] T020 Create backend/app/services/password_service.py (bcrypt hashing cost=12, verification per research.md)

### CORS and Security Configuration

- [X] T021 Update backend/app/main.py to configure CORS with credentials support for auth cookies
- [X] T022 [P] Generate RSA key pair for JWT signing per quickstart.md instructions

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Email Signup & Login (Priority: P1)

**Goal**: Enable users to create accounts with email/password and log in

**Independent Test**: Create account via /auth/signup, logout, login via /auth/login, verify session established

**Endpoints**: POST /auth/signup, POST /auth/login, POST /auth/logout, POST /auth/refresh, GET /auth/me

### Implementation for User Story 1

- [X] T023 [US1] Create backend/app/services/user_service.py (create_user, get_user_by_email, update_last_login, check_account_lock)
- [X] T024 [US1] Create backend/app/services/auth_service.py (signup, login, logout, refresh_token, validate_password per FR-003)
- [X] T025 [US1] Create backend/app/routers/auth.py with /auth/signup endpoint (FR-001, FR-002, FR-003, FR-004)
- [X] T026 [US1] Add /auth/login endpoint to backend/app/routers/auth.py (FR-006, FR-024 rate limiting)
- [X] T027 [US1] Add /auth/logout endpoint to backend/app/routers/auth.py (FR-013)
- [X] T028 [US1] Add /auth/refresh endpoint to backend/app/routers/auth.py (FR-010, FR-011, FR-012)
- [X] T029 [US1] Add /auth/me endpoint to backend/app/routers/auth.py (get current user info)
- [X] T030 [US1] Register auth router in backend/app/main.py with /auth prefix
- [X] T031 [US1] Implement login attempt tracking in backend/app/services/auth_service.py (FR-024, FR-028)
- [X] T032 [P] [US1] Create src/services/authApi.ts (signup, login, logout, refresh, getCurrentUser API client)
- [X] T033 [P] [US1] Create src/context/AuthContext.tsx (authentication state management, token refresh)
- [X] T034 [P] [US1] Create src/hooks/useAuth.ts (auth hook exposing login, logout, isAuthenticated)
- [X] T035 [US1] Create src/components/Auth/LoginForm.tsx (email/password form with validation per FR-030, FR-031)
- [X] T036 [US1] Create src/components/Auth/SignupForm.tsx (email/password form with password requirements per FR-003)
- [X] T037 [US1] Create src/components/Auth/AuthModal.tsx (modal wrapper for login/signup forms)
- [X] T038 [US1] Create src/components/Auth/LogoutButton.tsx (logout functionality)
- [X] T039 [US1] Create src/pages/login.tsx (login page routing to LoginForm)
- [X] T040 [US1] Create src/pages/signup.tsx (signup page routing to SignupForm)
- [X] T041 [US1] Add AuthContext provider to Docusaurus root component (src/theme/Root.tsx or equivalent)

**Checkpoint**: User Story 1 (Email Signup & Login) fully functional - can create account, login, logout independently

---

## Phase 4: User Story 2 - Google OAuth Login (Priority: P2)

**Goal**: Enable users to sign in with Google account for passwordless access

**Independent Test**: Click Google login, complete OAuth flow, verify session established and account created/linked

**Endpoints**: GET /auth/google, GET /auth/google/callback

### Implementation for User Story 2

- [X] T042 [US2] Create backend/app/services/oauth_service.py (Google OAuth token exchange, user info extraction)
- [X] T043 [US2] Create backend/app/routers/oauth.py with /auth/google endpoint (redirect to Google consent)
- [X] T044 [US2] Add /auth/google/callback endpoint to backend/app/routers/oauth.py (handle callback, create/link user per FR-008)
- [X] T045 [US2] Register oauth router in backend/app/main.py with /auth prefix
- [X] T046 [US2] Update backend/app/services/user_service.py with get_user_by_google_id and link_google_account functions
- [X] T047 [P] [US2] Create src/components/Auth/GoogleLoginButton.tsx (initiates OAuth flow)
- [X] T048 [US2] Update src/components/Auth/LoginForm.tsx to include GoogleLoginButton
- [X] T049 [US2] Update src/components/Auth/SignupForm.tsx to include GoogleLoginButton
- [X] T050 [US2] Handle OAuth callback redirect in src/pages/login.tsx (parse tokens from redirect)

**Checkpoint**: User Story 2 (Google OAuth) fully functional - can login via Google independently

---

## Phase 5: User Story 3 - Profile Collection Wizard (Priority: P3)

**Goal**: Collect user learning preferences via 5-question wizard after signup

**Independent Test**: Complete signup, presented with wizard, answer questions, verify profile saved

**Endpoints**: GET /users/profile, PUT /users/profile, POST /users/profile/skip

### Implementation for User Story 3

- [X] T051 [US3] Create backend/app/services/profile_service.py (create_profile, get_profile, update_profile, skip_profile)
- [X] T052 [US3] Create backend/app/routers/users.py with /users/profile GET endpoint (FR-017)
- [X] T053 [US3] Add /users/profile PUT endpoint to backend/app/routers/users.py (FR-017, FR-018)
- [X] T054 [US3] Add /users/profile/skip POST endpoint to backend/app/routers/users.py (FR-016)
- [X] T055 [US3] Register users router in backend/app/main.py with /users prefix
- [X] T056 [P] [US3] Create src/components/Profile/ProfileWizard.tsx (5-question wizard per spec Profile Questions)
- [X] T057 [P] [US3] Create src/components/Profile/ProfileBanner.tsx (incomplete profile reminder per FR-019)
- [X] T058 [P] [US3] Create src/components/Profile/ProfileSettings.tsx (edit profile form per FR-018)
- [X] T059 [US3] Create src/pages/profile.tsx (profile settings page routing)
- [X] T060 [US3] Update src/context/AuthContext.tsx to fetch and store profile, trigger wizard after signup
- [X] T061 [US3] Add ProfileBanner to Docusaurus layout for incomplete profiles

**Checkpoint**: User Story 3 (Profile Wizard) fully functional - can complete wizard, skip, and edit profile independently

---

## Phase 6: User Story 4 - Session Persistence (Priority: P4)

**Goal**: Sessions persist across browser sessions with silent token refresh

**Independent Test**: Login, close browser, reopen, verify still logged in; wait for token expiry, verify silent refresh

**Endpoints**: (Uses existing /auth/refresh from US1)

### Implementation for User Story 4

- [X] T062 [US4] Update src/context/AuthContext.tsx to check auth status on app load via /auth/me
- [X] T063 [US4] Implement token refresh interceptor in src/services/authApi.ts (silent refresh on 401)
- [X] T064 [US4] Add refresh token rotation logic to backend/app/services/auth_service.py (optional security enhancement)
- [X] T065 [US4] Configure cookie expiry (access: 24h, refresh: 30d) in backend/app/services/jwt_service.py per FR-011, FR-012

**Checkpoint**: User Story 4 (Session Persistence) fully functional - sessions persist across browser sessions

---

## Phase 7: User Story 5 - Rate Limiting by Auth Status (Priority: P5)

**Goal**: Authenticated users get higher chatbot query limits (50/hr vs 10/hr)

**Independent Test**: Make queries as anonymous until limit, login, verify higher limit applies

**Endpoints**: (Modifies existing /api/query endpoint rate limiting)

### Implementation for User Story 5

- [X] T066 [US5] Update backend/app/middleware/rate_limit.py to extract user_id from JWT cookie
- [X] T067 [US5] Modify backend/app/routers/query.py to apply auth-aware rate limits (FR-020, FR-021)
- [X] T068 [US5] Add rate limit headers to responses (X-RateLimit-Limit, X-RateLimit-Remaining, X-RateLimit-Reset) per FR-022
- [X] T069 [US5] Update src/components/ChatWidget (existing) to show rate limit info and signup prompt per FR-023
- [X] T070 [US5] Link backend/app/services/chat_service.py sessions to authenticated user_id

**Checkpoint**: User Story 5 (Rate Limiting) fully functional - auth-based rate limits working

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Security hardening, logging, and final validations

- [ ] T071 [P] Implement CSRF protection in backend/app/middleware/csrf.py per FR-027
- [ ] T072 [P] Add structured logging for auth events in backend/app/services/auth_service.py per FR-028
- [ ] T073 [P] Add input sanitization to all auth endpoints per FR-026
- [ ] T074 Verify all auth endpoints respond <500ms p95 per FR-029
- [ ] T075 Run OWASP Top 10 security checklist per constitution quality gates
- [ ] T076 [P] Update docusaurus.config.ts with API URL configuration
- [ ] T077 [P] Create .env.example with all required auth environment variables
- [ ] T078 Run quickstart.md validation steps to verify setup
- [ ] T079 [P] Add sessions management endpoints GET /users/sessions and DELETE /users/sessions/{id} to backend/app/routers/users.py per FR-014

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phases 3-7)**: All depend on Foundational phase completion
  - User stories can proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 -> P2 -> P3 -> P4 -> P5)
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational - Adds to US1 login forms but independently testable
- **User Story 3 (P3)**: Can start after Foundational - Triggered after US1/US2 signup but independently testable
- **User Story 4 (P4)**: Can start after Foundational - Uses US1 /auth/refresh but independently testable
- **User Story 5 (P5)**: Can start after Foundational - Requires US1 auth middleware integration

### Within Each User Story

- Backend models before services
- Backend services before routers
- Routers registered before frontend integration
- Frontend API client before context/hooks
- Context/hooks before components
- Components before pages

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T002-T010)
- All Foundational model tasks marked [P] can run in parallel (T015-T017)
- Once Foundational phase completes, user stories can start in parallel (if team capacity allows)
- Frontend and backend tasks marked [P] within same story can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all backend model tasks together (after Foundational):
Task: "Create backend/app/services/user_service.py"
Task: "Create backend/app/services/auth_service.py"

# Launch all frontend tasks together (after backend routers):
Task: "Create src/services/authApi.ts"
Task: "Create src/context/AuthContext.tsx"
Task: "Create src/hooks/useAuth.ts"

# Launch all UI component tasks together:
Task: "Create src/components/Auth/LoginForm.tsx"
Task: "Create src/components/Auth/SignupForm.tsx"
Task: "Create src/components/Auth/AuthModal.tsx"
Task: "Create src/components/Auth/LogoutButton.tsx"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Email Signup & Login)
4. **STOP and VALIDATE**: Test email signup/login flow independently
5. Deploy/demo if ready - users can create accounts and login

### Incremental Delivery

1. Complete Setup + Foundational -> Foundation ready
2. Add User Story 1 -> Test independently -> Deploy/Demo (MVP!)
3. Add User Story 2 -> Test independently -> Deploy/Demo (adds Google OAuth)
4. Add User Story 3 -> Test independently -> Deploy/Demo (adds profile wizard)
5. Add User Story 4 -> Test independently -> Deploy/Demo (adds session persistence)
6. Add User Story 5 -> Test independently -> Deploy/Demo (adds auth-based rate limits)
7. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Email Auth) - HIGHEST PRIORITY
   - Developer B: User Story 2 (Google OAuth) - can start after US1 forms exist
   - Developer C: User Story 3 (Profile Wizard) - can start independently
3. Stories complete and integrate independently

---

## Summary

| Metric | Value |
|--------|-------|
| **Total Tasks** | 79 |
| **Phase 1 (Setup)** | 10 tasks |
| **Phase 2 (Foundational)** | 12 tasks |
| **Phase 3 (US1 - Email Auth)** | 19 tasks |
| **Phase 4 (US2 - Google OAuth)** | 9 tasks |
| **Phase 5 (US3 - Profile)** | 11 tasks |
| **Phase 6 (US4 - Sessions)** | 4 tasks |
| **Phase 7 (US5 - Rate Limits)** | 5 tasks |
| **Phase 8 (Polish)** | 9 tasks |
| **Parallel Opportunities** | 35 tasks marked [P] |
| **Suggested MVP Scope** | Phases 1-3 (41 tasks) |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
