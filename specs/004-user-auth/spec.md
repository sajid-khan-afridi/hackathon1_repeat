# Feature Specification: User Authentication System

**Feature Branch**: `004-user-auth`
**Created**: 2025-12-17
**Status**: Draft
**Input**: Phase 4A: User Authentication System - Implement email/password and Google OAuth authentication for the Physical AI & Humanoid Robotics Textbook

## Clarifications

### Session 2025-12-17

- Q: What is the acceptable response time target for authentication API endpoints? → A: 500ms p95 latency (standard auth SLA)
- Q: Should authentication be a separate microservice or integrated into the existing backend? → A: Integrated into existing Railway backend
- Q: What loading/error UX pattern should authentication forms use? → A: Inline feedback (disable submit, spinner in button, errors below fields)

## Overview

Enable user identity, session management, and profile collection for the Physical AI & Humanoid Robotics Textbook. This unlocks personalization features in Phase 4B by providing authenticated user context.

### Dependencies

- Phase 1 (Docusaurus infrastructure) - Complete
- Phase 2 (MDX textbook chapters) - Complete
- Phase 3 (RAG chatbot backend) - Complete

### Out of Scope

- Email verification workflow
- Password reset/recovery flow
- Account deletion/GDPR data export
- Multi-factor authentication (MFA)
- Social login providers beyond Google (GitHub, Facebook, etc.)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Email Signup & Login (Priority: P1)

As a new visitor to the textbook, I want to create an account with my email and password so I can access personalized content and higher chatbot query limits.

**Why this priority**: Account creation is the foundation for all other authentication features. Without email signup, users cannot establish an identity in the system. This is the minimum viable authentication feature.

**Independent Test**: Can be fully tested by creating an account, logging out, and logging back in. Delivers immediate value by establishing user identity.

**Acceptance Scenarios**:

1. **Given** I am an anonymous visitor, **When** I navigate to signup and enter a valid email and password meeting requirements, **Then** my account is created and I am logged in automatically
2. **Given** I have an existing account, **When** I enter my email and correct password on the login form, **Then** I am authenticated and redirected to my previous location
3. **Given** I am on the signup form, **When** I enter a password shorter than 8 characters, **Then** I see a validation error before submission
4. **Given** I am on the signup form, **When** I enter a password without an uppercase letter, **Then** I see a validation error indicating password requirements
5. **Given** I am on the signup form, **When** I enter a password without a number, **Then** I see a validation error indicating password requirements
6. **Given** an account already exists with my email, **When** I try to sign up with the same email, **Then** I see an error that the email is already registered

---

### User Story 2 - Google OAuth Login (Priority: P2)

As a returning user, I want to sign in with my Google account for faster, passwordless access to the textbook.

**Why this priority**: Google OAuth provides convenience for users who prefer not to manage another password. It builds on P1's foundation but is not required for MVP authentication.

**Independent Test**: Can be tested by clicking Google login button, completing OAuth flow, and verifying session is established.

**Acceptance Scenarios**:

1. **Given** I am an anonymous visitor, **When** I click "Sign in with Google" and authorize the application, **Then** my account is created (if new) and I am logged in
2. **Given** I have an existing email account with the same email as my Google account, **When** I sign in with Google, **Then** my accounts are linked and I can use either method to log in
3. **Given** I click "Sign in with Google", **When** I cancel or deny authorization, **Then** I am returned to the login page with an appropriate message
4. **Given** the Google OAuth service is unavailable, **When** I try to sign in with Google, **Then** I see a helpful error message suggesting email login as alternative

---

### User Story 3 - Profile Collection Wizard (Priority: P3)

As a newly registered user, I want to answer 5 questions about my background so the textbook content can adapt to my experience level and learning goals.

**Why this priority**: Profile collection enables personalization (Phase 4B) but is not required for basic authentication. Users can skip and complete later.

**Independent Test**: Can be tested by completing signup, being presented with profile wizard, answering questions, and verifying profile is saved.

**Acceptance Scenarios**:

1. **Given** I just completed signup (email or Google), **When** the signup succeeds, **Then** I am presented with the profile wizard
2. **Given** I am in the profile wizard, **When** I answer all 5 questions and submit, **Then** my profile is saved and I proceed to the textbook
3. **Given** I am in the profile wizard, **When** I choose to skip, **Then** I proceed to the textbook with a reminder banner to complete my profile
4. **Given** I have an incomplete profile, **When** I navigate to any page, **Then** I see a non-intrusive banner prompting me to complete my profile
5. **Given** I have a complete profile, **When** I want to update my answers, **Then** I can access profile settings and modify my responses

**Profile Questions**:
1. Programming experience level (Beginner / Intermediate / Advanced)
2. ROS familiarity (None / Basic / Proficient)
3. Hardware access (Simulation Only / Jetson Kit / Full Robot Lab)
4. Learning goal (Career Transition / Academic Research / Hobby/Personal)
5. Preferred code examples (Python / C++ / Both)

---

### User Story 4 - Session Persistence (Priority: P4)

As a logged-in user, I want my session to persist across browser sessions and automatically refresh so I don't have to log in repeatedly.

**Why this priority**: Session persistence improves user experience but requires working authentication first. Can be added after P1-P3 are functional.

**Independent Test**: Can be tested by logging in, closing browser, reopening, and verifying still logged in.

**Acceptance Scenarios**:

1. **Given** I am logged in, **When** I close and reopen my browser within 30 days, **Then** I remain logged in
2. **Given** I am logged in, **When** my access token expires during a session, **Then** it is silently refreshed without interrupting my activity
3. **Given** I am logged in, **When** I click logout, **Then** my session is terminated and I cannot access protected features
4. **Given** my refresh token has expired (after 30 days), **When** I return to the site, **Then** I am prompted to log in again

---

### User Story 5 - Rate Limiting by Authentication Status (Priority: P5)

As an authenticated user, I want higher chatbot query limits than anonymous users so I can learn more effectively.

**Why this priority**: Rate limiting differentiation incentivizes signup but requires working authentication and chatbot integration first.

**Independent Test**: Can be tested by making queries as anonymous user until limit, then logging in and verifying higher limit.

**Acceptance Scenarios**:

1. **Given** I am an anonymous user, **When** I make chatbot queries, **Then** I am limited to 10 queries per hour
2. **Given** I am an authenticated user, **When** I make chatbot queries, **Then** I am limited to 50 queries per hour
3. **Given** I am anonymous and approaching my rate limit, **When** I have 2 queries remaining, **Then** I see a prompt suggesting I sign in for more queries
4. **Given** I have exceeded my rate limit, **When** I try to make another query, **Then** I see a clear message about when I can query again

---

### Edge Cases

- What happens when a user tries to sign up with an email that exists via Google OAuth?
  - System should offer to link accounts or notify user to sign in via Google
- How does the system handle concurrent sessions on multiple devices?
  - Sessions are independent; logout on one device does not affect others
- What happens if a user's email changes on their Google account?
  - Email is captured at first OAuth; subsequent logins match by Google ID
- What happens during authentication if the database is temporarily unavailable?
  - User sees a friendly error message asking them to try again shortly
- What happens if someone attempts brute-force password guessing?
  - After 10 failed attempts, the IP is locked out for 15 minutes

## Requirements *(mandatory)*

### Functional Requirements

**Account Management**
- **FR-001**: System MUST allow users to create accounts with email and password
- **FR-002**: System MUST validate email format before account creation
- **FR-003**: System MUST enforce password requirements: minimum 8 characters, at least 1 uppercase letter, at least 1 number
- **FR-004**: System MUST prevent duplicate accounts with the same email address
- **FR-005**: System MUST store passwords securely using industry-standard hashing

**Authentication**
- **FR-006**: System MUST authenticate users via email/password combination
- **FR-007**: System MUST authenticate users via Google OAuth 2.0
- **FR-008**: System MUST auto-link accounts when Google OAuth email matches existing email account
- **FR-009**: System MUST issue secure session tokens upon successful authentication
- **FR-010**: System MUST support silent token refresh without user interaction

**Session Management**
- **FR-011**: Access tokens MUST expire after 24 hours
- **FR-012**: Refresh tokens MUST expire after 30 days
- **FR-013**: System MUST invalidate all tokens when user logs out
- **FR-014**: System MUST support concurrent sessions on multiple devices

**Profile Collection**
- **FR-015**: System MUST present profile wizard to new users after signup
- **FR-016**: System MUST allow users to skip profile wizard
- **FR-017**: System MUST persist profile answers for authenticated users
- **FR-018**: System MUST allow users to update their profile at any time
- **FR-019**: System MUST show profile completion banner for incomplete profiles

**Rate Limiting**
- **FR-020**: System MUST apply 10 queries/hour limit to anonymous users
- **FR-021**: System MUST apply 50 queries/hour limit to authenticated users
- **FR-022**: System MUST include rate limit information in responses
- **FR-023**: System MUST prompt anonymous users to sign in when approaching limit

**Security**
- **FR-024**: System MUST lock out IP addresses after 10 failed login attempts for 15 minutes
- **FR-025**: System MUST use secure cookies (httpOnly, secure flags)
- **FR-026**: System MUST validate and sanitize all user inputs
- **FR-027**: System MUST protect against cross-site request forgery (CSRF)
- **FR-028**: System MUST log authentication events for security auditing

**Performance**
- **FR-029**: Authentication API endpoints (login, signup, token refresh) MUST respond within 500ms at p95

**User Experience**
- **FR-030**: Authentication forms MUST disable the submit button during submission and show a spinner inside the button
- **FR-031**: Validation errors MUST be displayed inline below the relevant form fields
- **FR-032**: Network/server errors MUST be displayed as a dismissible message above or below the form
- **FR-033**: Forms MUST prevent double-submission while a request is in progress

### Key Entities

- **User**: Represents an authenticated individual. Contains email, hashed password (optional for OAuth-only), Google ID (optional), creation timestamp, last login timestamp
- **Session**: Represents an active authentication session. Contains user reference, access token, refresh token, expiration timestamp, creation timestamp
- **UserProfile**: Represents learning preferences. Contains user reference, programming experience level, ROS familiarity, hardware access type, learning goal, preferred code language, completion status, profile hash for change detection

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete email signup in under 60 seconds
- **SC-002**: Users can complete Google OAuth login in under 30 seconds
- **SC-003**: 95% of login attempts succeed on first try (for valid credentials)
- **SC-004**: Session tokens refresh without user-visible interruption 100% of the time
- **SC-005**: Users can complete profile wizard in under 2 minutes
- **SC-006**: 80% of new users complete the profile wizard (not skipped)
- **SC-007**: Rate limiting correctly differentiates anonymous vs authenticated users 100% of the time
- **SC-008**: System handles 100 concurrent authentication requests without degradation
- **SC-009**: Zero security vulnerabilities in authentication flows (OWASP Top 10 audit passed)
- **SC-010**: 80% test coverage on all authentication flows

## Assumptions

- Users have modern browsers supporting secure cookies and localStorage
- Google OAuth credentials will be configured in environment variables
- The existing Neon PostgreSQL database can accommodate new authentication tables
- The existing Railway backend can be extended with authentication endpoints
- The existing Docusaurus frontend supports React context for auth state management
- HTTPS is enforced on all production endpoints

## Constraints

- **Architecture**: Authentication MUST be integrated into the existing Railway backend (not a separate microservice) to reduce operational complexity and avoid cross-service latency

## Integration Points

- **Floating Chat Widget**: Shows login prompt when anonymous users approach rate limit
- **RAG Chatbot Backend**: Query endpoint checks authentication status and applies appropriate rate limit
- **PersonalizedSection Component**: Reads user profile from auth context to adapt content display

## Skills to Use

- `better-auth-setup` - Authentication configuration and setup
- `user-profiling` - Post-signup profile collection wizard
- `neon-postgres` - Database schema creation and operations
