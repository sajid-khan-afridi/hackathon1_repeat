# Tasks: RAG Chatbot Core

**Input**: Design documents from `/specs/003-rag-chatbot-core/`
**Prerequisites**: plan.md (tech stack), spec.md (user stories), research.md (decisions), data-model.md (entities), contracts/ (API specs)

**Tests**: Tests are NOT included in this task list unless explicitly requested. The spec does not require TDD approach.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app structure**: `backend/` (FastAPI Python), `src/components/` (React/Docusaurus frontend)
- Backend paths: `backend/app/`, `backend/tests/`
- Frontend paths: `src/components/ChatbotWidget/`, `tests/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create backend directory structure: backend/app/{models,routers,services,middleware,migrations}
- [ ] T002 Initialize Python project in backend/ with requirements.txt from quickstart.md
- [ ] T003 [P] Create .env.example file in repository root with all required environment variables from quickstart.md
- [ ] T004 [P] Configure pytest in backend/ with conftest.py and test directory structure
- [ ] T005 [P] Setup black, flake8, mypy configuration in backend/pyproject.toml

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Create backend/app/config.py with pydantic-settings for environment configuration
- [ ] T007 Implement backend/app/models/query.py with QueryRequest and QueryResponse Pydantic models
- [ ] T008 [P] Implement backend/app/models/chat.py with ChatSession and ChatMessage models
- [ ] T009 [P] Implement backend/app/models/citations.py with SourceCitation model
- [ ] T010 Create database migration backend/app/migrations/001_create_chat_tables.sql from data-model.md
- [ ] T011 Create database migration backend/app/migrations/002_add_auto_purge.sql from data-model.md
- [ ] T012 [P] Implement backend/app/main.py with FastAPI app initialization, CORS, and middleware
- [ ] T013 [P] Implement backend/app/middleware/logging.py with correlation ID tracking
- [ ] T014 [P] Implement backend/app/middleware/cors.py with CORS configuration
- [ ] T015 Implement backend/app/routers/health.py with /health endpoint checking database, Qdrant, OpenAI connectivity
- [ ] T016 Create static/data/faq-fallback.json with curated FAQ content per research.md
- [ ] T017 Implement backend/app/services/vector_service.py with Qdrant client initialization and search_context method
- [ ] T018 Implement backend/app/services/llm_service.py with OpenAI client initialization and GPT-4o-mini integration
- [ ] T019 Implement backend/app/services/chat_service.py with asyncpg connection pool and session/message CRUD

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Student Asks Textbook Question (Priority: P1) 🎯 MVP

**Goal**: Enable students to ask questions about textbook content and receive accurate answers with citations and confidence scores

**Independent Test**: Ask a question about indexed content (e.g., "What is inverse kinematics?") and verify response includes answer, sources with chapter citations, and confidence score

### Implementation for User Story 1

- [ ] T020 [US1] Implement confidence score calculation in backend/app/services/rag_service.py using weighted average from research.md
- [ ] T021 [US1] Implement query embedding in backend/app/services/rag_service.py using OpenAI text-embedding-3-small
- [ ] T022 [US1] Implement vector search with score thresholding (0.2 minimum) in backend/app/services/rag_service.py
- [ ] T023 [US1] Implement context ranking and top-k selection in backend/app/services/rag_service.py
- [ ] T024 [US1] Implement LLM response generation with strict grounding prompt in backend/app/services/rag_service.py
- [ ] T025 [US1] Implement source citation formatting in backend/app/services/rag_service.py
- [ ] T026 [US1] Implement off-topic detection (confidence < 0.2) with decline response in backend/app/services/rag_service.py
- [ ] T027 [US1] Implement low confidence warning (0.2-0.3) with rephrase suggestion in backend/app/services/rag_service.py
- [ ] T028 [US1] Create /api/v1/query POST endpoint in backend/app/routers/query.py calling rag_service
- [ ] T029 [US1] Add input sanitization for XSS/injection prevention in backend/app/routers/query.py
- [ ] T030 [US1] Add request validation using QueryRequest model in backend/app/routers/query.py
- [ ] T031 [US1] Implement error handling for LLM timeout (503 response) in backend/app/routers/query.py
- [ ] T032 [US1] Implement error handling for vector store unavailable (503 with FAQ fallback) in backend/app/routers/query.py
- [ ] T033 [P] [US1] Create src/components/ChatbotWidget/types.ts with TypeScript interfaces from contracts/schemas.ts
- [ ] T034 [P] [US1] Create src/components/ChatbotWidget/ChatInput.tsx with input field and submit button
- [ ] T035 [P] [US1] Create src/components/ChatbotWidget/MessageList.tsx with message display and scrolling
- [ ] T036 [P] [US1] Create src/components/ChatbotWidget/SourceCitations.tsx with clickable chapter links
- [ ] T037 [P] [US1] Create src/components/ChatbotWidget/ConfidenceIndicator.tsx with color-coded confidence display
- [ ] T038 [US1] Create src/components/ChatbotWidget/index.tsx integrating all sub-components with state management
- [ ] T039 [US1] Implement query submission with fetch POST to /api/v1/query in src/components/ChatbotWidget/index.tsx
- [ ] T040 [US1] Implement response rendering with answer, sources, and confidence in src/components/ChatbotWidget/index.tsx
- [ ] T041 [US1] Implement low confidence warning banner (0.2-0.3) in src/components/ChatbotWidget/index.tsx
- [ ] T042 [US1] Add error handling for network failures with retry suggestion in src/components/ChatbotWidget/index.tsx
- [ ] T043 [US1] Add WCAG 2.1 AA accessibility: keyboard navigation, ARIA labels, focus management in src/components/ChatbotWidget/index.tsx
- [ ] T044 [US1] Create src/components/ChatbotWidget/ChatbotWidget.module.css with responsive styles and 44x44px touch targets
- [ ] T045 [US1] Add token usage display (input/output/total) in src/components/ChatbotWidget/index.tsx for educational transparency

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently - students can ask questions and receive answers with citations

---

## Phase 4: User Story 2 - Student Continues Conversation (Priority: P1)

**Goal**: Enable students to maintain conversation context for follow-up questions within a session

**Independent Test**: Ask an initial question, then a follow-up question referencing the previous answer (e.g., "Can you explain that in more detail?")

### Implementation for User Story 2

- [ ] T046 [US2] Implement session creation with session_token generation in backend/app/services/chat_service.py
- [ ] T047 [US2] Implement session retrieval by session_id or session_token in backend/app/services/chat_service.py
- [ ] T048 [US2] Implement message persistence (user and assistant messages) in backend/app/services/chat_service.py
- [ ] T049 [US2] Implement source citation persistence in separate table in backend/app/services/chat_service.py
- [ ] T050 [US2] Implement last_activity_at update on each message in backend/app/services/chat_service.py
- [ ] T051 [US2] Implement conversation context retrieval (last 5 exchanges) in backend/app/services/rag_service.py
- [ ] T052 [US2] Integrate conversation history into LLM prompt for follow-up questions in backend/app/services/rag_service.py
- [ ] T053 [US2] Modify /api/v1/query to accept optional sessionId and return/create session in backend/app/routers/query.py
- [ ] T054 [US2] Create GET /api/v1/chat/sessions/{sessionId} endpoint in backend/app/routers/query.py for history retrieval
- [ ] T055 [US2] Create DELETE /api/v1/chat/sessions/{sessionId} endpoint in backend/app/routers/query.py
- [ ] T056 [US2] Implement session state management in src/components/ChatbotWidget/index.tsx using React useReducer
- [ ] T057 [US2] Persist sessionId in browser localStorage for cross-tab synchronization in src/components/ChatbotWidget/index.tsx
- [ ] T058 [US2] Load previous conversation history on widget mount in src/components/ChatbotWidget/index.tsx
- [ ] T059 [US2] Display full conversation history in MessageList component in src/components/ChatbotWidget/MessageList.tsx
- [ ] T060 [US2] Add "Clear History" button with DELETE /chat/sessions/{sessionId} call in src/components/ChatbotWidget/index.tsx

**Checkpoint**: At this point, User Stories 1 AND 2 should both work - students can ask questions and continue conversations with context

---

## Phase 5: User Story 3 - Student Filters by Module (Priority: P2)

**Goal**: Enable students to narrow search scope to specific textbook modules

**Independent Test**: Set module filter to "Module 2" and verify all returned sources are from Module 2 only

### Implementation for User Story 3

- [ ] T061 [P] [US3] Implement metadata filtering in Qdrant search in backend/app/services/vector_service.py
- [ ] T062 [US3] Implement confidence-based adaptive filtering logic from research.md in backend/app/services/rag_service.py
- [ ] T063 [US3] Add filter_message generation for adaptive filtering behavior in backend/app/services/rag_service.py
- [ ] T064 [US3] Extend /api/v1/query to accept optional filters parameter in backend/app/routers/query.py
- [ ] T065 [US3] Validate module filter (1-10), difficulty filter, tags filter in backend/app/routers/query.py
- [ ] T066 [P] [US3] Create src/components/ChatbotWidget/ModuleFilter.tsx with dropdown for module selection (1-10)
- [ ] T067 [US3] Integrate ModuleFilter component into ChatbotWidget index in src/components/ChatbotWidget/index.tsx
- [ ] T068 [US3] Pass filters object in query request when module is selected in src/components/ChatbotWidget/index.tsx
- [ ] T069 [US3] Display filter_message when adaptive filtering occurs in src/components/ChatbotWidget/index.tsx
- [ ] T070 [US3] Add "Clear Filter" button to reset module filter in src/components/ChatbotWidget/index.tsx

**Checkpoint**: All user stories 1, 2, and 3 should now be independently functional - students can filter searches by module

---

## Phase 6: User Story 4 - System Handles Off-Topic Questions (Priority: P2)

**Goal**: Gracefully decline off-topic questions and guide users to relevant topics

**Independent Test**: Ask unrelated question (e.g., "What's the weather?") and verify appropriate decline response

### Implementation for User Story 4

- [ ] T071 [US4] Implement suggested search terms generation from indexed content in backend/app/services/rag_service.py
- [ ] T072 [US4] Enhance off-topic response (confidence < 0.2) to include 3-5 suggested terms in backend/app/services/rag_service.py
- [ ] T073 [US4] Add topic guidance message ("I can only answer questions about...") in backend/app/services/rag_service.py
- [ ] T074 [US4] Display suggested search terms as clickable chips in src/components/ChatbotWidget/index.tsx
- [ ] T075 [US4] Implement click handler for suggested terms to auto-populate query input in src/components/ChatbotWidget/index.tsx

**Checkpoint**: Off-topic handling is functional - system clearly communicates boundaries and guides users

---

## Phase 7: User Story 5 - System Handles High Load (Priority: P3)

**Goal**: Implement rate limiting to ensure fair access and prevent abuse during peak usage

**Independent Test**: Simulate 11 requests from same user within an hour and verify rate limit response

### Implementation for User Story 5

- [ ] T076 [US5] Implement rate limiter using SlowAPI with token bucket algorithm in backend/app/services/rate_limiter.py
- [ ] T077 [US5] Configure anonymous user rate limit (10 queries/hour) in backend/app/services/rate_limiter.py
- [ ] T078 [US5] Configure authenticated user rate limit (50 queries/hour) in backend/app/services/rate_limiter.py
- [ ] T079 [US5] Implement user identifier extraction (user_id or IP) in backend/app/middleware/rate_limit.py
- [ ] T080 [US5] Apply rate limiter to /api/v1/query endpoint in backend/app/routers/query.py
- [ ] T081 [US5] Implement 429 error response with Retry-After header in backend/app/routers/query.py
- [ ] T082 [US5] Add rate limit headers (X-RateLimit-Limit, X-RateLimit-Remaining, X-RateLimit-Reset) to all responses
- [ ] T083 [US5] Display rate limit error message with time until reset in src/components/ChatbotWidget/index.tsx
- [ ] T084 [US5] Add countdown timer for rate limit reset in src/components/ChatbotWidget/index.tsx

**Checkpoint**: All user stories are now complete - rate limiting ensures fair access during high load

---

## Phase 8: Streaming Responses (Performance Enhancement)

**Purpose**: Implement response streaming for improved perceived performance (p95 < 3s, first token < 1s)

- [ ] T085 Implement Server-Sent Events (SSE) response streaming in backend/app/services/llm_service.py
- [ ] T086 Modify /api/v1/query to return StreamingResponse when Accept: text/event-stream in backend/app/routers/query.py
- [ ] T087 Implement StreamChunk model with chunk, done, sources, confidence fields in backend/app/models/query.py
- [ ] T088 [P] Create src/components/ChatbotWidget/LoadingState.tsx with streaming/loading indicators
- [ ] T089 Implement EventSource client for SSE in src/components/ChatbotWidget/index.tsx
- [ ] T090 Implement progressive answer rendering as chunks arrive in src/components/ChatbotWidget/index.tsx
- [ ] T091 Implement stream interruption handling with partial response preservation in src/components/ChatbotWidget/index.tsx
- [ ] T092 Add retry button for interrupted streams in src/components/ChatbotWidget/index.tsx

**Checkpoint**: Streaming responses improve perceived performance with first token appearing in < 1s

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T093 [P] Create benchmark test set of 50 questions in backend/tests/benchmark/test_questions.json covering all 10 chapters
- [ ] T094 [P] Implement NDCG@10 calculation in backend/tests/benchmark/test_relevance.py
- [ ] T095 [P] Implement faithfulness testing (zero hallucinations) in backend/tests/benchmark/test_faithfulness.py
- [ ] T096 Run benchmark tests and verify NDCG@10 > 0.8 and zero hallucinations
- [ ] T097 [P] Add structured JSON logging with correlation IDs in backend/app/middleware/logging.py
- [ ] T098 [P] Implement metrics tracking (p50/p95/p99 response times) in backend/app/middleware/logging.py
- [ ] T099 [P] Create backend/Dockerfile for containerized deployment
- [ ] T100 [P] Create backend/docker-compose.yml for local development per quickstart.md
- [ ] T101 Validate all environment variables are documented in .env.example
- [ ] T102 Run linters: black, flake8, mypy on backend code
- [ ] T103 Run frontend linters: npm run lint, npm run typecheck
- [ ] T104 Validate quickstart.md instructions work end-to-end
- [ ] T105 Update OpenAPI spec contracts/openapi.yaml if any endpoints changed
- [ ] T106 Create PHR (Prompt History Record) documenting RAG implementation per CLAUDE.md
- [ ] T107 Create ADR for RAG architecture decisions per plan.md recommendations

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational - No dependencies on other stories
  - User Story 2 (P1): Can start after Foundational - Builds on US1 for session management
  - User Story 3 (P2): Can start after Foundational - Independent of US1/US2
  - User Story 4 (P2): Can start after Foundational - Enhances US1 off-topic handling
  - User Story 5 (P3): Can start after Foundational - Independent rate limiting
- **Streaming (Phase 8)**: Can start after US1 is functional
- **Polish (Phase 9)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Foundation only - core RAG pipeline
- **User Story 2 (P1)**: Foundation + creates session management (US1 uses it but doesn't require it)
- **User Story 3 (P2)**: Foundation only - filtering is independent
- **User Story 4 (P2)**: Foundation only - enhances US1 but doesn't depend on it
- **User Story 5 (P3)**: Foundation only - rate limiting is independent

### Within Each User Story

- Backend models before services
- Services before routers/endpoints
- Frontend types before components
- Sub-components before main ChatbotWidget integration
- Core implementation before error handling
- Story complete before moving to next priority

### Parallel Opportunities

- **Setup (Phase 1)**: T003, T004, T005 can run in parallel
- **Foundational (Phase 2)**: T008-T009, T012-T014, T017-T019 can run in parallel
- **User Story 1**: T033-T037 (frontend components), T061 (if starting US3 early)
- **User Story 2**: T066 (if starting US3 early)
- **User Story 3**: T061, T066 can run in parallel
- **Polish**: T093-T095, T097-T100 can run in parallel

---

## Parallel Example: User Story 1 Frontend Components

```bash
# Launch all frontend components for User Story 1 together:
Task: "Create src/components/ChatbotWidget/types.ts with TypeScript interfaces"
Task: "Create src/components/ChatbotWidget/ChatInput.tsx with input field and submit button"
Task: "Create src/components/ChatbotWidget/MessageList.tsx with message display"
Task: "Create src/components/ChatbotWidget/SourceCitations.tsx with clickable chapter links"
Task: "Create src/components/ChatbotWidget/ConfidenceIndicator.tsx with color-coded display"
```

---

## Implementation Strategy

### MVP First (User Stories 1 + 2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Core RAG Q&A)
4. Complete Phase 4: User Story 2 (Conversation continuity)
5. **STOP and VALIDATE**: Test both stories independently
6. Run quickstart.md end-to-end validation
7. Deploy/demo if ready

**Rationale**: User Stories 1 and 2 are both P1 priority and together provide complete conversational Q&A experience. This is the minimum viable product.

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Basic Q&A working
3. Add User Story 2 → Test independently → Conversation continuity working (MVP!)
4. Add User Story 3 → Test independently → Module filtering working
5. Add User Story 4 → Test independently → Off-topic handling working
6. Add User Story 5 → Test independently → Rate limiting working
7. Add Streaming → Test independently → Performance optimized
8. Polish phase → Production ready

Each increment adds value without breaking previous stories.

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Backend RAG pipeline)
   - Developer B: User Story 1 (Frontend components)
   - Developer C: User Story 2 (Session management)
3. Continue with remaining stories in priority order or in parallel

---

## Task Summary

- **Total Tasks**: 107
- **Setup (Phase 1)**: 5 tasks
- **Foundational (Phase 2)**: 14 tasks
- **User Story 1 (P1)**: 26 tasks (T020-T045)
- **User Story 2 (P1)**: 15 tasks (T046-T060)
- **User Story 3 (P2)**: 10 tasks (T061-T070)
- **User Story 4 (P2)**: 5 tasks (T071-T075)
- **User Story 5 (P3)**: 9 tasks (T076-T084)
- **Streaming Enhancement**: 8 tasks (T085-T092)
- **Polish & Cross-Cutting**: 15 tasks (T093-T107)

### Parallel Opportunities Identified

- **23 tasks** marked [P] can run in parallel within their phase
- **5 user stories** can run in parallel after Foundational phase (if team capacity allows)
- **Multiple frontend components** can be developed in parallel

### Suggested MVP Scope

**Minimum**: User Stories 1 + 2 (41 tasks after Setup + Foundational)
- Provides complete conversational Q&A experience
- Students can ask questions and continue conversations
- All core functionality present

**Recommended First Release**: Add User Story 3 (10 more tasks)
- Adds module filtering for focused studying
- Still within reasonable sprint scope

---

## Notes

- [P] tasks = different files, no dependencies within phase
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Tests are NOT included unless explicitly requested (spec does not require TDD)
- All file paths are exact locations per plan.md structure
- Confidence thresholds: <0.2 decline, 0.2-0.3 warning, >0.7 high confidence
- Rate limits: 10/hour anonymous, 50/hour authenticated
- Performance targets: p95 < 3s, first token < 1s, vector search < 1s
