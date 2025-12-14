# Feature Specification: RAG Chatbot Core

**Feature Branch**: `003-rag-chatbot-core`
**Created**: 2025-12-14
**Status**: Draft
**Input**: User description: "Phase 3 RAG Chatbot Core - Build the AI-powered question answering system for the robotics textbook"

## Overview

Build an AI-powered question answering system that enables students to ask questions about robotics textbook content and receive accurate, sourced answers. The system retrieves relevant information from indexed textbook chapters and generates contextual responses with proper citations.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Asks Textbook Question (Priority: P1)

A student studying robotics wants to understand a concept from the textbook. They type their question into the chatbot widget and receive an accurate answer with citations to specific chapters, along with a confidence score indicating answer reliability.

**Why this priority**: This is the core value proposition - students need accurate, sourced answers to learn effectively. Without this, the chatbot has no purpose.

**Independent Test**: Can be fully tested by asking a question about content in the indexed chapters and verifying the response includes relevant information with proper chapter citations.

**Acceptance Scenarios**:

1. **Given** a student on the textbook documentation site, **When** they type "What is inverse kinematics?" into the chatbot, **Then** they receive an answer explaining inverse kinematics with citations to relevant chapter(s), an excerpt from the source, and a confidence score.

2. **Given** a student asks a question covered in multiple chapters, **When** the system processes the query, **Then** it returns the most relevant sources (up to 5) ranked by relevance score.

3. **Given** a student asks a question, **When** the answer is generated, **Then** it includes only information that can be traced to retrieved source chunks (no hallucination).

---

### User Story 2 - Student Continues Conversation (Priority: P1)

A student has asked an initial question and wants to follow up with a related question. The system maintains conversation context so follow-up questions are understood in relation to previous exchanges.

**Why this priority**: Maintaining conversation flow is essential for learning - students naturally ask follow-up questions to deepen understanding.

**Independent Test**: Can be tested by asking an initial question, then a follow-up that references the previous answer (e.g., "Can you explain that in more detail?").

**Acceptance Scenarios**:

1. **Given** a student has received an answer about ROS publishers, **When** they ask "Can you show me an example?", **Then** the system understands this refers to ROS publishers and provides a relevant code example from the textbook.

2. **Given** a student's chat session, **When** they return within the same browser session, **Then** their previous conversation history is displayed and accessible.

3. **Given** a conversation exceeds 10 exchanges, **When** the student asks a new question, **Then** the system uses recent context (last 5 exchanges) to inform retrieval without performance degradation.

---

### User Story 3 - Student Filters by Module (Priority: P2)

A student wants to focus their search on a specific module of the textbook (e.g., "Module 2: Motion Planning"). They can filter the chatbot's search scope to only retrieve answers from that module's chapters.

**Why this priority**: Focused searching helps students studying specific topics avoid information overload from unrelated chapters.

**Independent Test**: Can be tested by setting a module filter and verifying all returned sources are from that module only.

**Acceptance Scenarios**:

1. **Given** a student selects "Module 2" as a filter, **When** they ask a question, **Then** all retrieved sources and the generated answer come only from Module 2 chapters.

2. **Given** a module filter is active, **When** no relevant content exists in that module for the query, **Then** the system responds with "I couldn't find information about that topic in Module 2. Would you like me to search all modules?"

3. **Given** a student has a module filter active, **When** they clear the filter, **Then** subsequent queries search across all indexed chapters.

---

### User Story 4 - System Handles Off-Topic Questions (Priority: P2)

A student asks a question unrelated to the robotics textbook content. The system gracefully declines to answer and guides them back to relevant topics.

**Why this priority**: Prevents confusion and hallucination by clearly communicating system boundaries.

**Independent Test**: Can be tested by asking questions about unrelated topics (e.g., "What's the weather?") and verifying appropriate decline response.

**Acceptance Scenarios**:

1. **Given** a student asks "What is the capital of France?", **When** the system processes this query, **Then** it responds with "I can only answer questions about the robotics textbook. Try asking about topics like ROS, kinematics, or motion planning."

2. **Given** a borderline query that partially relates to textbook content, **When** confidence score is below threshold (0.3), **Then** the system indicates low confidence and suggests reformulating the question.

3. **Given** no relevant chunks are retrieved (all scores below 0.2), **When** generating a response, **Then** the system returns "I couldn't find information about that topic in the textbook" with 3-5 suggested search terms based on indexed content.

---

### User Story 5 - System Handles High Load (Priority: P3)

During peak usage (exam periods), many students use the chatbot simultaneously. The system implements rate limiting to ensure fair access and prevent abuse while maintaining service quality.

**Why this priority**: Ensures system availability and fair resource distribution, though less critical than core functionality.

**Independent Test**: Can be tested by simulating multiple requests from same user and verifying rate limit responses.

**Acceptance Scenarios**:

1. **Given** an unauthenticated user has made 10 queries in the past hour, **When** they submit another query, **Then** they receive a message: "Rate limit reached. Please wait or sign in for increased limits."

2. **Given** an authenticated user has made 50 queries in the past hour, **When** they submit another query, **Then** they receive a rate limit message with time until reset.

3. **Given** rate limiting is active, **When** the limit resets after an hour, **Then** the user can resume querying without issues.

---

### Edge Cases

- **Empty query**: User submits blank input - system prompts for a valid question
- **Very long query**: Query exceeds 500 characters - system truncates with notification or rejects with character limit message
- **Special characters/injection**: Query contains SQL/script injection attempts - system sanitizes input and logs security event
- **Network timeout**: External service (embedding API, LLM) times out - system returns friendly error with retry suggestion
- **Database unavailable**: Chat history database unreachable - system operates in stateless mode without history persistence
- **Vector store unavailable**: Qdrant is unreachable - system falls back to static FAQ responses
- **Concurrent session conflict**: User opens multiple tabs - tabs share synchronized session context using the same session ID
- **Expired session**: User returns after 30 days - old history is purged, new session starts fresh

## Requirements *(mandatory)*

### Functional Requirements

**Query Processing**
- **FR-001**: System MUST accept natural language queries up to 500 characters
- **FR-002**: System MUST embed queries using the same model as indexed content (sentence-transformers/all-MiniLM-L6-v2)
- **FR-003**: System MUST retrieve top-k relevant chunks (default k=5, configurable 1-10)
- **FR-004**: System MUST generate responses using retrieved context only (no external knowledge)
- **FR-005**: System MUST return source citations with chapter ID, title, relevance score, and text excerpt

**Response Generation**
- **FR-006**: System MUST include confidence score (0.0-1.0) with every response
- **FR-007**: System MUST attribute all claims to specific source chunks
- **FR-008**: System MUST decline to answer when no relevant content is found (all scores < 0.2)
- **FR-009**: System MUST decline off-topic questions with guidance on available topics
- **FR-010**: System MUST support streaming responses for perceived performance

**Chat History**
- **FR-011**: System MUST persist chat sessions with unique identifiers
- **FR-012**: System MUST store message role (user/assistant), content, and source references
- **FR-013**: System MUST maintain conversation context for follow-up questions
- **FR-014**: System MUST auto-purge chat history older than 30 days
- **FR-015**: System MUST support anonymous (session-based) and authenticated users

**Filtering & Search**
- **FR-016**: System MUST support filtering retrieval by module number
- **FR-017**: System MUST support filtering by difficulty level (beginner/intermediate/advanced)
- **FR-018**: System MUST support filtering by tags/topics

**Rate Limiting**
- **FR-019**: System MUST enforce 10 queries/hour for unauthenticated users
- **FR-020**: System MUST enforce 50 queries/hour for authenticated users
- **FR-021**: System MUST return informative rate limit messages with reset time

**Error Handling**
- **FR-022**: System MUST return 503 with retry message when LLM service times out
- **FR-023**: System MUST fall back to static FAQ (curated JSON file in codebase) when vector store is unavailable
- **FR-024**: System MUST log all errors with correlation IDs for debugging
- **FR-025**: System MUST sanitize all user inputs to prevent injection attacks

**User Interface**
- **FR-026**: System MUST provide a chat widget embeddable in documentation pages
- **FR-027**: System MUST display message history within the current session
- **FR-028**: System MUST show loading states during query processing
- **FR-029**: System MUST display source citations with links to original chapters
- **FR-030**: System MUST support keyboard navigation and screen readers (WCAG 2.1 AA)
- **FR-031**: System MUST display token usage (input/output tokens) for each response in the UI for educational transparency
- **FR-032**: System MUST display a prominent warning banner with rephrase suggestion when confidence score is between 0.2-0.3
- **FR-033**: System MUST preserve partial response on stream interruption, appending an error indicator with retry button

### Key Entities

- **ChatSession**: Represents a conversation thread. Contains session ID, optional user ID, creation timestamp, and last activity timestamp.

- **ChatMessage**: Individual message in a conversation. Contains message ID, session reference, role (user/assistant), content text, source citations (for assistant messages), token usage, and timestamp.

- **QueryRequest**: Incoming question from user. Contains query text, optional user ID, optional filters (module, tags, difficulty), and top-k parameter.

- **QueryResponse**: System's answer to a question. Contains generated answer text, list of source citations with scores and excerpts, confidence score, chat session ID, and token usage breakdown.

- **SourceCitation**: Reference to textbook content. Contains chapter ID, chapter title, relevance score, and text excerpt.

- **RateLimitState**: Tracks user's API usage. Contains user/session identifier, query count, window start time, and limit tier.

## Success Criteria *(mandatory)*

### Measurable Outcomes

**Answer Quality**
- **SC-001**: System achieves NDCG@10 > 0.8 on benchmark test set of 50 questions covering all 10 chapters
- **SC-002**: 100% of generated answers cite only retrieved source chunks (zero hallucinations verified by faithfulness testing)
- **SC-003**: Users find relevant answers on first query attempt 80% of the time (measured by no immediate follow-up rephrasing)

**Performance**
- **SC-004**: 95% of queries complete end-to-end in under 3 seconds (from submit to first response token)
- **SC-005**: System handles 50 concurrent users without degradation
- **SC-006**: Chat history retrieval completes in under 200ms for conversations up to 50 messages

**Reliability**
- **SC-007**: System maintains 99% uptime during operating hours (excluding planned maintenance)
- **SC-008**: All errors are logged with correlation IDs enabling issue resolution within 4 hours
- **SC-009**: Fallback mechanisms activate within 5 seconds of primary service failure

**User Experience**
- **SC-010**: Chat widget loads in under 2 seconds on standard connections
- **SC-011**: 90% of users can successfully submit their first query without assistance
- **SC-012**: Source citations are clickable and navigate to correct chapter content

**Resource Constraints**
- **SC-013**: Vector index size remains under 1GB (Qdrant Cloud free tier limit)
- **SC-014**: Chat history storage remains under 0.5GB with 30-day auto-purge (Neon free tier limit)
- **SC-015**: Token usage per query averages under 2000 tokens total (retrieval + generation)

## Clarifications

### Session 2025-12-14

- Q: LLM Provider Architecture - How should we integrate with OpenAI for response generation? → A: OpenAI Agents SDK with structured tools (provides built-in tool calling, error handling, and extensibility)
- Q: Chat Session Isolation Strategy - How should we handle multiple browser tabs? → A: Single shared session across all tabs (immediate sync)
- Q: Token Usage Visibility - Should token usage be shown to users? → A: Show in UI for all users (educational transparency)
- Q: Static FAQ Fallback Source - Where does fallback FAQ content come from? → A: Curated JSON file shipped with the codebase (version-controlled)
- Q: Low Confidence Response Behavior - What to show when confidence is 0.2-0.3? → A: Show answer with prominent warning banner + suggestion to rephrase
- Q: Streaming Response Interruption - What to show if stream is interrupted? → A: Keep partial response visible, append error indicator + retry button
- Q: LLM Model Selection - Which OpenAI model should we use for response generation? → A: GPT-4o-mini for cost efficiency with acceptable quality for educational content
- Q: Module Filtering Behavior - How should the system handle queries when a module filter is active but the answer requires information from other modules? → A: Confidence-based adaptive filtering (0.7+ = filter strict, 0.4-0.7 = suggest, <0.4 = no filter)
- Q: Source Citation Storage - How should we store source citations in chat messages to optimize storage and enable click-to-navigate functionality? → A: Store references in separate table with IDs (normalized, recommended)

## Assumptions

1. **Phase 2 Complete**: 10 MDX chapters are already indexed and available in Qdrant with proper metadata (chapter_id, tags, difficulty_level)
2. **Environment Configuration**: Required environment variables (QDRANT_URL, OPENAI_API_KEY, DATABASE_URL) are configured via env-setup skill
3. **Embedding Consistency**: Content was indexed using sentence-transformers/all-MiniLM-L6-v2 (384 dimensions)
4. **Network Access**: System has reliable network access to Qdrant Cloud, OpenAI API, and Neon PostgreSQL
5. **User Identification**: Unauthenticated users are identified by browser session; authenticated users by user ID from auth system
6. **LLM Model**: GPT-4o-mini is used for response generation (cost-effective for educational use case)
7. **Single Language**: System operates in English only (Urdu translation is separate feature)
8. **Browser Support**: Modern browsers (Chrome, Firefox, Safari, Edge - last 2 versions)

## Dependencies

- **Phase 2 Output**: Indexed textbook content in Qdrant vector store
- **External Services**:
  - Qdrant Cloud (vector storage and search)
  - OpenAI API (embedding and generation)
  - Neon PostgreSQL (chat history persistence)
- **Internal Skills**:
  - qdrant-vectorstore: Vector operations
  - openai-agents-sdk: LLM integration
  - neon-postgres: Database operations
  - rag-pipeline: Orchestration
  - fastapi-backend: API endpoints
  - react-components: UI widget

## Out of Scope / Non-Goals

- Multi-turn complex reasoning chains (single Q&A per turn only)
- Voice input/output
- Image understanding or generation
- Real-time content updates (content is indexed in batches)
- Advanced analytics dashboard
- Multi-language support (English only for Phase 3)
- Custom fine-tuning of embedding or generation models
- Offline/PWA functionality
- Mobile-native applications (web widget only)

## Risks and Mitigations

| Risk                        | Impact | Likelihood | Mitigation                                                     |
| --------------------------- | ------ | ---------- | -------------------------------------------------------------- |
| LLM hallucination           | High   | Medium     | Strict system prompt, faithfulness testing, confidence thresholds |
| API rate limits exceeded    | Medium | Medium     | Caching frequent queries, batch embedding requests             |
| Free tier limits exceeded   | High   | Low        | Monitor usage, auto-purge old data, queue non-urgent requests  |
| Poor search relevance       | High   | Medium     | Benchmark testing, iterative prompt tuning, metadata filtering |
| Latency exceeds budget      | Medium | Medium     | Streaming responses, connection pooling, CDN for static assets |
