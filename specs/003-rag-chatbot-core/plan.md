# Implementation Plan: RAG Chatbot Core

**Branch**: `003-rag-chatbot-core` | **Date**: 2025-12-14 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-rag-chatbot-core/spec.md`

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

## Summary

Build an AI-powered question answering system that enables students to ask questions about robotics textbook content and receive accurate, sourced answers. The system implements a complete RAG pipeline using:
- **Qdrant Cloud** for vector storage and semantic search (indexed textbook chunks)
- **OpenAI GPT-4o-mini** via Chat Completions API for response generation
- **Neon PostgreSQL** for chat history persistence and session management
- **FastAPI** backend exposing `/api/v1/query` endpoint
- **React chatbot widget** embedded in Docusaurus documentation pages

Technical approach: Query embedding → Vector search → Context ranking → LLM generation with strict grounding → Citation formatting → Response streaming.

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript 5.6+ (frontend)
**Primary Dependencies**:
- Backend: FastAPI, uvicorn, pydantic, openai, qdrant-client, asyncpg, slowapi
- Frontend: React 19, Docusaurus 3.9.2, clsx
**Storage**:
- Vector: Qdrant Cloud (1GB free tier, 1536 dimensions for text-embedding-3-small)
- Relational: Neon PostgreSQL (0.5GB free tier) - chat_sessions, chat_messages, source_citations
**Testing**:
- Backend: pytest, pytest-asyncio, pytest-cov
- Frontend: Jest, React Testing Library, Playwright (E2E)
**Target Platform**: Web (Docusaurus static site + FastAPI backend on Railway/Render)
**Project Type**: Web application (frontend + backend)
**Performance Goals**:
- End-to-end response time: p95 < 3s
- First token latency: < 1s (streaming)
- Vector search: < 1s
- Chat history retrieval: < 200ms
**Constraints**:
- Qdrant: 1GB storage limit
- Neon: 0.5GB storage, 100 compute hours/month
- OpenAI: Rate limits per API key tier
- Rate limiting: 10 queries/hour (anonymous), 50 queries/hour (authenticated)
**Scale/Scope**:
- 1000+ indexed chapter chunks
- 50 concurrent users target
- 30-day chat history retention

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

<!--
  QUALITY GATES THREE-TIER HIERARCHY (per ADR-003):

  Tier 1: Constitution Detailed Exit Criteria (SOURCE OF TRUTH)
    Location: .specify/memory/constitution.md Section "QUALITY GATES"
    Reference: Lines 948-957 (Phase 3 Exit Criteria)

  Tier 2: Constitution Summary (INFORMATIONAL ONLY)
    Location: .specify/memory/constitution.md Section "Core Principles > Quality Over Speed"
    Reference: Lines 90-91
    Note: High-level summaries; NOT authoritative source

  Tier 3: Plan Phase Constitution Check (IMPLEMENTATION VIEW - THIS FILE)
    Rule: MUST exactly match Tier 1 (Constitution Detailed Exit Criteria)
    Any deviation is a conflict; Tier 1 always wins
-->

### Phase 3 Quality Gates (from Constitution)
<!-- Reference: .specify/memory/constitution.md lines 948-957 -->
- [ ] Vector database has 1000+ chunks
- [ ] `/query` API endpoint functional and documented
- [ ] NDCG@10 > 0.8 on 50-question test set
- [ ] Response time p95 < 3s (measured in CI)
- [ ] Zero hallucinations on test set
- [ ] ADR created for RAG architecture decisions
- [ ] PHR created documenting RAG implementation

### Core Principles Alignment

**Quality Over Speed**:
- Mandatory benchmark test set of 50 questions covering all 10 chapters
- Faithfulness testing to ensure 100% grounded responses (no hallucinations)
- 80% test coverage for all core RAG functionality
- Streaming responses enable perceived performance while maintaining quality

**Smallest Viable Change**:
- Single `/api/v1/query` endpoint (not multiple endpoints)
- Basic chatbot widget (simplified UI per spec - advanced features in Phase 4)
- No premature optimization (caching, batching) until baseline is measured
- Static FAQ fallback is a curated JSON file, not a complex system

**Security by Default**:
- All user input sanitized before processing
- Parameterized SQL queries (asyncpg) prevent injection
- Rate limiting enforced at API gateway level
- JWT validation for authenticated endpoints (optional in Phase 3)
- HTTPS-only for all external API calls

**Observability & Measurability**:
- Correlation IDs for all requests
- Structured JSON logging
- Token usage tracked per request (input/output tokens)
- Confidence scores returned with every response
- Response time metrics (p50/p95/p99) tracked

**Accessibility & Inclusivity**:
- Chat widget: WCAG 2.1 AA compliance (FR-030)
- Keyboard navigation for all interactive elements
- Screen reader support (ARIA labels, live regions)
- Focus management for message list
- Touch targets ≥ 44x44px on mobile

**Free Tier Sustainability**:
- Qdrant: 1GB limit → ~100k chunks max (well within limit for 10 chapters)
- Neon: 0.5GB limit → 30-day auto-purge of chat history
- OpenAI: Rate limiting to prevent quota exhaustion
- Token usage per query: target < 2000 tokens total

### Agent Ownership
**Primary**: RAGArchitect Agent (owns: `qdrant-vectorstore`, `neon-postgres`, `fastapi-backend`, `openai-agents-sdk`)
**Support**:
- DocusaurusBuilder Agent (owns: `react-components` for chatbot widget)
- AuthEngineer Agent (consultation for session management)
**Coordinator**: Orchestrator Agent (quality gate approval)

### Complexity Violations
*None identified - Phase follows YAGNI principle*

## Project Structure

### Documentation (this feature)

```text
specs/003-rag-chatbot-core/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   ├── openapi.yaml     # RAG API OpenAPI 3.1 spec
│   └── schemas.ts       # TypeScript types for frontend
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── app/
│   ├── __init__.py
│   ├── main.py              # FastAPI app initialization
│   ├── config.py            # Environment configuration
│   ├── models/
│   │   ├── __init__.py
│   │   ├── query.py         # QueryRequest, QueryResponse
│   │   ├── chat.py          # ChatSession, ChatMessage
│   │   └── citations.py     # SourceCitation
│   ├── routers/
│   │   ├── __init__.py
│   │   ├── query.py         # /api/v1/query endpoint
│   │   └── health.py        # /health endpoint
│   ├── services/
│   │   ├── __init__.py
│   │   ├── rag_service.py   # RAG pipeline orchestration
│   │   ├── vector_service.py # Qdrant operations
│   │   ├── llm_service.py   # OpenAI Chat Completions
│   │   ├── chat_service.py  # Chat history management
│   │   └── rate_limiter.py  # Rate limiting logic
│   └── middleware/
│       ├── __init__.py
│       ├── cors.py          # CORS configuration
│       ├── logging.py       # Request/response logging
│       └── rate_limit.py    # Rate limit middleware
├── tests/
│   ├── __init__.py
│   ├── conftest.py          # Pytest fixtures
│   ├── unit/
│   │   ├── test_rag_service.py
│   │   ├── test_vector_service.py
│   │   └── test_llm_service.py
│   ├── integration/
│   │   ├── test_query_endpoint.py
│   │   └── test_chat_history.py
│   └── benchmark/
│       ├── test_questions.json   # 50 benchmark questions
│       └── test_relevance.py     # NDCG@10 calculation
├── requirements.txt
├── Dockerfile
└── docker-compose.yml

src/components/
├── ChatbotWidget/
│   ├── index.tsx            # Main widget component
│   ├── ChatbotWidget.module.css
│   ├── ChatbotWidget.test.tsx
│   ├── ChatInput.tsx        # Input field + submit button
│   ├── MessageList.tsx      # Message display with citations
│   ├── SourceCitations.tsx  # Clickable source links
│   ├── ConfidenceIndicator.tsx  # Confidence score display
│   ├── LoadingState.tsx     # Streaming/loading indicators
│   └── types.ts             # TypeScript interfaces
└── ChatbotWidget.stories.tsx # Storybook (optional)

tests/
├── e2e/
│   └── chatbot.spec.ts      # Playwright E2E tests
└── components/
    └── ChatbotWidget.test.tsx

static/data/
└── faq-fallback.json        # Static FAQ for graceful degradation
```

**Structure Decision**: Web application structure with separate `backend/` (FastAPI Python) and frontend components in existing Docusaurus `src/components/`. Backend will be deployed independently to Railway/Render while frontend remains on GitHub Pages.

## Complexity Tracking

> **No violations identified** - This phase follows YAGNI principle strictly.

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| *None* | N/A | N/A |

## Post-Design Constitution Check (Phase 3 Complete)

### Phase 3 Quality Gates Remain Valid
<!-- Copy same checklist from Constitution Check above -->
- [ ] Vector database has 1000+ chunks
- [ ] `/query` API endpoint functional and documented
- [ ] NDCG@10 > 0.8 on 50-question test set
- [ ] Response time p95 < 3s (measured in CI)
- [ ] Zero hallucinations on test set
- [ ] ADR created for RAG architecture decisions
- [ ] PHR created documenting RAG implementation

### Design Artifacts Generated
- ✅ `research.md` - Research findings on RAG architecture, embedding models, streaming patterns, rate limiting strategies
- ✅ `data-model.md` - ChatSession, ChatMessage, SourceCitation, RateLimitState entities with relationships and validation rules
- ✅ `quickstart.md` - Local development setup guide with environment variables, database setup, and test commands
- ✅ `contracts/openapi.yaml` - OpenAPI 3.1 specification for `/api/v1/query` endpoint
- ✅ `contracts/schemas.ts` - TypeScript type definitions for frontend integration

### No New Complexity Violations

**Analysis**:
- **YAGNI Compliance**: Single endpoint, minimal services, no premature abstractions
- **Security**: Input sanitization, parameterized queries, rate limiting, CORS configuration
- **Performance**: Streaming responses, connection pooling, async operations
- **Accessibility**: WCAG 2.1 AA for chat widget, keyboard navigation, screen reader support
- **Free Tier**: Within Qdrant 1GB, Neon 0.5GB limits; auto-purge enabled

### Recommended ADRs

**Decision Detected**: RAG Architecture - OpenAI Chat Completions vs Agents SDK
- **Impact**: Determines response generation pattern, tool calling capabilities, extensibility
- **Alternatives**: LangChain, custom LLM orchestration, Anthropic Claude
- **Scope**: Cross-cutting - affects response quality, latency, cost

📋 Architectural decision detected: **Use OpenAI Chat Completions API with structured system prompts for RAG response generation**
   Document reasoning and tradeoffs? Run `/sp.adr rag-llm-architecture`

**Decision Detected**: Chat History Storage - Normalized vs Denormalized
- **Impact**: Query performance, storage efficiency, source citation navigation
- **Alternatives**: Embed citations in message JSON, separate citations table
- **Scope**: Database schema design

📋 Architectural decision detected: **Store source citations in separate normalized table with foreign key references**
   Document reasoning and tradeoffs? Run `/sp.adr chat-history-schema`

*Waiting for user consent before creating ADRs.*

---
