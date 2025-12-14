---
id: 001
title: Create RAG Chatbot Core Specification
stage: spec
date: 2025-12-14
surface: agent
model: claude-opus-4-5-20251101
feature: 001-rag-chatbot-core
branch: 001-rag-chatbot-core
user: sajid-khan-afridi
command: /sp.specify
labels: ["rag", "chatbot", "specification", "phase-3"]
links:
  spec: specs/001-rag-chatbot-core/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot-core/spec.md
  - specs/001-rag-chatbot-core/checklists/requirements.md
  - history/prompts/001-rag-chatbot-core/001-create-rag-chatbot-spec.spec.prompt.md
tests:
  - Specification quality checklist validation (all items PASS)
---

## Prompt

Phase 3 RAG Chatbot Core - Build the AI-powered question answering system for the robotics textbook. This phase implements the complete RAG pipeline: (1) Vector indexing of 10 MDX chapters from Phase 2 into Qdrant using sentence-transformers/all-MiniLM-L6-v2 (384-dim, cosine distance, HNSW m=16 ef_construct=100), collection name `robotics_textbook_chapters` with payload indexing on chapter_id, tags, difficulty_level. (2) FastAPI backend with `/api/v1/query` endpoint accepting JSON `{"query": "string", "user_id": "string?", "filters": {"module": int}?, "top_k": int?}` returning `{"answer": "string", "sources": [{"chapter_id", "title", "score", "excerpt"}], "confidence": float, "chat_id": "string", "tokens_used": {"retrieval", "generation", "total"}}`. (3) OpenAI Agents SDK integration using GPT-4o-mini for response generation with system prompt enforcing source attribution, confidence scoring, and no hallucination. (4) Chat history persistence in Neon Postgres tables: `chat_sessions(id, user_id, created_at)`, `chat_messages(id, session_id, role, content, sources_json, created_at)` with 30-day retention. (5) Basic chatbot React widget component for Docusaurus with input field, message history, source citations, and loading states.

Agent orchestration: rag-chatbot-agent owns RAG pipeline implementation using qdrant-vectorstore, openai-agents-sdk, neon-postgres, and rag-pipeline skills. backend-api-agent builds FastAPI endpoint using fastapi-backend skill. ui-component-agent creates chatbot widget using react-components skill with Docusaurus integration.

Quality gates (mandatory): NDCG@10 > 0.8 on 50-question test set (from tests/search-queries.json covering all 10 chapters), p95 latency < 3s (budget: embed 500ms + search 1s + generate 1.5s), zero hallucinations (faithfulness test: answers must cite retrieved chunks only), API documented with OpenAPI 3.1. Performance budgets: embedding batch < 500ms per query, Qdrant search top-10 < 1s, LLM generation streaming < 1.5s first token.

User scenarios to specify: (P1) Student asks textbook question, (P1) Student continues conversation, (P2) Student filters search to specific module, (P2) Student asks off-topic question, (P3) System handles high load.

Free tier constraints: Qdrant Cloud 1GB, Neon 0.5GB with 30-day auto-purge, OpenAI rate limits. Error handling: OpenAI timeout -> 503 with retry message, Qdrant unavailable -> fallback to static FAQ, no relevant results -> "I couldn't find information about that topic" with suggested search terms.

Dependencies: Phase 2 complete (10 MDX chapters indexed), env-setup skill provides QDRANT_URL, OPENAI_API_KEY, DATABASE_URL.

Non-goals: Multi-turn complex reasoning, voice input, image understanding, real-time content updates, advanced analytics dashboard.

## Response snapshot

Created comprehensive specification for RAG Chatbot Core feature:

**Branch**: `001-rag-chatbot-core`

**Deliverables**:
1. `specs/001-rag-chatbot-core/spec.md` - Full feature specification with:
   - 5 prioritized user stories (2 P1, 2 P2, 1 P3)
   - 30 functional requirements across 6 categories
   - 15 measurable success criteria
   - 8 edge cases with graceful handling
   - 6 key entities documented
   - Assumptions, dependencies, and risks identified

2. `specs/001-rag-chatbot-core/checklists/requirements.md` - Quality validation checklist (all items PASS)

**Key Decisions**:
- No [NEEDS CLARIFICATION] markers needed - user input was comprehensive
- Success criteria kept technology-agnostic while honoring model constraints
- Rate limits (10/50 queries per hour) treated as business requirements
- Phase 2 dependency explicitly documented as prerequisite

## Outcome

- ✅ Impact: Created complete specification ready for architectural planning
- 🧪 Tests: Specification quality checklist - all 12 validation items PASS
- 📁 Files: 2 files created (spec.md, requirements.md checklist)
- 🔁 Next prompts: `/sp.clarify` for refinement (optional), `/sp.plan` for architecture, `/sp.tasks` after planning
- 🧠 Reflection: User input was exceptionally detailed, eliminating need for clarification questions

## Evaluation notes (flywheel)

- Failure modes observed: None - comprehensive input enabled complete spec
- Graders run and results (PASS/FAIL): Quality checklist validation - PASS (12/12 items)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test spec with stakeholder review for clarity
