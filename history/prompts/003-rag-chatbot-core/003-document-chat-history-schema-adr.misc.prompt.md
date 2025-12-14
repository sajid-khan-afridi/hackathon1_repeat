---
id: 003
title: Document chat history schema ADR
stage: misc
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5
feature: 003-rag-chatbot-core
branch: 003-rag-chatbot-core
user: system
command: /sp.adr chat-history-schema
labels: ["adr", "documentation", "database", "schema-design"]
links:
  spec: specs/003-rag-chatbot-core/spec.md
  ticket: null
  adr: history/adr/008-chat-history-schema.md
  pr: null
files:
  - history/adr/008-chat-history-schema.md
tests:
  - none
---

## Prompt

chat-history-schema

## Response snapshot

Analyzed planning artifacts from `specs/003-rag-chatbot-core/plan.md` and `data-model.md` to identify architecturally significant decisions requiring ADR documentation.

Identified decision: **Chat History Schema Design** - normalized relational schema with separate tables for sessions, messages, and citations vs denormalized/embedded alternatives.

Applied significance test:
1. Impact: YES - Long-term consequence for data model, query patterns, storage efficiency
2. Alternatives: YES - Normalized vs denormalized, embedded citations, document stores, EAV pattern
3. Scope: YES - Cross-cutting concern affecting backend persistence, API responses, frontend state

Created ADR-008 documenting:
- **Decision**: Normalized relational schema with separate ChatSession, ChatMessage, and SourceCitation tables
- **Positive Consequences**: Query efficiency, source traceability, referential integrity, storage efficiency, flexible querying, extensibility, type safety, cost management, analytics ready
- **Negative Consequences**: Join complexity, write amplification, migration overhead, query count, index maintenance, cascade deletes
- **Alternatives**: Denormalized with embedded citations, document store (MongoDB/Firestore), EAV pattern, time-series partitioning

## Outcome

- ✅ Impact: Created ADR-008 documenting chat history schema design with clustered decision, explicit alternatives, and comprehensive tradeoffs
- 🧪 Tests: No tests (documentation task)
- 📁 Files: 1 created (history/adr/008-chat-history-schema.md)
- 🔁 Next prompts: `/sp.tasks` to generate implementation tasks for RAG chatbot core
- 🧠 Reflection: Successfully identified and documented architecturally significant decision from plan.md recommendation. Decision properly clustered (schema design as integrated solution) with clear alternatives and tradeoffs.

## Evaluation notes (flywheel)

- Failure modes observed: None - ADR properly clustered schema components, listed 4 alternatives with rationale
- Graders run and results (PASS/FAIL):
  - Clustering: PASS - Grouped related schema decisions (sessions, messages, citations) into single ADR
  - Alternatives: PASS - 4 distinct alternatives listed with rejection rationale
  - Tradeoffs: PASS - 9 positive and 6 negative consequences documented
  - Conciseness: PASS - Detailed but focused on architectural significance
- Prompt variant (if applicable): Standard /sp.adr workflow
- Next experiment (smallest change to try): N/A - workflow executed correctly
