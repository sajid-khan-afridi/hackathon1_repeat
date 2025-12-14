---
id: 002
title: ADR RAG LLM Architecture Stack
stage: plan
date: 2025-12-14
surface: agent
model: claude-opus-4-5-20251101
feature: 003-rag-chatbot-core
branch: 003-rag-chatbot-core
user: developer
command: /sp.adr rag-llm-architecture
labels: ["adr", "rag", "llm", "architecture", "openai"]
links:
  spec: specs/003-rag-chatbot-core/spec.md
  ticket: null
  adr: history/adr/007-rag-llm-architecture-stack.md
  pr: null
files:
  - history/adr/007-rag-llm-architecture-stack.md
tests:
  - null
---

## Prompt

/sp.adr rag-llm-architecture

User requested documentation of the RAG LLM Architecture decision detected during the planning phase. The plan.md file identified this as an architecturally significant decision requiring an ADR.

## Response snapshot

Created ADR-007: RAG LLM Architecture Stack documenting the decision to use:
- GPT-4o-mini for response generation (cost-effective educational Q&A)
- text-embedding-3-small for query embedding (1536 dimensions)
- OpenAI Chat Completions API (direct integration, no orchestration framework)
- Server-Sent Events for streaming responses
- Strict system prompt + weighted confidence scoring for grounding

Alternatives considered: LangChain + Multiple Providers, OpenAI Agents SDK, Self-hosted Llama 3.1, Anthropic Claude, WebSocket streaming.

## Outcome

- ✅ Impact: ADR-007 created documenting RAG LLM architecture decisions
- 🧪 Tests: N/A (documentation artifact)
- 📁 Files: 1 file created (history/adr/007-rag-llm-architecture-stack.md)
- 🔁 Next prompts: /sp.adr chat-history-schema (second pending ADR), /sp.tasks (generate implementation tasks)
- 🧠 Reflection: Decision cluster approach grouped LLM model, embedding, streaming, and grounding strategy into single ADR for coherence

## Evaluation notes (flywheel)

- Failure modes observed: None - decision was clearly significant and well-documented in research.md
- Graders run and results (PASS/FAIL): PASS - ADR documents clustered decision with explicit alternatives and tradeoffs
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Consider automated ADR creation script to reduce manual file creation
