---
id: 003
title: Phase 9 Polish - Benchmarks, Docker, Metrics, Linting
stage: green
date: 2025-12-15
surface: agent
model: claude-opus-4-5
feature: 003-rag-chatbot-core
branch: main
user: sajid-khan-afridi
command: manual implementation
labels: ["polish", "benchmarks", "docker", "metrics", "linting"]
links:
  spec: specs/003-rag-chatbot-core/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - backend/tests/benchmark/test_questions.json
  - backend/tests/benchmark/test_relevance.py
  - backend/tests/benchmark/test_faithfulness.py
  - backend/tests/benchmark/__init__.py
  - backend/tests/benchmark/conftest.py
  - backend/tests/__init__.py
  - backend/app/middleware/logging.py
  - backend/app/routers/health.py
  - backend/Dockerfile
  - backend/docker-compose.yml
  - backend/pyproject.toml
  - backend/conftest.py
  - eslint.config.js
  - specs/003-rag-chatbot-core/contracts/openapi.yaml
  - specs/003-rag-chatbot-core/tasks.md
tests:
  - backend/tests/benchmark/test_relevance.py (18 passed, 3 skipped)
  - backend/tests/benchmark/test_faithfulness.py (included in above)
---

## Prompt

Complete Phase 9: Polish & Cross-Cutting Concerns for the RAG Chatbot Core feature. This includes:
1. Create benchmark test set (50 questions covering all 10 chapters)
2. Implement NDCG@10 relevance calculation
3. Implement faithfulness testing (zero hallucinations)
4. Add structured JSON logging with correlation IDs
5. Implement metrics tracking (p50/p95/p99 response times)
6. Create Dockerfile and docker-compose.yml
7. Validate environment variables documentation
8. Run backend linters (black, flake8)
9. Run frontend linters (ESLint, TypeScript)
10. Validate quickstart.md instructions
11. Update OpenAPI spec for new endpoints

## Response snapshot

### Completed Tasks (T093-T105)

**Benchmarks (T093-T096)**
- Created `backend/tests/benchmark/test_questions.json` with 50 questions covering all 10 robotics chapters
- Implemented NDCG@10 calculation in `test_relevance.py` with DCG/NDCG algorithms
- Implemented faithfulness testing in `test_faithfulness.py` with claim extraction and verification
- All 18 unit tests pass; 3 integration tests skipped (require live RAG service)

**Logging & Metrics (T097-T098)**
- Enhanced `backend/app/middleware/logging.py` with:
  - `MetricsCollector` class for response time tracking
  - Thread-safe percentile calculations (p50/p95/p99)
  - Per-endpoint metrics tracking
  - Production JSON formatter with ISO 8601 timestamps
- Added `/metrics`, `/metrics/percentiles`, `/metrics/endpoint/{path}` endpoints to health router

**Docker (T099-T100)**
- Dockerfile already existed; verified multi-stage build structure
- docker-compose.yml already existed; verified service configuration

**Validation (T101-T105)**
- Verified all 22 environment variables documented in `.env.example`
- Ran black formatter (20 files reformatted)
- Ran flake8 (minor unused import warnings)
- Created `eslint.config.js` for ESLint v9 with TypeScript support
- Ran frontend linting (43 issues: 5 errors, 38 warnings - mostly `any` types)
- Verified quickstart.md instructions are accurate
- Updated OpenAPI spec with new metrics endpoints and MetricsResponse schema

## Outcome

- Impact: Phase 9 complete - 13/15 tasks done (T106, T107 remaining for PHR/ADR)
- Tests: 18 benchmark tests passing, integration tests ready for live service
- Files: 15 files created/modified
- Next prompts: Create ADR for RAG architecture decisions
- Reflection: Benchmark framework provides solid foundation for measuring NDCG@10 > 0.8 target

## Evaluation notes (flywheel)

- Failure modes observed: Python 3.14 compatibility issues with pydantic-core build
- Graders run and results (PASS/FAIL): pytest benchmark tests PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Run full integration tests with live Qdrant/OpenAI services
