# Quickstart: RAG Chatbot Core

**Feature**: 003-rag-chatbot-core
**Date**: 2025-12-14

This guide helps you set up the local development environment for the RAG Chatbot Core feature.

---

## Prerequisites

### Required Software

| Software | Version | Purpose |
|----------|---------|---------|
| Python | 3.11+ | Backend runtime |
| Node.js | 20+ | Frontend tooling |
| Docker | Latest | Container runtime (optional) |
| Git | Latest | Version control |

### External Services

| Service | Purpose | Free Tier |
|---------|---------|-----------|
| [Qdrant Cloud](https://cloud.qdrant.io) | Vector database | 1GB storage |
| [OpenAI](https://platform.openai.com) | Embeddings + LLM | Pay-per-use |
| [Neon](https://neon.tech) | PostgreSQL database | 0.5GB storage |

---

## 1. Clone and Setup

```bash
# Clone the repository
git clone https://github.com/sajid-khan-afridi/hackathon1_repeat.git
cd hackathon1_repeat

# Checkout the feature branch
git checkout 003-rag-chatbot-core

# Install frontend dependencies
npm install
```

---

## 2. Environment Configuration

### Create Environment File

Create a `.env` file in the repository root:

```bash
# Copy the template
cp .env.example .env
```

### Required Environment Variables

```env
# =============================================================================
# OpenAI Configuration
# =============================================================================
OPENAI_API_KEY=sk-your-openai-api-key-here

# =============================================================================
# Qdrant Vector Database
# =============================================================================
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Collection name for textbook content
QDRANT_COLLECTION=robotics_textbook

# =============================================================================
# Neon PostgreSQL Database
# =============================================================================
DATABASE_URL=postgres://user:password@ep-your-endpoint.neon.tech/dbname?sslmode=require

# =============================================================================
# Backend Configuration
# =============================================================================
# Environment: development | production
ENVIRONMENT=development

# API host and port
API_HOST=0.0.0.0
API_PORT=8000

# CORS allowed origins (comma-separated)
CORS_ORIGINS=http://localhost:3000,http://localhost:8000

# =============================================================================
# Rate Limiting
# =============================================================================
# Queries per hour for anonymous users
RATE_LIMIT_ANONYMOUS=10

# Queries per hour for authenticated users
RATE_LIMIT_AUTHENTICATED=50

# =============================================================================
# Logging
# =============================================================================
LOG_LEVEL=DEBUG
LOG_FORMAT=json

# =============================================================================
# Optional: Redis (for production rate limiting)
# =============================================================================
# REDIS_URL=redis://localhost:6379
```

---

## 3. Backend Setup

### Create Python Virtual Environment

```bash
# Navigate to backend directory (will be created during implementation)
cd backend

# Create virtual environment
python -m venv venv

# Activate virtual environment
# Windows
.\venv\Scripts\activate
# macOS/Linux
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### Backend Dependencies (requirements.txt)

```txt
# Core framework
fastapi==0.109.0
uvicorn[standard]==0.27.0
pydantic==2.5.3
pydantic-settings==2.1.0

# OpenAI integration
openai==1.6.1

# Qdrant vector database
qdrant-client==1.7.0

# PostgreSQL
asyncpg==0.29.0

# Rate limiting
slowapi==0.1.9

# HTTP client
httpx==0.26.0

# Utilities
python-dotenv==1.0.0
python-multipart==0.0.6

# Testing
pytest==7.4.4
pytest-asyncio==0.23.3
pytest-cov==4.1.0
httpx==0.26.0

# Type checking
mypy==1.8.0
types-redis==4.6.0
```

### Run Database Migrations

```bash
# Run the migration script
python -m app.migrations.run

# Or manually with psql
psql $DATABASE_URL -f app/migrations/001_create_chat_tables.sql
psql $DATABASE_URL -f app/migrations/002_add_auto_purge.sql
```

### Start the Backend

```bash
# Development mode with auto-reload
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

# Or use the provided script
python -m app.main
```

The API will be available at `http://localhost:8000`.

API Documentation:
- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`

---

## 4. Frontend Setup

### Start Docusaurus Development Server

```bash
# From repository root
npm run start
```

The documentation site will be available at `http://localhost:3000`.

### Build for Production

```bash
npm run build
npm run serve
```

---

## 5. Verify Setup

### Health Check

```bash
# Check API health
curl http://localhost:8000/api/v1/health

# Expected response:
# {
#   "status": "healthy",
#   "version": "1.0.0",
#   "dependencies": {
#     "database": "healthy",
#     "vectorStore": "healthy",
#     "llmService": "healthy"
#   }
# }
```

### Test Query

```bash
# Submit a test query
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'

# Expected response:
# {
#   "answer": "ROS 2 (Robot Operating System 2) is...",
#   "sources": [...],
#   "confidence": 0.85,
#   "sessionId": "...",
#   "tokensUsed": {...}
# }
```

---

## 6. Running Tests

### Backend Tests

```bash
cd backend

# Run all tests
pytest

# Run with coverage
pytest --cov=app --cov-report=html

# Run specific test file
pytest tests/unit/test_rag_service.py

# Run benchmark tests
pytest tests/benchmark/test_relevance.py -v
```

### Frontend Tests

```bash
# From repository root

# Run unit tests
npm run test

# Run with coverage
npm run test:coverage

# Run E2E tests
npm run test:e2e
```

### Lint and Type Check

```bash
# Backend
cd backend
mypy app/
black app/ --check
flake8 app/

# Frontend
npm run lint
npm run typecheck
```

---

## 7. Development Workflow

### Making Changes

1. Create a feature branch from `003-rag-chatbot-core`
2. Make your changes
3. Run tests: `pytest` (backend), `npm run test` (frontend)
4. Run linters: `black app/`, `npm run lint`
5. Commit with descriptive message
6. Push and create PR

### API Development

1. Define endpoint in `backend/app/routers/`
2. Add Pydantic models in `backend/app/models/`
3. Implement service logic in `backend/app/services/`
4. Add tests in `backend/tests/`
5. Update OpenAPI spec if needed

### Frontend Development

1. Create component in `src/components/ChatbotWidget/`
2. Add TypeScript types in `types.ts`
3. Add CSS modules in `*.module.css`
4. Add tests in `*.test.tsx`
5. Import and use in documentation pages

---

## 8. Troubleshooting

### Common Issues

#### CORS Errors

```
Access to fetch at 'http://localhost:8000' from origin 'http://localhost:3000' has been blocked by CORS policy
```

**Solution**: Ensure `CORS_ORIGINS` in `.env` includes your frontend URL.

#### Qdrant Connection Failed

```
Connection refused: qdrant_client.http.exceptions.ConnectionError
```

**Solution**: Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct.

#### OpenAI Rate Limit

```
openai.error.RateLimitError: You exceeded your current quota
```

**Solution**: Check your OpenAI API quota or implement request queuing.

#### Database Connection Failed

```
asyncpg.exceptions.InvalidPasswordError: password authentication failed
```

**Solution**: Verify `DATABASE_URL` connection string is correct.

### Debug Mode

Enable verbose logging:

```env
LOG_LEVEL=DEBUG
```

Enable SQL query logging:

```python
# In app/config.py
SQLALCHEMY_ECHO = True
```

---

## 9. Docker Setup (Optional)

### Build and Run with Docker Compose

```bash
# Build images
docker-compose build

# Start services
docker-compose up -d

# View logs
docker-compose logs -f backend

# Stop services
docker-compose down
```

### docker-compose.yml

```yaml
version: '3.8'

services:
  backend:
    build:
      context: ./backend
      dockerfile: Dockerfile
    ports:
      - "8000:8000"
    environment:
      - OPENAI_API_KEY=${OPENAI_API_KEY}
      - QDRANT_URL=${QDRANT_URL}
      - QDRANT_API_KEY=${QDRANT_API_KEY}
      - DATABASE_URL=${DATABASE_URL}
    volumes:
      - ./backend:/app
    command: uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

  frontend:
    build:
      context: .
      dockerfile: Dockerfile.frontend
    ports:
      - "3000:3000"
    volumes:
      - .:/app
      - /app/node_modules
    command: npm run start
```

---

## 10. Next Steps

After completing setup:

1. **Verify indexed content**: Ensure Phase 2 content indexing is complete
2. **Create benchmark test set**: Develop 50 questions covering all chapters
3. **Implement RAG pipeline**: Follow the tasks in `tasks.md` (generated by `/sp.tasks`)
4. **Test and measure**: Run benchmark tests, verify NDCG@10 > 0.8
5. **Deploy**: Use Railway/Render for backend, GitHub Pages for frontend

---

## References

- Spec: [spec.md](./spec.md)
- Data Model: [data-model.md](./data-model.md)
- Research: [research.md](./research.md)
- API Contract: [contracts/openapi.yaml](./contracts/openapi.yaml)
- TypeScript Types: [contracts/schemas.ts](./contracts/schemas.ts)
