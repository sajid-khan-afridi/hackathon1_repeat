# Physical AI & Humanoid Robotics Textbook

Live demo: https://sajid-khan-afridi.github.io/hackathon1_repeat/

## Overview

An interactive, AI-powered learning platform for Physical AI and Humanoid Robotics. This project combines a comprehensive textbook with intelligent personalization features to adapt content to each learner's experience level, learning goals, and hardware access.

## Features

### Phase 4B: Personalization Engine ✨ NEW

The platform now includes advanced personalization features that adapt the learning experience to each user:

#### 1. **Skill Level Classification**
- Automatic classification into beginner/intermediate/advanced tiers
- Based on programming experience and ROS familiarity
- Deterministic weighted scoring algorithm
- Sub-500ms classification time
- **API**: `GET /api/v1/skill-level`

#### 2. **Chapter Progress Tracking**
- Track started, completed, and bookmarked chapters
- Auto-start timer (10-second threshold)
- Completion detection via scroll position
- Independent bookmark functionality
- **API**: `POST /api/v1/progress/{start,complete,bookmark}`
- **API**: `GET /api/v1/progress` (with filters)

#### 3. **Smart Chapter Recommendations**
- Multi-factor scoring algorithm:
  - Skill level match (35% weight)
  - Learning goal alignment (30% weight)
  - Hardware compatibility (20% weight)
  - Prerequisite readiness (15% weight)
- 1-hour recommendation cache with auto-invalidation
- Up to 3 recommendations per request
- Target relevance score > 0.75
- **API**: `GET /api/v1/recommendations`

#### 4. **Adaptive Content Depth**
- Dynamic content filtering based on user profile
- Language-specific code examples (Python/C++)
- Experience level-appropriate explanations
- Hardware-specific instructions
- WCAG 2.1 AA accessible

#### 5. **Profile-Aware RAG Chatbot**
- Personalized answer complexity
- Preferred language code examples
- Hardware-specific guidance
- Sub-2s response time
- **API**: `POST /api/v1/chat/query` (enhanced)

### Previous Features

- **Phase 3: RAG Chatbot Core** - AI-powered Q&A system with vector search
- **Phase 4A: Authentication** - Better Auth integration with Google OAuth
- **Phases 1-2: Content Infrastructure** - Docusaurus-based textbook with MDX chapters

## Architecture

### Backend (FastAPI + Python)
- **Framework**: FastAPI 0.104+
- **Database**: Neon Serverless Postgres
- **Caching**: In-memory TTLCache (3600s TTL)
- **Authentication**: Better Auth (JWT tokens)
- **LLM Integration**: OpenAI Agents SDK

### Frontend (React + TypeScript)
- **Framework**: Docusaurus v3
- **Language**: TypeScript 5.x (strict mode)
- **State Management**: React Context
- **UI Components**: Custom React components

### Key Services
- `ClassificationService` - Skill level classification
- `ProgressTrackingService` - Chapter progress CRUD
- `RecommendationService` - Smart recommendations
- `PersonalizationService` - Profile context generation
- `RAGService` - Retrieval-augmented generation

## API Documentation

### Personalization Endpoints

#### GET /api/v1/skill-level
Get user's skill level classification.

**Response**:
```json
{
  "user_id": "uuid",
  "skill_level": "intermediate",
  "calculated_at": "2025-12-23T00:00:00Z",
  "based_on_profile": { ... }
}
```

#### GET /api/v1/recommendations
Get personalized chapter recommendations.

**Query Parameters**:
- `force_refresh` (optional): Bypass cache

**Response**:
```json
{
  "user_id": "uuid",
  "recommendations": [
    {
      "chapter_id": "module-2/ros-publishers",
      "relevance_score": 0.92,
      "reason": "Matches your intermediate skill level...",
      "title": "Creating ROS 2 Publishers"
    }
  ],
  "from_cache": false
}
```

#### POST /api/v1/progress/start
Mark chapter as started.

**Request**:
```json
{
  "chapter_id": "module-1/ros-intro"
}
```

**Response**:
```json
{
  "id": "uuid",
  "user_id": "uuid",
  "chapter_id": "module-1/ros-intro",
  "status": "started",
  "started_at": "2025-12-23T00:00:00Z"
}
```

For complete API documentation, see:
- OpenAPI Spec: `specs/1-personalization-engine/contracts/personalization-api.yaml`
- Interactive Docs: `http://localhost:8000/docs` (when backend running)

## Getting Started

### Prerequisites
- Python 3.11+
- Node.js 18+
- PostgreSQL (Neon account recommended)
- OpenAI API key

### Quick Start

1. **Clone the repository**
```bash
git clone https://github.com/your-org/hackathon1_repeat.git
cd hackathon1_repeat
```

2. **Backend Setup**
```bash
cd backend
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env with your credentials

# Run migrations
psql $DATABASE_URL < app/migrations/008_personalization_schema.sql

# Start server
uvicorn app.main:app --reload
```

3. **Frontend Setup**
```bash
cd ..  # Back to root
npm install
npm run start
```

4. **Access the application**
- Frontend: http://localhost:3000
- Backend API: http://localhost:8000
- API Docs: http://localhost:8000/docs

For detailed setup instructions, see `specs/1-personalization-engine/quickstart.md`

## Project Structure

```
├── backend/
│   ├── app/
│   │   ├── models/          # Pydantic models
│   │   ├── services/        # Business logic
│   │   ├── routers/         # API endpoints
│   │   └── migrations/      # Database migrations
│   └── tests/               # Unit & integration tests
├── src/
│   ├── components/          # React components
│   ├── hooks/               # Custom React hooks
│   ├── services/            # API clients
│   └── pages/               # Docusaurus pages
├── docs/                    # MDX content
└── specs/                   # Feature specifications
```

## Testing

### Backend Tests
```bash
cd backend
pytest tests/ -v --cov=app
```

### Frontend Tests
```bash
npm test
```

### Integration Tests
```bash
pytest tests/integration/ -v -m integration
```

## Performance

- **Skill Classification**: < 500ms
- **Personalization Response**: < 2s end-to-end
- **Recommendation API**: < 1s (cached), < 100ms (cache hit)
- **Progress Tracking**: < 300ms
- **RAG Chatbot**: < 3s (including personalization)

## Contributing

See `CONTRIBUTING.md` for development guidelines and coding standards.

## License

MIT License - See `LICENSE` file for details

## Support

- Documentation: `specs/1-personalization-engine/`
- Issues: [GitHub Issues](https://github.com/your-org/hackathon1_repeat/issues)
- API Docs: http://localhost:8000/docs

## Acknowledgments

Built with:
- [FastAPI](https://fastapi.tiangolo.com/)
- [Docusaurus](https://docusaurus.io/)
- [Better Auth](https://www.better-auth.com/)
- [Neon](https://neon.tech/)
- [OpenAI](https://platform.openai.com/)
