# RAG Chatbot Backend

FastAPI backend for the RAG-powered chatbot system.

## Setup

1. Create a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Set up environment variables:
```bash
cp ../.env.example .env
# Edit .env with your actual API keys and configuration
```

4. Run the development server:
```bash
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

## API Documentation

Once running, visit:
- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

## Testing

Run all tests:
```bash
pytest
```

Run with coverage:
```bash
pytest --cov=app --cov-report=html
```

Run specific test categories:
```bash
pytest -m "not slow"                    # Skip slow tests
pytest -m integration                   # Run only integration tests
pytest -m benchmark                     # Run only benchmark tests
```

## Code Quality

Format code:
```bash
black .
```

Check linting:
```bash
flake8 .
```

Type checking:
```bash
mypy .
```

## Project Structure

```
backend/
├── app/
│   ├── models/         # Pydantic models
│   ├── routers/        # API routes
│   ├── services/       # Business logic
│   ├── middleware/     # FastAPI middleware
│   ├── migrations/     # Database migrations
│   └── main.py         # FastAPI app entry point
├── tests/
│   ├── unit/           # Unit tests
│   ├── integration/    # Integration tests
│   └── benchmark/      # Performance tests
├── requirements.txt    # Python dependencies
├── pyproject.toml      # Tool configuration
└── conftest.py        # Pytest fixtures
```