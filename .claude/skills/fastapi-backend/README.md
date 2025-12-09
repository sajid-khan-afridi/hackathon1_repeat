# FastAPI Backend Skill

This skill creates FastAPI backend services for REST API endpoints, user management, and RAG chatbot functionality.

## Features

- **FastAPI Application**: Complete FastAPI setup with middleware and configuration
- **CORS Support**: Pre-configured for Docusaurus frontend integration
- **JWT Authentication**: Token-based authentication middleware
- **Request/Response Logging**: Comprehensive logging for debugging and monitoring
- **Health Check**: Built-in health check endpoint
- **Three Endpoint Types**:
  - `rag_query`: For RAG chatbot queries
  - `user_auth`: For user authentication and authorization
  - `personalization`: For user preference management

## Usage

### Basic Usage

```bash
skill fastapi-backend --endpoint_type rag_query --route_path "/query"
```

### Advanced Usage

```bash
skill fastapi-backend \
  --endpoint_type rag_query \
  --route_path "/query" \
  --request_model "QueryRequest(question: str, context: Optional[str] = None)" \
  --response_model "QueryResponse(answer: str, sources: List[str], confidence: float)" \
  --http_method POST \
  --auth_required true
```

## Parameters

### Required Parameters

- `endpoint_type`: Type of endpoint to create
  - `"rag_query"`: RAG chatbot query endpoint
  - `"user_auth"`: User authentication endpoint
  - `"personalization"`: User personalization endpoint

- `route_path`: Route path for the endpoint (e.g., `/query`, `/auth/login`)

### Optional Parameters

- `request_model`: Pydantic model definition for request body
- `response_model`: Pydantic model definition for response body
- `http_method`: HTTP method (default: `"POST"`)
- `auth_required`: Whether JWT authentication is required (default: `true`)

## Generated Structure

```
backend/
├── app/
│   ├── main.py              # Main FastAPI application
│   ├── config.py            # Configuration settings
│   ├── routers/             # API routers
│   │   ├── chat.py          # RAG query endpoints
│   │   ├── auth.py          # Authentication endpoints
│   │   └── personalize.py   # Personalization endpoints
│   ├── models/              # Pydantic models
│   │   └── __init__.py      # All model definitions
│   └── services/            # Business logic services
│       ├── rag_service.py
│       ├── auth_service.py
│       └── personalization_service.py
└── requirements.txt         # Python dependencies
```

## Examples

### 1. RAG Query Endpoint

```bash
skill fastapi-backend \
  --endpoint_type rag_query \
  --route_path "/query" \
  --request_model "QueryRequest(question: str, context: Optional[str] = None)" \
  --response_model "QueryResponse(answer: str, sources: List[str], confidence: float)"
```

**Generated Models:**
```python
class QueryRequest(BaseModel):
    question: str = Field(..., description="The question to answer")
    context: Optional[str] = Field(None, description="Additional context for the query")
    max_tokens: Optional[int] = Field(500, description="Maximum tokens in response")
    temperature: Optional[float] = Field(0.7, description="Response temperature")

class QueryResponse(BaseModel):
    answer: str = Field(..., description="The answer to the question")
    sources: List[str] = Field(default_factory=list, description="Source documents used")
    confidence: float = Field(..., description="Confidence score of the answer")
    query_time: Optional[float] = Field(None, description="Time taken to process query")
```

### 2. User Authentication

```bash
skill fastapi-backend \
  --endpoint_type user_auth \
  --route_path "/auth/login" \
  --request_model "LoginRequest(email: str, password: str)" \
  --response_model "TokenResponse(access_token: str, token_type: str = 'bearer')" \
  --auth_required false
```

**Generated Models:**
```python
class LoginRequest(BaseModel):
    email: EmailStr = Field(..., description="User email")
    password: str = Field(..., description="User password")

class TokenResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"
    expires_in: Optional[int] = None
```

### 3. Personalization Settings

```bash
skill fastapi-backend \
  --endpoint_type personalization \
  --route_path "/user/preferences" \
  --request_model "PreferencesUpdate(theme: str, language: str, notifications: bool)" \
  --response_model "UserPreferences(user_id: str, preferences: dict)"
```

**Generated Models:**
```python
class PreferencesUpdate(BaseModel):
    theme: ThemeEnum = ThemeEnum.LIGHT
    language: LanguageEnum = LanguageEnum.EN
    notifications: bool = True
    timezone: str = "UTC"

class UserPreferences(BaseModel):
    user_id: str
    preferences: PreferencesBase
    updated_at: datetime = Field(default_factory=datetime.now)
```

## Configuration

The skill generates a comprehensive `config.py` file with:

- API configuration
- Security settings (JWT)
- Database connection settings
- CORS configuration
- RAG system settings
- Logging configuration

## Running the Application

1. Install dependencies:
```bash
cd backend
pip install -r requirements.txt
```

2. Run the application:
```bash
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

3. Access the API documentation:
   - Swagger UI: `http://localhost:8000/docs`
   - ReDoc: `http://localhost:8000/redoc`
   - Health Check: `http://localhost:8000/health`

## Environment Variables

Create a `.env` file in the backend directory:

```env
SECRET_KEY=your-secret-key-change-in-production
DATABASE_URL=postgresql://user:password@localhost/dbname
VECTOR_STORE_URL=http://localhost:6333
OPENAI_API_KEY=your-openai-api-key
```

## Security Features

- **JWT Authentication**: Token-based authentication with configurable expiration
- **Password Hashing**: bcrypt for secure password storage
- **CORS Protection**: Configured for frontend domains
- **Input Validation**: Pydantic models for request/response validation
- **Logging**: Comprehensive request/response logging

## Integration with Docusaurus

The CORS is pre-configured to work with Docusaurus development servers:

- `http://localhost:3000` (default Docusaurus port)
- `http://localhost:8080` (alternative port)

## Dependencies

The generated `requirements.txt` includes:

- **FastAPI**: Modern web framework
- **Pydantic**: Data validation and settings management
- **python-jose**: JWT handling
- **passlib**: Password hashing
- **langchain**: RAG framework
- **openai**: OpenAI API integration
- **qdrant-client**: Vector database client
- **uvicorn**: ASGI server

## Next Steps

After generating the backend:

1. **Implement Business Logic**: Replace placeholder implementations in service files
2. **Set Up Database**: Configure your database connection and models
3. **Configure RAG**: Set up your vector store and embedding models
4. **Add Tests**: Write unit and integration tests
5. **Deploy**: Configure deployment settings for production