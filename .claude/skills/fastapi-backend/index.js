const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');

class FastAPIBackend {
  constructor() {
    this.templatesDir = path.join(__dirname, 'templates');
    this.utilsDir = path.join(__dirname, 'utils');
    this.examplesDir = path.join(__dirname, 'examples');
  }

  async execute(params) {
    const { endpoint_type, route_path, request_model, response_model, http_method = 'POST', auth_required = true } = params;

    // Create backend directory structure
    const backendDir = path.join(process.cwd(), 'backend');
    const appDir = path.join(backendDir, 'app');
    const routersDir = path.join(appDir, 'routers');
    const modelsDir = path.join(appDir, 'models');
    const servicesDir = path.join(appDir, 'services');

    // Ensure directories exist
    this.ensureDirectoryExists(backendDir);
    this.ensureDirectoryExists(appDir);
    this.ensureDirectoryExists(routersDir);
    this.ensureDirectoryExists(modelsDir);
    this.ensureDirectoryExists(servicesDir);

    // Generate files based on endpoint type
    const results = {};

    // Create or update main.py
    results.main_file = await this.createMainApp(appDir);

    // Create config.py
    results.config_file = await this.createConfig(appDir);

    // Create requirements.txt
    results.requirements_file = await this.createRequirements(backendDir);

    // Create router based on endpoint type
    const routerFileName = this.getRouterFileName(endpoint_type);
    results.router_file = await this.createRouter(
      routersDir,
      routerFileName,
      endpoint_type,
      route_path,
      request_model,
      response_model,
      http_method,
      auth_required
    );

    // Create or update models
    results.models_file = await this.createModels(modelsDir, request_model, response_model, endpoint_type);

    // Create service file if needed
    if (endpoint_type === 'rag_query') {
      await this.createRAGService(servicesDir);
    } else if (endpoint_type === 'user_auth') {
      await this.createAuthService(servicesDir);
    } else if (endpoint_type === 'personalization') {
      await this.createPersonalizationService(servicesDir);
    }

    return results;
  }

  ensureDirectoryExists(dirPath) {
    if (!fs.existsSync(dirPath)) {
      fs.mkdirSync(dirPath, { recursive: true });
    }
  }

  getRouterFileName(endpoint_type) {
    const mapping = {
      'rag_query': 'chat.py',
      'user_auth': 'auth.py',
      'personalization': 'personalize.py'
    };
    return mapping[endpoint_type] || 'custom.py';
  }

  async createMainApp(appDir) {
    const mainPath = path.join(appDir, 'main.py');

    let content = `from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware import Middleware
from fastapi.responses import JSONResponse
import logging
import sys
from datetime import datetime

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('app.log'),
        logging.StreamHandler(sys.stdout)
    ]
)

logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot Backend",
    description="FastAPI backend for RAG chatbot with user management and personalization",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:8080"],  # Docusaurus frontend
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request logging middleware
@app.middleware("http")
async def log_requests(request, call_next):
    start_time = datetime.now()
    logger.info(f"Request started: {request.method} {request.url}")

    response = await call_next(request)

    process_time = datetime.now() - start_time
    logger.info(f"Request completed: {response.status_code} in {process_time.total_seconds():.4f}s")

    return response

# Health check endpoint
@app.get("/health")
async def health_check():
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "version": "1.0.0"
    }

# Import routers
`;

    // Check if routers exist and import them
    const routersDir = path.join(appDir, 'routers');
    if (fs.existsSync(routersDir)) {
      const routerFiles = fs.readdirSync(routersDir).filter(f => f.endsWith('.py'));
      for (const file of routerFiles) {
        const routerName = file.replace('.py', '');
        content += `from app.routers import ${routerName}\n`;
      }
    }

    content += `
# Include routers
`;

    // Add router includes
    if (fs.existsSync(routersDir)) {
      const routerFiles = fs.readdirSync(routersDir).filter(f => f.endsWith('.py'));
      for (const file of routerFiles) {
        const routerName = file.replace('.py', '');
        content += `app.include_router(${routerName}.router)\n`;
      }
    }

    content += `

@app.exception_handler(Exception)
async def global_exception_handler(request, exc):
    logger.error(f"Global exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={"detail": "Internal server error"}
    )

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
`;

    if (!fs.existsSync(mainPath)) {
      fs.writeFileSync(mainPath, content);
    } else {
      // Update existing main.py to include new router
      let existingContent = fs.readFileSync(mainPath, 'utf8');
      if (!existingContent.includes(`from app.routers import ${path.basename(mainPath).replace('.py', '')}`)) {
        fs.writeFileSync(mainPath, content);
      }
    }

    return mainPath;
  }

  async createConfig(appDir) {
    const configPath = path.join(appDir, 'config.py');
    const content = `from pydantic_settings import BaseSettings
from typing import Optional
import os

class Settings(BaseSettings):
    # API Configuration
    API_V1_STR: str = "/api/v1"
    PROJECT_NAME: str = "RAG Chatbot Backend"

    # Security
    SECRET_KEY: str = os.getenv("SECRET_KEY", "your-secret-key-change-in-production")
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30

    # Database (if needed)
    DATABASE_URL: Optional[str] = os.getenv("DATABASE_URL")

    # CORS
    BACKEND_CORS_ORIGINS: list = ["http://localhost:3000", "http://localhost:8080"]

    # RAG Configuration
    VECTOR_STORE_URL: Optional[str] = os.getenv("VECTOR_STORE_URL")
    EMBEDDING_MODEL: str = "text-embedding-ada-002"

    # Logging
    LOG_LEVEL: str = "INFO"

    class Config:
        env_file = ".env"

settings = Settings()
`;

    fs.writeFileSync(configPath, content);
    return configPath;
  }

  async createRequirements(backendDir) {
    const requirementsPath = path.join(backendDir, 'requirements.txt');
    const content = `fastapi>=0.104.0
uvicorn[standard]>=0.24.0
pydantic>=2.0.0
pydantic-settings>=2.0.0
python-jose[cryptography]>=3.3.0
passlib[bcrypt]>=1.7.4
python-multipart>=0.0.6
python-dotenv>=1.0.0
httpx>=0.25.0
openai>=1.0.0
langchain>=0.0.300
langchain-openai>=0.0.2
qdrant-client>=1.6.0
sentence-transformers>=2.2.0
transformers>=4.35.0
torch>=2.0.0
numpy>=1.24.0
pandas>=2.0.0
scikit-learn>=1.3.0
`;

    fs.writeFileSync(requirementsPath, content);
    return requirementsPath;
  }

  async createRouter(routersDir, fileName, endpointType, routePath, requestModel, responseModel, httpMethod, authRequired) {
    const routerPath = path.join(routersDir, fileName);

    let content = `from fastapi import APIRouter, Depends, HTTPException, status
from typing import Optional, List, Dict, Any
import logging
from datetime import datetime

logger = logging.getLogger(__name__)
router = APIRouter()

`;

    if (authRequired) {
      content += `# JWT Authentication dependencies
from jose import JWTError, jwt
from app.config import settings

async def get_current_user(token: str):
    try:
        payload = jwt.decode(token, settings.SECRET_KEY, algorithms=[settings.ALGORITHM])
        user_id: str = payload.get("sub")
        if user_id is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )
        return user_id
    except JWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

`;
    }

    // Add models import
    content += `from app.models import `;

    if (endpointType === 'rag_query') {
      content += `QueryRequest, QueryResponse\n\n`;
    } else if (endpointType === 'user_auth') {
      content += `LoginRequest, TokenResponse, UserCreate\n\n`;
    } else if (endpointType === 'personalization') {
      content += `PreferencesUpdate, UserPreferences\n\n`;
    }

    // Add the endpoint
    const cleanRoutePath = routePath.startsWith('/') ? routePath : '/' + routePath;

    content += `@router.${httpMethod.toLowerCase()}("${cleanRoutePath}")\n`;

    if (authRequired) {
      content += `async def ${this.getFunctionName(endpointType, routePath)}`;
      if (endpointType === 'user_auth') {
        // For auth endpoints, don't require authentication
        content += `(request: `;
      } else {
        content += `(request: `;
      }

      if (requestModel) {
        content += `${requestModel.split('(')[0]}`;
      } else {
        content += 'dict';
      }

      if (endpointType !== 'user_auth') {
        content += `, current_user: str = Depends(get_current_user)`;
      }

      content += `):\n`;
    } else {
      content += `async def ${this.getFunctionName(endpointType, routePath)}(request: `;
      if (requestModel) {
        content += `${requestModel.split('(')[0]}`;
      } else {
        content += 'dict';
      }
      content += `):\n`;
    }

    content += `    """
    ${this.getEndpointDescription(endpointType)}
    """
    try:
        logger.info(f"Processing ${endpointType} request")

        # TODO: Implement business logic here
        result = await process_${endpointType.replace('_', '')}(request)

        logger.info(f"Successfully processed ${endpointType} request")
        return result

    except Exception as e:
        logger.error(f"Error processing ${endpointType} request: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to process ${endpointType} request"
        )

`;

    // Add placeholder function
    content += `async def process_${endpointType.replace('_', '')}(request):
    """Process ${endpointType} request - implement your business logic here"""

    if (endpointType === 'rag_query'):
        return {
            "answer": f"Processed query: {getattr(request, 'question', 'No question provided')}",
            "sources": ["source1", "source2"],
            "confidence": 0.85
        }
    elif (endpointType === 'user_auth':
        return {
            "access_token": "sample_jwt_token",
            "token_type": "bearer"
        }
    elif (endpointType === 'personalization':
        return {
            "user_id": "sample_user_id",
            "preferences": {
                "theme": "light",
                "language": "en",
                "notifications": True
            }
        }

    return {"status": "processed", "data": request}
`;

    fs.writeFileSync(routerPath, content);
    return routerPath;
  }

  getFunctionName(endpointType, routePath) {
    const cleanPath = routePath.replace(/[^a-zA-Z0-9]/g, '_').replace(/^_+|_+$/g, '');
    return endpointType.replace('_', '_') + '_' + (cleanPath || 'endpoint');
  }

  getEndpointDescription(endpointType) {
    const descriptions = {
      'rag_query': 'Process RAG query and return answer with sources',
      'user_auth': 'Authenticate user and return access token',
      'personalization': 'Update user personalization preferences'
    };
    return descriptions[endpointType] || 'Process endpoint request';
  }

  async createModels(modelsDir, requestModel, responseModel, endpointType) {
    const modelsPath = path.join(modelsDir, '__init__.py');

    let content = `from pydantic import BaseModel, Field, EmailStr
from typing import Optional, List, Dict, Any
from datetime import datetime
from enum import Enum

`;

    // Add common models
    content += `class BaseResponse(BaseModel):
    success: bool = True
    message: Optional[str] = None
    timestamp: datetime = Field(default_factory=datetime.now)

class ErrorResponse(BaseModel):
    success: bool = False
    error: str
    detail: Optional[str] = None
    timestamp: datetime = Field(default_factory=datetime.now)

`;

    // Add specific models based on endpoint type
    if (endpointType === 'rag_query') {
      content += `class QueryRequest(BaseModel):
    question: str = Field(..., description="The question to answer")
    context: Optional[str] = Field(None, description="Additional context for the query")
    max_tokens: Optional[int] = Field(500, description="Maximum tokens in response")
    temperature: Optional[float] = Field(0.7, description="Response temperature")

class QueryResponse(BaseModel):
    answer: str = Field(..., description="The answer to the question")
    sources: List[str] = Field(default_factory=list, description="Source documents used")
    confidence: float = Field(..., description="Confidence score of the answer")
    query_time: Optional[float] = Field(None, description="Time taken to process query")

class Document(BaseModel):
    content: str
    metadata: Dict[str, Any]
    score: float

`;
    } else if (endpointType === 'user_auth') {
      content += `class UserBase(BaseModel):
    email: EmailStr
    username: Optional[str] = None
    full_name: Optional[str] = None

class UserCreate(UserBase):
    password: str = Field(..., min_length=8)

class UserLogin(BaseModel):
    email: EmailStr
    password: str

class LoginRequest(BaseModel):
    email: EmailStr = Field(..., description="User email")
    password: str = Field(..., description="User password")

class TokenResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"
    expires_in: Optional[int] = None

class User(UserBase):
    id: str
    is_active: bool = True
    created_at: datetime

    class Config:
        from_attributes = True

`;
    } else if (endpointType === 'personalization') {
      content += `class ThemeEnum(str, Enum):
    LIGHT = "light"
    DARK = "dark"
    AUTO = "auto"

class LanguageEnum(str, Enum):
    EN = "en"
    ES = "es"
    FR = "fr"
    DE = "de"
    ZH = "zh"

class PreferencesBase(BaseModel):
    theme: ThemeEnum = ThemeEnum.LIGHT
    language: LanguageEnum = LanguageEnum.EN
    notifications: bool = True
    timezone: str = "UTC"

class PreferencesUpdate(PreferencesBase):
    pass

class UserPreferences(BaseModel):
    user_id: str
    preferences: PreferencesBase
    updated_at: datetime = Field(default_factory=datetime.now)

class UserPreferencesResponse(BaseModel):
    user_id: str
    theme: ThemeEnum
    language: LanguageEnum
    notifications: bool
    timezone: str
    updated_at: datetime

`;
    }

    fs.writeFileSync(modelsPath, content);
    return modelsPath;
  }

  async createRAGService(servicesDir) {
    const servicePath = path.join(servicesDir, 'rag_service.py');
    const content = `from typing import List, Dict, Any, Optional
import logging
from datetime import datetime

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        # Initialize your RAG components here
        # This could include vector store, embedding model, LLM, etc.
        pass

    async def query(self, question: str, context: Optional[str] = None, **kwargs) -> Dict[str, Any]:
        """
        Process a RAG query and return results
        """
        try:
            start_time = datetime.now()

            # TODO: Implement your RAG logic here
            # 1. Retrieve relevant documents
            # 2. Generate context
            # 3. Generate answer using LLM

            # Placeholder implementation
            answer = f"This is a placeholder answer for: {question}"
            sources = ["doc1", "doc2", "doc3"]
            confidence = 0.85

            process_time = (datetime.now() - start_time).total_seconds()

            return {
                "answer": answer,
                "sources": sources,
                "confidence": confidence,
                "query_time": process_time
            }

        except Exception as e:
            logger.error(f"Error in RAG query: {str(e)}")
            raise

    async def add_documents(self, documents: List[Dict[str, Any]]) -> bool:
        """
        Add documents to the vector store
        """
        try:
            # TODO: Implement document indexing
            logger.info(f"Adding {len(documents)} documents to vector store")
            return True
        except Exception as e:
            logger.error(f"Error adding documents: {str(e)}")
            return False

# Global service instance
rag_service = RAGService()
`;

    fs.writeFileSync(servicePath, content);
    return servicePath;
  }

  async createAuthService(servicesDir) {
    const servicePath = path.join(servicesDir, 'auth_service.py');
    const content = `from typing import Optional
from datetime import datetime, timedelta
import logging
from passlib.context import CryptContext
from jose import JWTError, jwt
from app.config import settings

logger = logging.getLogger(__name__)

# Password hashing
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

class AuthService:
    def __init__(self):
        self.secret_key = settings.SECRET_KEY
        self.algorithm = settings.ALGORITHM
        self.access_token_expire_minutes = settings.ACCESS_TOKEN_EXPIRE_MINUTES

    def verify_password(self, plain_password: str, hashed_password: str) -> bool:
        """Verify a password against its hash"""
        return pwd_context.verify(plain_password, hashed_password)

    def get_password_hash(self, password: str) -> str:
        """Hash a password"""
        return pwd_context.hash(password)

    def create_access_token(self, data: dict, expires_delta: Optional[timedelta] = None) -> str:
        """Create a JWT access token"""
        to_encode = data.copy()
        if expires_delta:
            expire = datetime.utcnow() + expires_delta
        else:
            expire = datetime.utcnow() + timedelta(minutes=self.access_token_expire_minutes)

        to_encode.update({"exp": expire})
        encoded_jwt = jwt.encode(to_encode, self.secret_key, algorithm=self.algorithm)
        return encoded_jwt

    def verify_token(self, token: str) -> Optional[str]:
        """Verify and decode a JWT token"""
        try:
            payload = jwt.decode(token, self.secret_key, algorithms=[self.algorithm])
            user_id: str = payload.get("sub")
            if user_id is None:
                return None
            return user_id
        except JWTError:
            return None

    async def authenticate_user(self, email: str, password: str) -> Optional[dict]:
        """
        Authenticate a user with email and password
        TODO: Implement actual user authentication against your database
        """
        try:
            # Placeholder implementation
            # In a real app, you would:
            # 1. Query user from database by email
            # 2. Verify password using verify_password
            # 3. Return user data if authenticated

            # For demo purposes, accept any email/password
            if email and password:
                return {
                    "id": "user_123",
                    "email": email,
                    "is_active": True
                }
            return None

        except Exception as e:
            logger.error(f"Authentication error: {str(e)}")
            return None

    async def create_user(self, email: str, password: str, **kwargs) -> Optional[dict]:
        """
        Create a new user
        TODO: Implement actual user creation in your database
        """
        try:
            # Placeholder implementation
            # In a real app, you would:
            # 1. Check if user already exists
            # 2. Hash the password using get_password_hash
            # 3. Store user in database

            hashed_password = self.get_password_hash(password)

            return {
                "id": "new_user_123",
                "email": email,
                "hashed_password": hashed_password,
                "is_active": True,
                "created_at": datetime.utcnow()
            }

        except Exception as e:
            logger.error(f"User creation error: {str(e)}")
            return None

# Global service instance
auth_service = AuthService()
`;

    fs.writeFileSync(servicePath, content);
    return servicePath;
  }

  async createPersonalizationService(servicesDir) {
    const servicePath = path.join(servicesDir, 'personalization_service.py');
    const content = `from typing import Dict, Any, Optional
import logging
from datetime import datetime
from app.models import PreferencesBase

logger = logging.getLogger(__name__)

class PersonalizationService:
    def __init__(self):
        # TODO: Initialize database connection or other storage
        self.user_preferences = {}  # Placeholder storage

    async def get_user_preferences(self, user_id: str) -> Optional[PreferencesBase]:
        """
        Get user preferences
        TODO: Implement actual database query
        """
        try:
            # Placeholder implementation
            if user_id in self.user_preferences:
                return self.user_preferences[user_id]

            # Return default preferences
            default_prefs = PreferencesBase()
            self.user_preferences[user_id] = default_prefs
            return default_prefs

        except Exception as e:
            logger.error(f"Error getting preferences for user {user_id}: {str(e)}")
            return None

    async def update_user_preferences(self, user_id: str, preferences: PreferencesBase) -> bool:
        """
        Update user preferences
        TODO: Implement actual database update
        """
        try:
            # Placeholder implementation
            self.user_preferences[user_id] = preferences
            logger.info(f"Updated preferences for user {user_id}")
            return True

        except Exception as e:
            logger.error(f"Error updating preferences for user {user_id}: {str(e)}")
            return False

    async def reset_user_preferences(self, user_id: str) -> bool:
        """
        Reset user preferences to defaults
        """
        try:
            default_prefs = PreferencesBase()
            return await self.update_user_preferences(user_id, default_prefs)
        except Exception as e:
            logger.error(f"Error resetting preferences for user {user_id}: {str(e)}")
            return False

    async def get_user_stats(self, user_id: str) -> Dict[str, Any]:
        """
        Get user statistics for personalization
        TODO: Implement actual analytics
        """
        try:
            # Placeholder implementation
            return {
                "user_id": user_id,
                "queries_count": 0,
                "last_active": datetime.utcnow(),
                "preferred_topics": [],
                "session_duration": 0
            }
        except Exception as e:
            logger.error(f"Error getting stats for user {user_id}: {str(e)}")
            return {}

# Global service instance
personalization_service = PersonalizationService()
`;

    fs.writeFileSync(servicePath, content);
    return servicePath;
  }
}

module.exports = FastAPIBackend;