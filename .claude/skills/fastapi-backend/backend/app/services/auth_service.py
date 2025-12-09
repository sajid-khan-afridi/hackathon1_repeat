import logging
from datetime import datetime, timedelta
from typing import Optional, Dict, Any
import secrets
import hashlib
from jose import JWTError, jwt

logger = logging.getLogger(__name__)


class AuthService:
    """
    Service for handling authentication and authorization
    """

    def __init__(self):
        # TODO: Load these from settings
        self.secret_key = "your-secret-key-change-in-production"
        self.algorithm = "HS256"
        self.access_token_expire_minutes = 30
        self.refresh_token_expire_days = 7

    async def create_access_token(self, data: Dict, expires_delta: Optional[timedelta] = None) -> str:
        """
        Create a JWT access token
        """
        to_encode = data.copy()

        if expires_delta:
            expire = datetime.utcnow() + expires_delta
        else:
            expire = datetime.utcnow() + timedelta(minutes=self.access_token_expire_minutes)

        to_encode.update({"exp": expire, "type": "access"})
        encoded_jwt = jwt.encode(to_encode, self.secret_key, algorithm=self.algorithm)

        return encoded_jwt

    async def create_refresh_token(self, data: Dict) -> str:
        """
        Create a JWT refresh token
        """
        to_encode = data.copy()
        expire = datetime.utcnow() + timedelta(days=self.refresh_token_expire_days)
        to_encode.update({"exp": expire, "type": "refresh", "jti": secrets.token_urlsafe(32)})
        encoded_jwt = jwt.encode(to_encode, self.secret_key, algorithm=self.algorithm)

        # TODO: Store refresh token in database for revocation support
        return encoded_jwt

    async def verify_token(self, token: str) -> Dict:
        """
        Verify and decode JWT token
        """
        try:
            payload = jwt.decode(token, self.secret_key, algorithms=[self.algorithm])

            # Check token type
            token_type = payload.get("type")
            if token_type != "access":
                raise ValueError("Invalid token type")

            # Check expiration
            exp = payload.get("exp")
            if exp and datetime.fromtimestamp(exp) < datetime.utcnow():
                raise ValueError("Token has expired")

            return payload

        except JWTError as e:
            logger.error(f"JWT verification error: {str(e)}")
            raise ValueError("Invalid token")

    async def verify_refresh_token(self, token: str) -> Dict:
        """
        Verify and decode refresh token
        """
        try:
            payload = jwt.decode(token, self.secret_key, algorithms=[self.algorithm])

            # Check token type
            token_type = payload.get("type")
            if token_type != "refresh":
                raise ValueError("Invalid token type")

            # TODO: Check if refresh token is revoked/not in database
            # This would involve database lookup

            return payload

        except JWTError as e:
            logger.error(f"Refresh token verification error: {str(e)}")
            raise ValueError("Invalid refresh token")

    async def authenticate(self, email: str, password: str) -> Optional[Dict]:
        """
        Authenticate user with email and password
        """
        try:
            # TODO: Implement actual authentication
            # This would typically:
            # 1. Find user by email in database
            # 2. Verify password hash
            # 3. Return user data if valid

            # Mock implementation
            if email == "dev@example.com" and password == "password123":
                return {
                    "user_id": "dev_user_123",
                    "email": email,
                    "username": "dev_user",
                    "full_name": "Development User",
                    "is_active": True,
                    "created_at": datetime.utcnow(),
                    "last_login": None
                }

            return None

        except Exception as e:
            logger.error(f"Authentication error: {str(e)}")
            return None

    async def hash_password(self, password: str) -> str:
        """
        Hash password using bcrypt
        """
        # TODO: Implement actual password hashing
        # from passlib.context import CryptContext
        # pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
        # return pwd_context.hash(password)

        # Mock implementation (INSECURE - replace with real hashing)
        return hashlib.sha256(password.encode()).hexdigest()

    async def verify_password(self, plain_password: str, hashed_password: str) -> bool:
        """
        Verify password against hash
        """
        # TODO: Implement actual password verification
        # from passlib.context import CryptContext
        # pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
        # return pwd_context.verify(plain_password, hashed_password)

        # Mock implementation (INSECURE - replace with real verification)
        return hashlib.sha256(plain_password.encode()).hexdigest() == hashed_password

    async def generate_password_reset_token(self, email: str) -> str:
        """
        Generate password reset token
        """
        data = {
            "sub": email,
            "type": "password_reset",
            "exp": datetime.utcnow() + timedelta(hours=1)  # Token expires in 1 hour
        }

        token = jwt.encode(data, self.secret_key, algorithm=self.algorithm)

        # TODO: Store reset token in database
        logger.info(f"Password reset token generated for: {email}")

        return token

    async def verify_password_reset_token(self, token: str) -> Dict:
        """
        Verify password reset token
        """
        try:
            payload = jwt.decode(token, self.secret_key, algorithms=[self.algorithm])

            # Check token type
            token_type = payload.get("type")
            if token_type != "password_reset":
                raise ValueError("Invalid token type")

            # Check expiration
            exp = payload.get("exp")
            if exp and datetime.fromtimestamp(exp) < datetime.utcnow():
                raise ValueError("Token has expired")

            # TODO: Check if token exists in database and is not used
            # This would involve database lookup

            return payload

        except JWTError as e:
            logger.error(f"Password reset token verification error: {str(e)}")
            raise ValueError("Invalid password reset token")