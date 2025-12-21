"""
Password service for secure password hashing and verification.
Uses bcrypt with cost factor 12 per constitution security requirements.
"""

import logging
from passlib.context import CryptContext

logger = logging.getLogger(__name__)

# Configure bcrypt with cost factor 12
# This provides ~300ms hash time for brute force protection
# while staying under 500ms p95 latency target
pwd_context = CryptContext(
    schemes=["bcrypt"],
    deprecated="auto",
    bcrypt__rounds=12,  # Cost factor 12 per constitution
)


class PasswordService:
    """Service for password hashing and verification."""

    def hash_password(self, password: str) -> str:
        """
        Hash a password using bcrypt.

        Args:
            password: Plain text password

        Returns:
            Bcrypt hash of the password
        """
        hashed = pwd_context.hash(password)
        logger.debug("Password hashed successfully")
        return hashed

    def verify_password(self, plain_password: str, hashed_password: str) -> bool:
        """
        Verify a password against its hash.

        Args:
            plain_password: Plain text password to verify
            hashed_password: Bcrypt hash to verify against

        Returns:
            True if password matches, False otherwise
        """
        try:
            is_valid = pwd_context.verify(plain_password, hashed_password)
            if is_valid:
                logger.debug("Password verification successful")
            else:
                logger.debug("Password verification failed")
            return is_valid
        except Exception as e:
            logger.warning(f"Password verification error: {e}")
            return False

    def needs_rehash(self, hashed_password: str) -> bool:
        """
        Check if a password hash needs to be rehashed.

        This is useful when upgrading the hashing algorithm or cost factor.

        Args:
            hashed_password: Existing password hash

        Returns:
            True if rehash is recommended, False otherwise
        """
        return pwd_context.needs_update(hashed_password)


# Global password service instance
password_service = PasswordService()
