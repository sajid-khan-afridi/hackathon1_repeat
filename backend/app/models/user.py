"""
User models for authentication.
Pydantic models for user data validation and serialization.
"""

from pydantic import BaseModel, EmailStr, Field, field_validator
from uuid import UUID
from datetime import datetime
from typing import Optional
import re


class UserBase(BaseModel):
    """Base user model with email."""

    email: EmailStr

    @field_validator("email", mode="before")
    @classmethod
    def sanitize_email(cls, v: str) -> str:
        """
        Sanitize email input (FR-026):
        - Strip whitespace
        - Convert to lowercase
        """
        if isinstance(v, str):
            return v.strip().lower()
        return v


class UserCreate(UserBase):
    """Model for user registration (signup)."""

    password: str = Field(..., min_length=8)

    @field_validator("password")
    @classmethod
    def validate_password(cls, v: str) -> str:
        """
        Validate and sanitize password (FR-003, FR-026):
        - At least 8 characters
        - At least 1 uppercase letter
        - At least 1 number
        - No null bytes or control characters
        """
        # Sanitize: check for null bytes and control characters (FR-026)
        if "\x00" in v:
            raise ValueError("Invalid characters in password")
        if any(ord(char) < 32 for char in v if char not in "\n\r\t"):
            raise ValueError("Invalid control characters in password")

        # Validate requirements (FR-003)
        if len(v) < 8:
            raise ValueError("Password must be at least 8 characters")
        if not re.search(r"[A-Z]", v):
            raise ValueError("Password must contain at least one uppercase letter")
        if not re.search(r"[0-9]", v):
            raise ValueError("Password must contain at least one number")
        return v


class UserLogin(UserBase):
    """Model for user login."""

    password: str


class UserResponse(UserBase):
    """User response model for API responses."""

    id: UUID
    created_at: datetime
    is_active: bool
    has_google: bool = Field(default=False, description="Whether Google OAuth is linked")
    has_github: bool = Field(default=False, description="Whether GitHub OAuth is linked")
    profile_complete: bool = Field(
        default=False, description="Whether profile wizard is complete"
    )

    class Config:
        from_attributes = True


class UserInDB(UserBase):
    """User model with all database fields."""

    id: UUID
    password_hash: Optional[str] = None
    google_id: Optional[str] = None
    github_id: Optional[str] = None
    created_at: datetime
    updated_at: datetime
    last_login_at: Optional[datetime] = None
    is_active: bool = True
    failed_login_attempts: int = 0
    locked_until: Optional[datetime] = None

    class Config:
        from_attributes = True
