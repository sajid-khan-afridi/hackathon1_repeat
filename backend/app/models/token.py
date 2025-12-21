"""
Token models for JWT authentication.
Pydantic models for token payloads and responses.
"""

from pydantic import BaseModel
from uuid import UUID
from datetime import datetime
from typing import Literal, Optional


class TokenPayload(BaseModel):
    """JWT token payload structure."""

    sub: UUID  # user_id
    email: str
    type: Literal["access", "refresh"]
    iat: datetime
    exp: datetime


class TokenResponse(BaseModel):
    """Response model for token endpoints (for reference, tokens are in cookies)."""

    access_token: str
    refresh_token: str
    token_type: str = "bearer"
    expires_in: int  # seconds until access token expires


class RefreshTokenRequest(BaseModel):
    """Request model for token refresh (tokens read from cookies)."""

    # Refresh token is read from httpOnly cookie, not request body
    pass


class RefreshTokenInDB(BaseModel):
    """Refresh token model with database fields."""

    id: UUID
    user_id: UUID
    token_hash: str
    expires_at: datetime
    created_at: datetime
    revoked_at: Optional[datetime] = None
    user_agent: Optional[str] = None
    ip_address: Optional[str] = None

    class Config:
        from_attributes = True


class SessionInfo(BaseModel):
    """Session info for listing user sessions."""

    id: UUID
    created_at: datetime
    expires_at: datetime
    user_agent: Optional[str] = None
    ip_address: Optional[str] = None
    is_current: bool = False

    class Config:
        from_attributes = True
