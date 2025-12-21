"""
Google OAuth service for token exchange and user info extraction.
Implements OAuth 2.0 authorization code flow per research.md.
"""

import hashlib
import logging
import secrets
from typing import Optional, TypedDict
from urllib.parse import urlencode

import httpx

from app.config import settings

logger = logging.getLogger(__name__)

# Google OAuth endpoints
GOOGLE_AUTH_URL = "https://accounts.google.com/o/oauth2/v2/auth"
GOOGLE_TOKEN_URL = "https://oauth2.googleapis.com/token"
GOOGLE_USERINFO_URL = "https://www.googleapis.com/oauth2/v3/userinfo"

# OAuth scopes required for authentication
GOOGLE_SCOPES = ["openid", "email", "profile"]


class GoogleUserInfo(TypedDict):
    """Google user info response structure."""

    sub: str  # Google subject ID (unique user identifier)
    email: str
    email_verified: bool
    name: Optional[str]
    picture: Optional[str]


class OAuthService:
    """Service for Google OAuth 2.0 operations."""

    def __init__(self):
        """Initialize OAuth service."""
        self._client_id = settings.google_client_id
        self._client_secret = settings.google_client_secret
        self._redirect_uri = settings.google_redirect_uri

    @property
    def is_configured(self) -> bool:
        """Check if Google OAuth is configured."""
        return settings.google_oauth_configured

    def generate_state_token(self) -> str:
        """
        Generate a CSRF state token for OAuth flow.

        Returns:
            URL-safe random token
        """
        return secrets.token_urlsafe(32)

    def get_authorization_url(self, state: str, redirect_after: Optional[str] = None) -> str:
        """
        Build the Google OAuth authorization URL.

        Args:
            state: CSRF state token
            redirect_after: Optional URL to redirect to after OAuth completion

        Returns:
            Full authorization URL for Google OAuth
        """
        if not self.is_configured:
            raise ValueError("Google OAuth is not configured")

        # Build redirect URI with optional final destination
        redirect_uri = self._redirect_uri
        if redirect_after:
            # Encode the final destination in the state
            state = f"{state}|{redirect_after}"

        params = {
            "client_id": self._client_id,
            "redirect_uri": redirect_uri,
            "response_type": "code",
            "scope": " ".join(GOOGLE_SCOPES),
            "state": state,
            "access_type": "offline",  # Request refresh token
            "prompt": "consent",  # Force consent screen to get refresh token
        }

        url = f"{GOOGLE_AUTH_URL}?{urlencode(params)}"
        logger.debug(f"Generated Google OAuth URL: {url[:100]}...")
        return url

    async def exchange_code_for_tokens(self, code: str) -> dict:
        """
        Exchange authorization code for access and refresh tokens.

        Args:
            code: Authorization code from Google callback

        Returns:
            Token response containing access_token, id_token, etc.

        Raises:
            httpx.HTTPError: If token exchange fails
        """
        if not self.is_configured:
            raise ValueError("Google OAuth is not configured")

        data = {
            "client_id": self._client_id,
            "client_secret": self._client_secret,
            "code": code,
            "grant_type": "authorization_code",
            "redirect_uri": self._redirect_uri,
        }

        async with httpx.AsyncClient() as client:
            response = await client.post(
                GOOGLE_TOKEN_URL,
                data=data,
                headers={"Content-Type": "application/x-www-form-urlencoded"},
            )

            if response.status_code != 200:
                error_data = response.json()
                logger.error(f"Token exchange failed: {error_data}")
                raise ValueError(
                    f"Token exchange failed: {error_data.get('error_description', error_data.get('error', 'Unknown error'))}"
                )

            return response.json()

    async def get_user_info(self, access_token: str) -> GoogleUserInfo:
        """
        Fetch user info from Google using access token.

        Args:
            access_token: Google OAuth access token

        Returns:
            GoogleUserInfo with user's Google profile data

        Raises:
            httpx.HTTPError: If userinfo request fails
        """
        async with httpx.AsyncClient() as client:
            response = await client.get(
                GOOGLE_USERINFO_URL,
                headers={"Authorization": f"Bearer {access_token}"},
            )

            if response.status_code != 200:
                logger.error(f"Failed to get user info: {response.text}")
                raise ValueError("Failed to get user info from Google")

            data = response.json()
            logger.debug(f"Got Google user info for: {data.get('email')}")

            return GoogleUserInfo(
                sub=data["sub"],
                email=data["email"],
                email_verified=data.get("email_verified", False),
                name=data.get("name"),
                picture=data.get("picture"),
            )

    async def authenticate(self, code: str) -> GoogleUserInfo:
        """
        Complete OAuth authentication: exchange code and get user info.

        Args:
            code: Authorization code from Google callback

        Returns:
            GoogleUserInfo with user's Google profile data

        Raises:
            ValueError: If authentication fails
        """
        # Exchange code for tokens
        tokens = await self.exchange_code_for_tokens(code)
        access_token = tokens.get("access_token")

        if not access_token:
            raise ValueError("No access token in response")

        # Get user info
        user_info = await self.get_user_info(access_token)

        # Verify email
        if not user_info.get("email_verified", False):
            logger.warning(f"Unverified email for Google user: {user_info['email']}")
            # Still allow login but log warning

        return user_info

    def parse_state(self, state: str) -> tuple[str, Optional[str]]:
        """
        Parse state token to extract CSRF token and optional redirect URL.

        Args:
            state: State parameter from callback

        Returns:
            Tuple of (csrf_token, redirect_url or None)
        """
        if "|" in state:
            parts = state.split("|", 1)
            return parts[0], parts[1]
        return state, None


# Global OAuth service instance
oauth_service = OAuthService()
