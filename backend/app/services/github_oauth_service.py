"""
GitHub OAuth service for token exchange and user info extraction.
Implements OAuth 2.0 authorization code flow.
"""

import logging
import secrets
from typing import Optional, TypedDict
from urllib.parse import urlencode

import httpx

from app.config import settings

logger = logging.getLogger(__name__)

# GitHub OAuth endpoints
GITHUB_AUTH_URL = "https://github.com/login/oauth/authorize"
GITHUB_TOKEN_URL = "https://github.com/login/oauth/access_token"
GITHUB_USER_URL = "https://api.github.com/user"

# OAuth scopes required for authentication
GITHUB_SCOPES = ["read:user", "user:email"]


class GitHubUserInfo(TypedDict):
    """GitHub user info response structure."""

    id: int  # GitHub user ID (unique identifier)
    login: str  # GitHub username
    email: Optional[str]
    name: Optional[str]
    avatar_url: Optional[str]


class GitHubOAuthService:
    """Service for GitHub OAuth 2.0 operations."""

    def __init__(self):
        """Initialize GitHub OAuth service."""
        self._client_id = settings.github_client_id
        self._client_secret = settings.github_client_secret
        self._redirect_uri = settings.github_redirect_uri

    @property
    def is_configured(self) -> bool:
        """Check if GitHub OAuth is configured."""
        return settings.github_oauth_configured

    def generate_state_token(self) -> str:
        """
        Generate a CSRF state token for OAuth flow.

        Returns:
            URL-safe random token
        """
        return secrets.token_urlsafe(32)

    def get_authorization_url(self, state: str, redirect_after: Optional[str] = None) -> str:
        """
        Build the GitHub OAuth authorization URL.

        Args:
            state: CSRF state token
            redirect_after: Optional URL to redirect to after OAuth completion

        Returns:
            Full authorization URL for GitHub OAuth
        """
        if not self.is_configured:
            raise ValueError("GitHub OAuth is not configured")

        # Encode the final destination in the state if provided
        if redirect_after:
            state = f"{state}|{redirect_after}"

        params = {
            "client_id": self._client_id,
            "redirect_uri": self._redirect_uri,
            "scope": " ".join(GITHUB_SCOPES),
            "state": state,
            "allow_signup": "true",
        }

        url = f"{GITHUB_AUTH_URL}?{urlencode(params)}"
        logger.debug(f"Generated GitHub OAuth URL: {url[:100]}...")
        return url

    async def exchange_code_for_token(self, code: str) -> str:
        """
        Exchange authorization code for access token.

        Args:
            code: Authorization code from GitHub callback

        Returns:
            Access token string

        Raises:
            httpx.HTTPError: If token exchange fails
        """
        if not self.is_configured:
            raise ValueError("GitHub OAuth is not configured")

        data = {
            "client_id": self._client_id,
            "client_secret": self._client_secret,
            "code": code,
            "redirect_uri": self._redirect_uri,
        }

        async with httpx.AsyncClient() as client:
            response = await client.post(
                GITHUB_TOKEN_URL,
                data=data,
                headers={"Accept": "application/json"},
            )

            if response.status_code != 200:
                error_data = response.json()
                logger.error(f"Token exchange failed: {error_data}")
                raise ValueError(
                    f"Token exchange failed: {error_data.get('error_description', error_data.get('error', 'Unknown error'))}"
                )

            token_data = response.json()
            access_token = token_data.get("access_token")

            if not access_token:
                raise ValueError("No access token in response")

            return access_token

    async def get_user_info(self, access_token: str) -> GitHubUserInfo:
        """
        Fetch user info from GitHub using access token.

        Args:
            access_token: GitHub OAuth access token

        Returns:
            GitHubUserInfo with user's GitHub profile data

        Raises:
            httpx.HTTPError: If userinfo request fails
        """
        async with httpx.AsyncClient() as client:
            # Get user info
            response = await client.get(
                GITHUB_USER_URL,
                headers={
                    "Authorization": f"Bearer {access_token}",
                    "Accept": "application/vnd.github+json",
                    "X-GitHub-Api-Version": "2022-11-28",
                },
            )

            if response.status_code != 200:
                logger.error(f"Failed to get user info: {response.text}")
                raise ValueError("Failed to get user info from GitHub")

            data = response.json()

            # If email is not public, fetch it from emails endpoint
            email = data.get("email")
            if not email:
                email_response = await client.get(
                    "https://api.github.com/user/emails",
                    headers={
                        "Authorization": f"Bearer {access_token}",
                        "Accept": "application/vnd.github+json",
                        "X-GitHub-Api-Version": "2022-11-28",
                    },
                )

                if email_response.status_code == 200:
                    emails = email_response.json()
                    # Get primary verified email
                    for email_data in emails:
                        if email_data.get("primary") and email_data.get("verified"):
                            email = email_data.get("email")
                            break

            if not email:
                raise ValueError("GitHub account does not have a verified email")

            logger.debug(f"Got GitHub user info for: {email}")

            return GitHubUserInfo(
                id=data["id"],
                login=data["login"],
                email=email,
                name=data.get("name"),
                avatar_url=data.get("avatar_url"),
            )

    async def authenticate(self, code: str) -> GitHubUserInfo:
        """
        Complete OAuth authentication: exchange code and get user info.

        Args:
            code: Authorization code from GitHub callback

        Returns:
            GitHubUserInfo with user's GitHub profile data

        Raises:
            ValueError: If authentication fails
        """
        # Exchange code for access token
        access_token = await self.exchange_code_for_token(code)

        # Get user info
        user_info = await self.get_user_info(access_token)

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


# Global GitHub OAuth service instance
github_oauth_service = GitHubOAuthService()
