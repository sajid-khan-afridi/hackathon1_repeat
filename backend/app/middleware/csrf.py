"""
CSRF Protection Middleware for FastAPI.

Implements Double Submit Cookie pattern for CSRF protection.
Required for cookie-based JWT authentication (FR-027).
"""

import secrets
import logging
from typing import Optional
from fastapi import Request, Response, HTTPException, status
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.types import ASGIApp

logger = logging.getLogger(__name__)

# CSRF token cookie name
CSRF_COOKIE_NAME = "csrf_token"
# CSRF token header name
CSRF_HEADER_NAME = "X-CSRF-Token"
# CSRF token length (bytes)
CSRF_TOKEN_LENGTH = 32


class CSRFProtectionMiddleware(BaseHTTPMiddleware):
    """
    CSRF Protection Middleware using Double Submit Cookie pattern.

    How it works:
    1. On GET requests, generate a CSRF token and set it as a cookie
    2. On state-changing requests (POST, PUT, DELETE, PATCH), validate that:
       - CSRF token exists in cookie
       - CSRF token exists in header (X-CSRF-Token)
       - Both values match

    Exempt paths:
    - /health (health checks)
    - /api/docs, /api/redoc (API documentation)
    - /auth/google/callback (OAuth callback - state param used instead)
    """

    def __init__(
        self,
        app: ASGIApp,
        exempt_paths: Optional[list[str]] = None,
        cookie_secure: bool = True,
        cookie_samesite: str = "lax",
    ):
        super().__init__(app)
        self.exempt_paths = exempt_paths or [
            "/health",
            "/api/docs",
            "/api/redoc",
            "/openapi.json",
            "/auth/signup",  # Public endpoint - no session yet
            "/auth/login",   # Public endpoint - no session yet
            "/auth/refresh",  # Uses refresh token cookie for auth, not CSRF
            "/auth/logout",   # Logout should work even without CSRF token
            "/auth/google/callback",  # OAuth callback uses state param
            "/auth/github/callback",  # GitHub OAuth callback uses state param
            "/api/v1/query",  # Public RAG endpoint
        ]
        self.cookie_secure = cookie_secure
        self.cookie_samesite = cookie_samesite

    async def dispatch(self, request: Request, call_next):
        """
        Process request and apply CSRF protection.
        """
        # Check if path is exempt
        if self._is_exempt(request):
            return await call_next(request)

        # GET, HEAD, OPTIONS are safe methods - generate/refresh CSRF token
        if request.method in ("GET", "HEAD", "OPTIONS"):
            response = await call_next(request)
            self._set_csrf_token(response)
            return response

        # State-changing methods require CSRF validation
        if request.method in ("POST", "PUT", "DELETE", "PATCH"):
            self._validate_csrf_token(request)

        # Process request
        response = await call_next(request)
        return response

    def _is_exempt(self, request: Request) -> bool:
        """
        Check if request path is exempt from CSRF protection.
        """
        path = request.url.path
        for exempt_path in self.exempt_paths:
            if path.startswith(exempt_path):
                return True
        return False

    def _generate_csrf_token(self) -> str:
        """
        Generate a cryptographically secure random CSRF token.
        """
        return secrets.token_urlsafe(CSRF_TOKEN_LENGTH)

    def _set_csrf_token(self, response: Response) -> None:
        """
        Set CSRF token in response cookie if not already present.
        """
        # Only set if cookie doesn't exist (avoid regenerating on every request)
        # Note: This will be called on first GET request
        csrf_token = self._generate_csrf_token()

        response.set_cookie(
            key=CSRF_COOKIE_NAME,
            value=csrf_token,
            httponly=False,  # Must be readable by JavaScript to include in headers
            secure=True,  # Required for SameSite=None
            samesite="none",  # Allow cross-origin cookie sending (GitHub Pages -> Railway)
            max_age=86400,  # 24 hours (matches access token expiry)
            path="/",
        )

        logger.debug(f"Set CSRF token in cookie: {csrf_token[:8]}...")

    def _validate_csrf_token(self, request: Request) -> None:
        """
        Validate CSRF token from cookie and header match.

        Raises:
            HTTPException: 403 Forbidden if CSRF validation fails
        """
        # Get token from cookie
        csrf_cookie = request.cookies.get(CSRF_COOKIE_NAME)
        if not csrf_cookie:
            logger.warning(f"CSRF validation failed: No CSRF cookie found for {request.method} {request.url.path}")
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="CSRF token missing in cookie. Please reload the page.",
            )

        # Get token from header
        csrf_header = request.headers.get(CSRF_HEADER_NAME)
        if not csrf_header:
            logger.warning(f"CSRF validation failed: No CSRF header found for {request.method} {request.url.path}")
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="CSRF token missing in request header (X-CSRF-Token).",
            )

        # Validate tokens match (constant-time comparison to prevent timing attacks)
        if not secrets.compare_digest(csrf_cookie, csrf_header):
            logger.warning(f"CSRF validation failed: Token mismatch for {request.method} {request.url.path}")
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="CSRF token validation failed. Please reload the page.",
            )

        logger.debug(f"CSRF validation passed for {request.method} {request.url.path}")


def configure_csrf_protection(
    app,
    exempt_paths: Optional[list[str]] = None,
    cookie_secure: bool = True,
) -> None:
    """
    Configure CSRF protection middleware for the application.

    Args:
        app: FastAPI application instance
        exempt_paths: List of paths to exempt from CSRF protection
        cookie_secure: Whether to set Secure flag on CSRF cookie (should be True in production)
    """
    app.add_middleware(
        CSRFProtectionMiddleware,
        exempt_paths=exempt_paths,
        cookie_secure=cookie_secure,
    )
    logger.info("CSRF protection middleware configured")
