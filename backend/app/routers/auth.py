"""
Authentication router for signup, login, logout, refresh, and user info endpoints.
"""

import logging
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Request, Response, status

from app.middleware.auth import AuthenticatedUser, get_current_user
from app.models.user import UserCreate, UserLogin
from app.services.auth_service import auth_service
from app.services.jwt_service import jwt_service

logger = logging.getLogger(__name__)

router = APIRouter()


@router.get("/csrf")
async def get_csrf_token():
    """
    Get CSRF token for authenticated requests.

    The CSRF token is automatically set in the cookie by the CSRF middleware.
    Clients should read it from the cookie and include it in the X-CSRF-Token header.
    """
    return {
        "message": "CSRF token set in cookie. Include it in X-CSRF-Token header for POST/PUT/DELETE requests."
    }


def _get_client_info(request: Request) -> tuple[str, Optional[str]]:
    """Extract client IP and user agent from request."""
    # Get IP from X-Forwarded-For header (for proxies) or client host
    forwarded_for = request.headers.get("x-forwarded-for")
    if forwarded_for:
        ip_address = forwarded_for.split(",")[0].strip()
    else:
        ip_address = request.client.host if request.client else "unknown"

    user_agent = request.headers.get("user-agent")
    return ip_address, user_agent


@router.post("/signup", status_code=status.HTTP_201_CREATED)
async def signup(
    user_data: UserCreate,
    request: Request,
    response: Response,
):
    """
    Create a new user account with email and password.

    Creates an empty user profile for the profile wizard.
    Sets authentication cookies on success.

    FR-001, FR-002, FR-003, FR-004
    """
    ip_address, user_agent = _get_client_info(request)

    result = await auth_service.signup(user_data, user_agent, ip_address)

    if not result.success:
        if result.error_code == "EMAIL_EXISTS":
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail={
                    "error": {
                        "code": result.error_code,
                        "message": result.error_message,
                    }
                },
            )
        elif result.error_code == "VALIDATION_ERROR":
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail={
                    "error": {
                        "code": result.error_code,
                        "message": result.error_message,
                    }
                },
            )
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail={
                    "error": {
                        "code": "INTERNAL_ERROR",
                        "message": "An error occurred during signup",
                    }
                },
            )

    # Set auth cookies
    jwt_service.set_auth_cookies(response, result.access_token, result.refresh_token)

    return {
        "user": result.user.model_dump() if result.user else None,
        "profile": result.profile.model_dump() if result.profile else None,
    }


@router.post("/login")
async def login(
    login_data: UserLogin,
    request: Request,
    response: Response,
):
    """
    Authenticate with email and password.

    Returns JWT tokens in httpOnly cookies.
    Implements rate limiting and account lockout (FR-024).

    FR-006
    """
    ip_address, user_agent = _get_client_info(request)

    result = await auth_service.login(login_data, user_agent, ip_address)

    if not result.success:
        if result.error_code == "ACCOUNT_LOCKED":
            raise HTTPException(
                status_code=status.HTTP_423_LOCKED,
                detail={
                    "error": {
                        "code": result.error_code,
                        "message": result.error_message,
                        "locked_until": result.locked_until.isoformat() if result.locked_until else None,
                        "retry_after": int((result.locked_until - __import__("datetime").datetime.now(__import__("datetime").timezone.utc)).total_seconds()) if result.locked_until else None,
                    }
                },
            )
        elif result.error_code in ["INVALID_CREDENTIALS", "NO_PASSWORD"]:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail={
                    "error": {
                        "code": result.error_code,
                        "message": result.error_message,
                    }
                },
            )
        else:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail={
                    "error": {
                        "code": "AUTH_FAILED",
                        "message": "Authentication failed",
                    }
                },
            )

    # Set auth cookies
    jwt_service.set_auth_cookies(response, result.access_token, result.refresh_token)

    return {
        "user": result.user.model_dump() if result.user else None,
        "profile": result.profile.model_dump() if result.profile else None,
    }


@router.post("/logout")
async def logout(
    request: Request,
    response: Response,
):
    """
    End the current session.

    Revokes the refresh token and clears authentication cookies.

    FR-013
    """
    refresh_token = jwt_service.get_refresh_token_from_request(request)

    # Extract user_id from token for logging
    user_id = None
    if refresh_token:
        payload = jwt_service.verify_token(refresh_token, expected_type="refresh")
        if payload:
            user_id = payload.get("user_id")
        await auth_service.logout(refresh_token, user_id=user_id)

    # Clear cookies regardless
    jwt_service.clear_auth_cookies(response)

    return {"message": "Logged out successfully"}


@router.post("/refresh")
async def refresh_token(
    request: Request,
    response: Response,
):
    """
    Refresh access token using refresh token from cookie.

    Implements silent token refresh (FR-010).

    FR-010, FR-011, FR-012
    """
    refresh_token = jwt_service.get_refresh_token_from_request(request)

    if not refresh_token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={
                "error": {
                    "code": "NO_REFRESH_TOKEN",
                    "message": "No refresh token provided",
                }
            },
        )

    ip_address, user_agent = _get_client_info(request)

    result = await auth_service.refresh_tokens(refresh_token, user_agent, ip_address)

    if not result.success:
        # Clear cookies on refresh failure
        jwt_service.clear_auth_cookies(response)
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={
                "error": {
                    "code": result.error_code,
                    "message": result.error_message,
                }
            },
        )

    # Set new auth cookies
    jwt_service.set_auth_cookies(response, result.access_token, result.refresh_token)

    return {
        "message": "Token refreshed",
        "expires_in": jwt_service.get_token_expiry_seconds(),
    }


@router.get("/me")
async def get_current_user_info(
    current_user: AuthenticatedUser = Depends(get_current_user),
):
    """
    Get current authenticated user information.

    Used to check authentication status on page load.
    """
    try:
        logger.info(f"GET /auth/me called for user_id: {current_user.user_id}")
        result = await auth_service.get_current_user(current_user.user_id)

        if not result:
            logger.warning(f"User not found for user_id: {current_user.user_id}")
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail={
                    "error": {
                        "code": "USER_NOT_FOUND",
                        "message": "User not found",
                    }
                },
            )

        user_response, profile = result
        logger.info(f"GET /auth/me successful for user: {current_user.email}")

        return {
            "user": user_response.model_dump(),
            "profile": profile.model_dump() if profile else None,
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in /auth/me for user_id {current_user.user_id}: {type(e).__name__}: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": {
                    "code": "INTERNAL_ERROR",
                    "message": f"An error occurred: {type(e).__name__}",
                    "debug": str(e),
                }
            },
        )
