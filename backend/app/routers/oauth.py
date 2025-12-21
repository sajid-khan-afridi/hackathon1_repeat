"""
Google OAuth router for authentication via Google accounts.
Implements FR-007 (Google OAuth login) and FR-008 (account linking).
"""

import logging
from typing import Optional

from fastapi import APIRouter, HTTPException, Query, Request, Response, status
from fastapi.responses import RedirectResponse

from app.config import settings
from app.services.oauth_service import oauth_service
from app.services.github_oauth_service import github_oauth_service
from app.services.user_service import user_service
from app.services.jwt_service import jwt_service
from app.services.auth_service import auth_service

logger = logging.getLogger(__name__)

router = APIRouter()

# In-memory state store for CSRF protection (in production, use Redis or database)
# States expire after 10 minutes
_oauth_states: dict[str, float] = {}


def _cleanup_expired_states() -> None:
    """Remove expired OAuth states (older than 10 minutes)."""
    import time

    current_time = time.time()
    expired = [
        state
        for state, timestamp in _oauth_states.items()
        if current_time - timestamp > 600  # 10 minutes
    ]
    for state in expired:
        del _oauth_states[state]


@router.get("/google")
async def google_oauth_init(
    redirect_uri: Optional[str] = Query(
        None, description="URL to redirect after OAuth completion"
    )
) -> RedirectResponse:
    """
    Initiate Google OAuth flow.
    Redirects to Google's OAuth consent screen.

    FR-007: Enable Google OAuth login
    """
    if not oauth_service.is_configured:
        logger.error("Google OAuth not configured")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail={
                "error": {
                    "code": "OAUTH_NOT_CONFIGURED",
                    "message": "Google OAuth is not configured on this server",
                }
            },
        )

    # Generate CSRF state token
    import time

    _cleanup_expired_states()
    state = oauth_service.generate_state_token()
    _oauth_states[state] = time.time()

    # Build authorization URL
    auth_url = oauth_service.get_authorization_url(
        state=state, redirect_after=redirect_uri
    )

    logger.info("Initiating Google OAuth flow")
    return RedirectResponse(url=auth_url, status_code=status.HTTP_302_FOUND)


@router.get("/google/callback")
async def google_oauth_callback(
    request: Request,
    response: Response,
    code: Optional[str] = Query(None, description="Authorization code from Google"),
    state: Optional[str] = Query(None, description="CSRF state token"),
    error: Optional[str] = Query(None, description="Error code if OAuth was denied"),
    error_description: Optional[str] = Query(
        None, description="Error description if OAuth was denied"
    ),
) -> RedirectResponse:
    """
    Handle Google OAuth callback.
    Creates or links user account and issues JWT tokens.

    FR-007: Enable Google OAuth login
    FR-008: Auto-link accounts with matching email
    """
    # Get frontend URL for redirects
    frontend_url = settings.frontend_url

    # Handle OAuth errors
    if error:
        logger.warning(f"Google OAuth error: {error} - {error_description}")
        error_redirect = f"{frontend_url}/login?error=oauth_denied&message={error_description or error}"
        return RedirectResponse(url=error_redirect, status_code=status.HTTP_302_FOUND)

    # Validate required parameters
    if not code:
        logger.warning("Google OAuth callback missing code parameter")
        return RedirectResponse(
            url=f"{frontend_url}/login?error=missing_code",
            status_code=status.HTTP_302_FOUND,
        )

    if not state:
        logger.warning("Google OAuth callback missing state parameter")
        return RedirectResponse(
            url=f"{frontend_url}/login?error=missing_state",
            status_code=status.HTTP_302_FOUND,
        )

    # Verify CSRF state token
    csrf_token, redirect_after = oauth_service.parse_state(state)
    _cleanup_expired_states()

    if csrf_token not in _oauth_states:
        logger.warning("Invalid or expired OAuth state token")
        return RedirectResponse(
            url=f"{frontend_url}/login?error=invalid_state",
            status_code=status.HTTP_302_FOUND,
        )

    # Remove used state
    del _oauth_states[csrf_token]

    try:
        # Exchange code for user info
        google_user = await oauth_service.authenticate(code)
        logger.info(f"Google OAuth successful for: {google_user['email']}")

        # Check if user exists by Google ID
        user = await user_service.get_user_by_google_id(google_user["sub"])

        if user:
            # Existing Google user - log them in
            logger.info(f"Existing Google user login: {user.id}")
        else:
            # Check if user exists by email (for account linking per FR-008)
            user = await user_service.get_user_by_email(google_user["email"])

            if user:
                # Link Google account to existing email user
                logger.info(f"Linking Google account to existing user: {user.id}")
                await user_service.link_google_account(user.id, google_user["sub"])
            else:
                # Create new user with Google OAuth
                logger.info(f"Creating new user from Google OAuth: {google_user['email']}")
                user = await user_service.create_user_from_google(
                    email=google_user["email"],
                    google_id=google_user["sub"],
                )
                # Create empty profile for new user
                await user_service.create_empty_profile(user.id)

        # Update last login
        await user_service.update_last_login(user.id)

        # Generate tokens
        access_token = jwt_service.create_access_token(user.id, user.email)
        refresh_token = jwt_service.create_refresh_token(user.id, user.email)

        # Store refresh token
        await auth_service._store_refresh_token(
            user_id=user.id,
            refresh_token=refresh_token,
            user_agent=request.headers.get("user-agent"),
            ip_address=request.client.host if request.client else None,
        )

        # Determine redirect URL
        final_redirect = redirect_after or frontend_url

        # Create redirect response
        redirect_response = RedirectResponse(
            url=final_redirect, status_code=status.HTTP_302_FOUND
        )

        # Set auth cookies on the redirect response
        jwt_service.set_auth_cookies(redirect_response, access_token, refresh_token)

        logger.info(f"Google OAuth complete, redirecting to: {final_redirect}")
        return redirect_response

    except ValueError as e:
        logger.error(f"Google OAuth error: {e}")
        return RedirectResponse(
            url=f"{frontend_url}/login?error=oauth_failed&message={str(e)}",
            status_code=status.HTTP_302_FOUND,
        )
    except Exception as e:
        logger.exception(f"Unexpected error in Google OAuth callback: {e}")
        return RedirectResponse(
            url=f"{frontend_url}/login?error=server_error",
            status_code=status.HTTP_302_FOUND,
        )


@router.get("/github")
async def github_oauth_init(
    redirect_uri: Optional[str] = Query(
        None, description="URL to redirect after OAuth completion"
    )
) -> RedirectResponse:
    """
    Initiate GitHub OAuth flow.
    Redirects to GitHub's OAuth authorization page.
    """
    if not github_oauth_service.is_configured:
        logger.error("GitHub OAuth not configured")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail={
                "error": {
                    "code": "OAUTH_NOT_CONFIGURED",
                    "message": "GitHub OAuth is not configured on this server",
                }
            },
        )

    # Generate CSRF state token
    import time

    _cleanup_expired_states()
    state = github_oauth_service.generate_state_token()
    _oauth_states[state] = time.time()

    # Build authorization URL
    auth_url = github_oauth_service.get_authorization_url(
        state=state, redirect_after=redirect_uri
    )

    logger.info("Initiating GitHub OAuth flow")
    return RedirectResponse(url=auth_url, status_code=status.HTTP_302_FOUND)


@router.get("/github/callback")
async def github_oauth_callback(
    request: Request,
    response: Response,
    code: Optional[str] = Query(None, description="Authorization code from GitHub"),
    state: Optional[str] = Query(None, description="CSRF state token"),
    error: Optional[str] = Query(None, description="Error code if OAuth was denied"),
    error_description: Optional[str] = Query(
        None, description="Error description if OAuth was denied"
    ),
) -> RedirectResponse:
    """
    Handle GitHub OAuth callback.
    Creates or links user account and issues JWT tokens.
    """
    # Get frontend URL for redirects
    frontend_url = settings.frontend_url

    # Handle OAuth errors
    if error:
        logger.warning(f"GitHub OAuth error: {error} - {error_description}")
        error_redirect = f"{frontend_url}/login?error=oauth_denied&message={error_description or error}"
        return RedirectResponse(url=error_redirect, status_code=status.HTTP_302_FOUND)

    # Validate required parameters
    if not code:
        logger.warning("GitHub OAuth callback missing code parameter")
        return RedirectResponse(
            url=f"{frontend_url}/login?error=missing_code",
            status_code=status.HTTP_302_FOUND,
        )

    if not state:
        logger.warning("GitHub OAuth callback missing state parameter")
        return RedirectResponse(
            url=f"{frontend_url}/login?error=missing_state",
            status_code=status.HTTP_302_FOUND,
        )

    # Verify CSRF state token
    csrf_token, redirect_after = github_oauth_service.parse_state(state)
    _cleanup_expired_states()

    if csrf_token not in _oauth_states:
        logger.warning("Invalid or expired OAuth state token")
        return RedirectResponse(
            url=f"{frontend_url}/login?error=invalid_state",
            status_code=status.HTTP_302_FOUND,
        )

    # Remove used state
    del _oauth_states[csrf_token]

    try:
        # Exchange code for user info
        github_user = await github_oauth_service.authenticate(code)
        logger.info(f"GitHub OAuth successful for: {github_user['email']}")

        # Check if user exists by GitHub ID
        user = await user_service.get_user_by_github_id(str(github_user["id"]))

        if user:
            # Existing GitHub user - log them in
            logger.info(f"Existing GitHub user login: {user.id}")
        else:
            # Check if user exists by email (for account linking)
            user = await user_service.get_user_by_email(github_user["email"])

            if user:
                # Link GitHub account to existing email user
                logger.info(f"Linking GitHub account to existing user: {user.id}")
                await user_service.link_github_account(user.id, str(github_user["id"]))
            else:
                # Create new user with GitHub OAuth
                logger.info(f"Creating new user from GitHub OAuth: {github_user['email']}")
                user = await user_service.create_user_from_github(
                    email=github_user["email"],
                    github_id=str(github_user["id"]),
                )
                # Create empty profile for new user
                await user_service.create_empty_profile(user.id)

        # Update last login
        await user_service.update_last_login(user.id)

        # Generate tokens
        access_token = jwt_service.create_access_token(user.id, user.email)
        refresh_token = jwt_service.create_refresh_token(user.id, user.email)

        # Store refresh token
        await auth_service._store_refresh_token(
            user_id=user.id,
            refresh_token=refresh_token,
            user_agent=request.headers.get("user-agent"),
            ip_address=request.client.host if request.client else None,
        )

        # Determine redirect URL
        final_redirect = redirect_after or frontend_url

        # Create redirect response
        redirect_response = RedirectResponse(
            url=final_redirect, status_code=status.HTTP_302_FOUND
        )

        # Set auth cookies on the redirect response
        jwt_service.set_auth_cookies(redirect_response, access_token, refresh_token)

        logger.info(f"GitHub OAuth complete, redirecting to: {final_redirect}")
        return redirect_response

    except ValueError as e:
        logger.error(f"GitHub OAuth error: {e}")
        return RedirectResponse(
            url=f"{frontend_url}/login?error=oauth_failed&message={str(e)}",
            status_code=status.HTTP_302_FOUND,
        )
    except Exception as e:
        logger.exception(f"Unexpected error in GitHub OAuth callback: {e}")
        return RedirectResponse(
            url=f"{frontend_url}/login?error=server_error",
            status_code=status.HTTP_302_FOUND,
        )
