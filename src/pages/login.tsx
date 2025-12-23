/**
 * Login page for the Physical AI & Humanoid Robotics Textbook.
 * Provides email/password login for returning users.
 * Handles OAuth callback error display.
 */

import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { useAuth } from '../hooks/useAuth';
import { LoginForm } from '../components/Auth';
import styles from './auth.module.css';

/**
 * Parse OAuth error from URL query parameters.
 */
function getOAuthError(): string | null {
  if (typeof window === 'undefined') return null;

  const params = new URLSearchParams(window.location.search);
  const error = params.get('error');

  if (!error) return null;

  // Map error codes to user-friendly messages
  const errorMessages: Record<string, string> = {
    oauth_denied: 'Google sign-in was cancelled.',
    missing_code: 'Authentication failed. Please try again.',
    missing_state: 'Authentication session expired. Please try again.',
    invalid_state: 'Authentication session expired. Please try again.',
    oauth_failed: params.get('message') || 'Google sign-in failed. Please try again.',
    server_error: 'An error occurred. Please try again later.',
  };

  return errorMessages[error] || 'An error occurred during sign-in.';
}

export default function LoginPage(): JSX.Element {
  const { isAuthenticated, isLoading } = useAuth();
  const [oauthError, setOauthError] = useState<string | null>(null);

  // Get base URLs for navigation
  const homeUrl = useBaseUrl('/');
  const signupUrl = useBaseUrl('/signup');

  // Check for OAuth errors on mount
  useEffect(() => {
    const error = getOAuthError();
    if (error) {
      setOauthError(error);
      // Clean up URL by removing query params
      window.history.replaceState({}, '', window.location.pathname);
    }
  }, []);

  // Redirect to home if already authenticated
  useEffect(() => {
    if (isAuthenticated && !isLoading) {
      // Use window.location for Docusaurus compatibility
      window.location.href = homeUrl;
    }
  }, [isAuthenticated, isLoading, homeUrl]);

  const handleSuccess = () => {
    // Redirect to home or profile wizard
    window.location.href = homeUrl;
  };

  const handleSignupClick = () => {
    window.location.href = signupUrl;
  };

  // Show loading state while checking auth
  if (isLoading) {
    return (
      <Layout title="Log In" description="Log in to your account">
        <main className={styles.authPage}>
          <div className={styles.authContainer}>
            <p>Loading...</p>
          </div>
        </main>
      </Layout>
    );
  }

  // Don't render form if already authenticated (will redirect)
  if (isAuthenticated) {
    return (
      <Layout title="Log In" description="Log in to your account">
        <main className={styles.authPage}>
          <div className={styles.authContainer}>
            <p>Redirecting...</p>
          </div>
        </main>
      </Layout>
    );
  }

  return (
    <Layout title="Log In" description="Log in to your account">
      <main className={styles.authPage}>
        <div className={styles.authContainer}>
          {oauthError && (
            <div className={styles.oauthError} role="alert">
              {oauthError}
            </div>
          )}
          <LoginForm onSuccess={handleSuccess} onSignupClick={handleSignupClick} />
        </div>
      </main>
    </Layout>
  );
}
