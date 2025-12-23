/**
 * Signup page for the Physical AI & Humanoid Robotics Textbook.
 * Provides email/password registration for new users.
 */

import React, { useEffect } from 'react';
import Layout from '@theme/Layout';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { useAuth } from '../hooks/useAuth';
import { SignupForm } from '../components/Auth';
import styles from './auth.module.css';

export default function SignupPage(): JSX.Element {
  const { isAuthenticated, isLoading } = useAuth();

  // Get base URLs for navigation
  const homeUrl = useBaseUrl('/');
  const loginUrl = useBaseUrl('/login');

  // Redirect to home if already authenticated
  useEffect(() => {
    if (isAuthenticated && !isLoading) {
      // Use window.location for Docusaurus compatibility
      window.location.href = homeUrl;
    }
  }, [isAuthenticated, isLoading, homeUrl]);

  const handleSuccess = () => {
    // Redirect to home (profile wizard will be triggered by AuthContext)
    window.location.href = homeUrl;
  };

  const handleLoginClick = () => {
    window.location.href = loginUrl;
  };

  // Show loading state while checking auth
  if (isLoading) {
    return (
      <Layout title="Create Account" description="Create a new account">
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
      <Layout title="Create Account" description="Create a new account">
        <main className={styles.authPage}>
          <div className={styles.authContainer}>
            <p>Redirecting...</p>
          </div>
        </main>
      </Layout>
    );
  }

  return (
    <Layout title="Create Account" description="Create a new account">
      <main className={styles.authPage}>
        <div className={styles.authContainer}>
          <SignupForm onSuccess={handleSuccess} onLoginClick={handleLoginClick} />
        </div>
      </main>
    </Layout>
  );
}
