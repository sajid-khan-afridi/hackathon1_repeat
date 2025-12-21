/**
 * Profile page for viewing and editing user learning preferences.
 * Accessible at /profile route.
 */

import React, { useEffect } from 'react';
import { useHistory } from '@docusaurus/router';
import Layout from '@theme/Layout';
import { useAuth } from '../hooks/useAuth';
import { ProfileSettings } from '../components/Profile/ProfileSettings';
import styles from './auth.module.css';

export default function ProfilePage(): JSX.Element {
  const { isAuthenticated, isLoading } = useAuth();
  const history = useHistory();

  // Redirect to login if not authenticated
  useEffect(() => {
    if (!isLoading && !isAuthenticated) {
      history.push('/login?redirect=/profile');
    }
  }, [isAuthenticated, isLoading, history]);

  if (isLoading) {
    return (
      <Layout title="Profile" description="Edit your learning profile">
        <div className={styles.pageContainer}>
          <div className={styles.loadingContainer}>
            <p>Loading...</p>
          </div>
        </div>
      </Layout>
    );
  }

  if (!isAuthenticated) {
    return null; // Will redirect via useEffect
  }

  return (
    <Layout title="Profile Settings" description="Update your learning preferences">
      <div className={styles.pageContainer}>
        <ProfileSettings />
      </div>
    </Layout>
  );
}
