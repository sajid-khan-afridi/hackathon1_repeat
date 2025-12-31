/**
 * Profile page for viewing and editing user learning preferences.
 * Accessible at /profile route.
 *
 * Updated: T059 - Added recommendations section (User Story 3)
 */

import React, { useEffect } from 'react';
import { useHistory } from '@docusaurus/router';
import Layout from '@theme/Layout';
import { useAuth } from '../hooks/useAuth';
import { useRecommendations } from '../hooks/useRecommendations';
import { ProfileSettings } from '../components/Profile/ProfileSettings';
import RecommendationCard from '../components/RecommendationCard';
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

        {/* T059: Recommendations section - User Story 3 */}
        <RecommendationsSection />
      </div>
    </Layout>
  );
}

/**
 * Recommendations section component for displaying personalized chapter recommendations.
 * Implementation: T059 - User Story 3 (Smart Chapter Recommendations)
 */
function RecommendationsSection(): JSX.Element {
  const { recommendations, isLoading, error, fromCache, forceRefresh } = useRecommendations();
  const history = useHistory();

  const handleRecommendationClick = (chapterId: string) => {
    // Navigate to the chapter page
    history.push(`/docs/${chapterId}`);
  };

  return (
    <div style={{ marginTop: '3rem' }}>
      <div
        style={{
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center',
          marginBottom: '1.5rem',
        }}
      >
        <h2 style={{ margin: 0, fontSize: '1.75rem', fontWeight: 600 }}>📚 Recommended For You</h2>
        <button
          onClick={forceRefresh}
          disabled={isLoading}
          style={{
            padding: '0.5rem 1rem',
            borderRadius: '0.375rem',
            border: '1px solid #007bff',
            backgroundColor: '#007bff',
            color: '#fff',
            cursor: isLoading ? 'not-allowed' : 'pointer',
            fontSize: '0.875rem',
            fontWeight: 500,
            opacity: isLoading ? 0.6 : 1,
          }}
          aria-label="Refresh recommendations"
        >
          {isLoading ? 'Refreshing...' : '🔄 Refresh'}
        </button>
      </div>

      {fromCache && (
        <p style={{ fontSize: '0.875rem', color: '#6c757d', marginBottom: '1rem' }}>
          💾 Showing cached recommendations (refreshed hourly)
        </p>
      )}

      {isLoading && !recommendations.length && (
        <div style={{ textAlign: 'center', padding: '2rem', color: '#6c757d' }}>
          <p>Loading personalized recommendations...</p>
        </div>
      )}

      {error && (
        <div
          style={{
            padding: '1rem',
            backgroundColor: '#f8d7da',
            border: '1px solid #f5c6cb',
            borderRadius: '0.375rem',
            color: '#721c24',
            marginBottom: '1rem',
          }}
          role="alert"
        >
          <strong>Error:</strong> {error}
        </div>
      )}

      {!isLoading && !error && recommendations.length === 0 && (
        <div style={{ textAlign: 'center', padding: '2rem', color: '#6c757d' }}>
          <p>
            No recommendations available at this time. Complete more chapters or update your profile
            to get personalized suggestions!
          </p>
        </div>
      )}

      {recommendations.length > 0 && (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '1rem' }}>
          {recommendations.map((recommendation) => (
            <RecommendationCard
              key={recommendation.chapter_id}
              recommendation={recommendation}
              onClick={() => handleRecommendationClick(recommendation.chapter_id)}
              showScore={true}
            />
          ))}
        </div>
      )}

      {recommendations.length > 0 && (
        <p
          style={{ fontSize: '0.875rem', color: '#6c757d', marginTop: '1rem', textAlign: 'center' }}
        >
          💡 Recommendations are based on your skill level, learning goals, and progress
        </p>
      )}
    </div>
  );
}
