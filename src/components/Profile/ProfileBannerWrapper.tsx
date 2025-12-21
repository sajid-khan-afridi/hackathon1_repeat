/**
 * Profile Banner Wrapper - conditionally renders ProfileBanner for authenticated users
 * with incomplete profiles.
 * Implements FR-019 (show profile completion banner for incomplete profiles).
 */

import React from 'react';
import { useAuth } from '../../hooks/useAuth';
import { ProfileBanner } from './ProfileBanner';

export function ProfileBannerWrapper(): JSX.Element | null {
  const { isAuthenticated, user, profile, isLoading } = useAuth();

  // Don't show banner while loading
  if (isLoading) {
    return null;
  }

  // Don't show banner if not authenticated
  if (!isAuthenticated || !user) {
    return null;
  }

  // Don't show banner if profile is complete
  if (profile?.isComplete) {
    return null;
  }

  // Show banner for authenticated users with incomplete profiles
  return <ProfileBanner />;
}
