import React from 'react';
import { useUserContext } from '@site/src/context/UserContext';

interface PersonalizedSectionProps {
  level: 'beginner' | 'intermediate' | 'advanced';
  rosFamiliarity?: 'novice' | 'intermediate' | 'expert';
  hardwareAccess?: boolean;
  children: React.ReactNode;
}

/**
 * Component for rendering personalized content based on user profile.
 *
 * Usage:
 * <PersonalizedSection level="beginner" hardwareAccess={false}>
 *   Content for beginners without hardware access
 * </PersonalizedSection>
 */
export default function PersonalizedSection({
  level,
  rosFamiliarity,
  hardwareAccess,
  children,
}: PersonalizedSectionProps): JSX.Element | null {
  const userProfile = useUserContext();

  // If no user context is available, render default content
  if (!userProfile) {
    return <>{children}</>;
  }

  // Check if content matches user's experience level
  if (userProfile.experienceLevel !== level) {
    return null;
  }

  // Check ROS familiarity if specified
  if (rosFamiliarity && userProfile.rosFamiliarity !== rosFamiliarity) {
    return null;
  }

  // Check hardware access if specified
  if (hardwareAccess !== undefined && userProfile.hardwareAccess !== hardwareAccess) {
    return null;
  }

  return <>{children}</>;
}