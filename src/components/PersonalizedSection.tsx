import React, { useMemo } from 'react';
import { useUserContext } from '@site/src/context/UserContext';

interface PersonalizedSectionProps {
  level: 'beginner' | 'intermediate' | 'advanced';
  rosFamiliarity?: 'novice' | 'intermediate' | 'expert';
  hardwareAccess?: boolean;
  children: React.ReactNode;
}

/**
 * Component for rendering personalized content based on user profile.
 * Includes accessibility features for screen readers.
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
  const { userProfile } = useUserContext();

  // Generate accessible label describing the personalization criteria
  const ariaLabel = useMemo(() => {
    const parts = [`Content for ${level} level learners`];
    if (rosFamiliarity) {
      parts.push(`with ${rosFamiliarity} ROS familiarity`);
    }
    if (hardwareAccess !== undefined) {
      parts.push(hardwareAccess ? 'with hardware access' : 'without hardware access');
    }
    return parts.join(' ');
  }, [level, rosFamiliarity, hardwareAccess]);

  // If no user profile is set, render content for all users (anonymous/default view)
  if (!userProfile) {
    return (
      <section
        role="region"
        aria-label={ariaLabel}
        className="personalized-section personalized-section--default"
        data-personalization-level={level}
      >
        {children}
      </section>
    );
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

  return (
    <section
      role="region"
      aria-label={ariaLabel}
      className={`personalized-section personalized-section--${level}`}
      data-personalization-level={level}
      data-ros-familiarity={rosFamiliarity}
      data-hardware-access={hardwareAccess}
    >
      {children}
    </section>
  );
}