import React, { useMemo } from 'react';
import { useUserContext } from '@site/src/context/UserContext';
import { useAuth } from '@site/src/hooks/useAuth';
import type { ROSFamiliarity, HardwareAccess } from '@site/src/types/auth';

interface PersonalizedSectionProps {
  level?: 'beginner' | 'intermediate' | 'advanced';
  rosFamiliarity?: 'novice' | 'intermediate' | 'expert';
  hardwareAccess?: boolean;
  language?: 'python' | 'cpp';
  hardware?: 'physical' | 'simulation';
  children: React.ReactNode;
}

/**
 * Maps AuthContext ROSFamiliarity to PersonalizedSection rosFamiliarity values
 */
function mapRosFamiliarity(ros: ROSFamiliarity | null | undefined): 'novice' | 'intermediate' | 'expert' | undefined {
  switch (ros) {
    case 'none':
      return 'novice';
    case 'basic':
      return 'intermediate';
    case 'proficient':
      return 'expert';
    default:
      return undefined;
  }
}

/**
 * Maps AuthContext HardwareAccess to boolean for PersonalizedSection
 */
function mapHardwareAccess(hardware: HardwareAccess | null | undefined): boolean | undefined {
  if (hardware === null || hardware === undefined) return undefined;
  return hardware !== 'simulation_only';
}

/**
 * Component for rendering personalized content based on user profile.
 * Includes accessibility features for screen readers and keyboard navigation (WCAG 2.1 AA).
 *
 * Props:
 * - level: Target experience level (beginner/intermediate/advanced)
 * - rosFamiliarity: Required ROS familiarity (novice/intermediate/expert)
 * - hardwareAccess: Deprecated - use 'hardware' prop instead
 * - language: Target programming language (python/cpp)
 * - hardware: Target hardware setup (physical/simulation)
 *
 * Usage Examples:
 * ```mdx
 * <PersonalizedSection level="beginner">
 *   Content for beginners
 * </PersonalizedSection>
 *
 * <PersonalizedSection language="python">
 *   ```python
 *   # Python code example
 *   node.get_logger().info('Hello ROS 2')
 *   ```
 * </PersonalizedSection>
 *
 * <PersonalizedSection language="cpp">
 *   ```cpp
 *   // C++ code example
 *   node->get_logger()->info("Hello ROS 2");
 *   ```
 * </PersonalizedSection>
 *
 * <PersonalizedSection hardware="physical">
 *   Connect your robot to the power supply...
 * </PersonalizedSection>
 * ```
 *
 * Special Cases:
 * - If user's preferredLanguage is "both", show all code examples (language filtering disabled)
 * - If no user profile is set, all content is shown (default view for anonymous users)
 * - Multiple filters can be combined (e.g., level + language + hardware)
 */
export default function PersonalizedSection({
  level,
  rosFamiliarity,
  hardwareAccess,
  language,
  hardware,
  children,
}: PersonalizedSectionProps): React.ReactElement | null {
  const { userProfile } = useUserContext();
  const { profile: authProfile, isAuthenticated } = useAuth();

  /**
   * Determines if content should be shown based on user profile and section filters.
   * Implements filtering logic from FR-020 to FR-023 (Adaptive Content Depth).
   *
   * Priority: AuthContext profile (source of truth) > UserContext (localStorage fallback)
   *
   * @returns true if content should be shown, false to hide
   */
  const shouldShowContent = useMemo(() => {
    // Determine which profile to use - prefer AuthContext (source of truth)
    let effectiveProfile: {
      experienceLevel?: 'beginner' | 'intermediate' | 'advanced';
      rosFamiliarity?: 'novice' | 'intermediate' | 'expert';
      hardwareAccess?: boolean;
      preferredLanguage?: 'python' | 'cpp' | 'both';
    } | null = null;

    if (isAuthenticated && authProfile?.experienceLevel) {
      // Use auth profile (mapped to match expected types)
      effectiveProfile = {
        experienceLevel: authProfile.experienceLevel,
        rosFamiliarity: mapRosFamiliarity(authProfile.rosFamiliarity),
        hardwareAccess: mapHardwareAccess(authProfile.hardwareAccess),
        preferredLanguage: authProfile.preferredLanguage || undefined,
      };
    } else if (userProfile) {
      // Fallback to localStorage profile for anonymous/legacy users
      effectiveProfile = userProfile;
    }

    // If no profile available, show all content (default view for anonymous users)
    if (!effectiveProfile) {
      return true;
    }

    // Language filtering (FR-020: preferred_language filter)
    if (language) {
      // Special case: If user prefers "both", show all code examples
      if (effectiveProfile.preferredLanguage === 'both') {
        // Allow language-specific content to be shown
      } else if (effectiveProfile.preferredLanguage && effectiveProfile.preferredLanguage !== language) {
        return false; // Hide if language doesn't match user preference
      }
    }

    // Experience level filtering (FR-021: level filter)
    if (level) {
      if (effectiveProfile.experienceLevel !== level) {
        return false; // Hide if level doesn't match
      }
    }

    // ROS familiarity filtering (existing feature)
    if (rosFamiliarity) {
      if (effectiveProfile.rosFamiliarity !== rosFamiliarity) {
        return false;
      }
    }

    // Hardware filtering (FR-022: hardware filter)
    // Support both old (hardwareAccess boolean) and new (hardware string) props
    if (hardware === 'physical') {
      // Only show physical hardware content if user has hardware access
      if (effectiveProfile.hardwareAccess === false) {
        return false;
      }
    } else if (hardware === 'simulation') {
      // Simulation content is always shown (works for both hardware and simulation-only users)
    }

    // Backward compatibility: old hardwareAccess prop
    if (hardwareAccess !== undefined) {
      if (effectiveProfile.hardwareAccess !== hardwareAccess) {
        return false;
      }
    }

    return true; // Show content if all filters pass
  }, [userProfile, authProfile, isAuthenticated, level, rosFamiliarity, hardwareAccess, language, hardware]);

  // Generate accessible label describing the personalization criteria (WCAG 2.1 AA)
  const ariaLabel = useMemo(() => {
    const parts: string[] = [];
    if (level) {
      parts.push(`Content for ${level} level learners`);
    }
    if (rosFamiliarity) {
      parts.push(`with ${rosFamiliarity} ROS familiarity`);
    }
    if (language) {
      parts.push(`${language.toUpperCase()} code examples`);
    }
    if (hardware) {
      parts.push(`for ${hardware} setup`);
    } else if (hardwareAccess !== undefined) {
      parts.push(hardwareAccess ? 'with hardware access' : 'without hardware access');
    }
    return parts.length > 0 ? parts.join(' ') : 'Personalized content';
  }, [level, rosFamiliarity, hardwareAccess, language, hardware]);

  // Hide content if filters don't match (return null for accessibility - screen readers won't announce)
  if (!shouldShowContent) {
    return null;
  }

  // Render content with proper semantic HTML and ARIA attributes
  return (
    <section
      role="region"
      aria-label={ariaLabel}
      className={`personalized-section ${level ? `personalized-section--${level}` : ''} ${
        language ? `personalized-section--${language}` : ''
      }`.trim()}
      data-personalization-level={level}
      data-ros-familiarity={rosFamiliarity}
      data-hardware-access={hardwareAccess}
      data-language={language}
      data-hardware={hardware}
      tabIndex={0}
    >
      {children}
    </section>
  );
}
