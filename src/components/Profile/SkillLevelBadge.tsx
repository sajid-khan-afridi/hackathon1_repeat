/**
 * Skill Level Badge Component
 * Displays user's computed skill level classification (beginner/intermediate/advanced)
 * Implements T023 - skill level display in UI
 */

import React from 'react';
import { useSkillLevel, SkillLevel } from '../../hooks/useSkillLevel';
import styles from './SkillLevelBadge.module.css';

interface SkillLevelBadgeProps {
  /**
   * Optional: show loading skeleton instead of using hook internally
   */
  isLoading?: boolean;

  /**
   * Optional: override skill level from hook
   */
  level?: SkillLevel | null;

  /**
   * Optional: show recalculate button
   */
  showRecalculate?: boolean;

  /**
   * Optional: size variant
   */
  size?: 'small' | 'medium' | 'large';
}

/**
 * Badge component to display user skill level classification
 *
 * @example
 * ```tsx
 * // Auto-fetch skill level
 * <SkillLevelBadge showRecalculate />
 *
 * // With controlled level
 * <SkillLevelBadge level="intermediate" size="small" />
 * ```
 */
export function SkillLevelBadge({
  isLoading: externalLoading,
  level: externalLevel,
  showRecalculate = false,
  size = 'medium',
}: SkillLevelBadgeProps): JSX.Element {
  const {
    skillLevel: hookLevel,
    isLoading: hookLoading,
    error,
    recalculate,
    isRecalculating,
  } = useSkillLevel();

  const skillLevel = externalLevel !== undefined ? externalLevel : hookLevel;
  const isLoading = externalLoading !== undefined ? externalLoading : hookLoading;

  /**
   * Get badge styling based on skill level
   */
  const getBadgeClass = (): string => {
    const baseClass = styles.badge;
    const sizeClass = styles[`badge--${size}`];

    if (!skillLevel) return `${baseClass} ${sizeClass}`;

    const levelClass = styles[`badge--${skillLevel}`];
    return `${baseClass} ${sizeClass} ${levelClass}`;
  };

  /**
   * Get label text for skill level
   */
  const getLevelLabel = (): string => {
    if (!skillLevel) return 'Not classified';

    const labels: Record<SkillLevel, string> = {
      beginner: 'Beginner',
      intermediate: 'Intermediate',
      advanced: 'Advanced',
    };

    return labels[skillLevel];
  };

  /**
   * Get icon for skill level
   */
  const getLevelIcon = (): string => {
    if (!skillLevel) return '⭐';

    const icons: Record<SkillLevel, string> = {
      beginner: '🌱',
      intermediate: '🚀',
      advanced: '⚡',
    };

    return icons[skillLevel];
  };

  /**
   * Handle recalculate button click
   */
  const handleRecalculate = async () => {
    try {
      await recalculate();
    } catch (err) {
      console.error('[SkillLevelBadge] Recalculate failed:', err);
    }
  };

  if (isLoading) {
    return (
      <div className={`${styles.container} ${styles[`container--${size}`]}`}>
        <div className={styles.skeleton} aria-label="Loading skill level">
          <span className={styles.skeletonIcon}></span>
          <span className={styles.skeletonText}></span>
        </div>
      </div>
    );
  }

  if (error && !skillLevel) {
    return (
      <div className={`${styles.container} ${styles[`container--${size}`]}`}>
        <div className={styles.error} role="alert">
          <span>⚠️</span>
          <span className={styles.errorText}>Unable to load skill level</span>
        </div>
      </div>
    );
  }

  return (
    <div className={`${styles.container} ${styles[`container--${size}`]}`}>
      <div className={getBadgeClass()} role="status" aria-label={`Skill level: ${getLevelLabel()}`}>
        <span className={styles.icon} aria-hidden="true">
          {getLevelIcon()}
        </span>
        <span className={styles.label}>{getLevelLabel()}</span>
      </div>

      {showRecalculate && (
        <button
          onClick={handleRecalculate}
          disabled={isRecalculating}
          className={styles.recalculateButton}
          aria-label="Recalculate skill level based on current profile"
          title="Recalculate skill level"
        >
          {isRecalculating ? (
            <>
              <span className={styles.spinner} aria-hidden="true"></span>
              <span>Recalculating...</span>
            </>
          ) : (
            <>
              <span aria-hidden="true">🔄</span>
              <span>Recalculate</span>
            </>
          )}
        </button>
      )}
    </div>
  );
}
