/**
 * Profile Banner component for prompting users to complete their profile.
 * Implements FR-019 (show profile completion banner for incomplete profiles).
 */

import React, { useState } from 'react';
import Link from '@docusaurus/Link';
import styles from './ProfileBanner.module.css';

interface ProfileBannerProps {
  onDismiss?: () => void;
}

export function ProfileBanner({ onDismiss }: ProfileBannerProps): JSX.Element {
  const [isDismissed, setIsDismissed] = useState(false);

  const handleDismiss = () => {
    setIsDismissed(true);
    onDismiss?.();
  };

  if (isDismissed) {
    return null;
  }

  return (
    <div className={styles.banner} role="banner" aria-live="polite">
      <div className={styles.bannerContent}>
        <div className={styles.bannerIcon}>
          <svg width="20" height="20" viewBox="0 0 20 20" fill="none" aria-hidden="true">
            <path
              d="M10 0C4.48 0 0 4.48 0 10s4.48 10 10 10 10-4.48 10-10S15.52 0 10 0zm1 15H9v-2h2v2zm0-4H9V5h2v6z"
              fill="currentColor"
            />
          </svg>
        </div>
        <p className={styles.bannerText}>
          Complete your profile to get personalized content recommendations and learning paths.
        </p>
        <div className={styles.bannerActions}>
          <Link to="/profile" className={styles.bannerButton}>
            Complete Profile
          </Link>
          <button
            type="button"
            onClick={handleDismiss}
            className={styles.dismissButton}
            aria-label="Dismiss profile banner"
          >
            &times;
          </button>
        </div>
      </div>
    </div>
  );
}
