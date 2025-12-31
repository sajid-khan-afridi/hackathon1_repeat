/**
 * TranslationErrorBanner Component
 * Displays a notification when translation is unavailable with fallback to English
 * Phase 5: Translation Feature - T044
 */

import React, { useState, useCallback, useEffect } from 'react';
import { useLanguageContext } from '../../context/LanguageContext';
import styles from './TranslationErrorBanner.module.css';

export type TranslationErrorType = 'unavailable' | 'loading_failed' | 'in_progress' | 'offline';

export interface TranslationErrorBannerProps {
  /** Type of error to display */
  errorType?: TranslationErrorType;
  /** Callback when retry is clicked */
  onRetry?: () => void;
  /** Callback when dismiss is clicked */
  onDismiss?: () => void;
  /** Whether to show retry button */
  showRetry?: boolean;
  /** Custom message override */
  customMessage?: string;
  /** Chapter or section name for context */
  contentName?: string;
}

/**
 * TranslationErrorBanner shows a user-friendly message when translation
 * content is unavailable, with options to retry or dismiss.
 */
export function TranslationErrorBanner({
  errorType = 'unavailable',
  onRetry,
  onDismiss,
  showRetry = true,
  customMessage,
  contentName,
}: TranslationErrorBannerProps): React.ReactElement | null {
  const { language, setLanguage, hasTranslationError } = useLanguageContext();
  const [isDismissed, setIsDismissed] = useState(false);
  const [isRetrying, setIsRetrying] = useState(false);
  const [isOnline, setIsOnline] = useState(true);

  // Monitor online status
  useEffect(() => {
    if (typeof window === 'undefined') {
      return;
    }

    const handleOnline = () => setIsOnline(true);
    const handleOffline = () => setIsOnline(false);

    setIsOnline(navigator.onLine);
    window.addEventListener('online', handleOnline);
    window.addEventListener('offline', handleOffline);

    return () => {
      window.removeEventListener('online', handleOnline);
      window.removeEventListener('offline', handleOffline);
    };
  }, []);

  const handleDismiss = useCallback(() => {
    setIsDismissed(true);
    onDismiss?.();
  }, [onDismiss]);

  const handleRetry = useCallback(async () => {
    setIsRetrying(true);
    try {
      if (onRetry) {
        await onRetry();
      } else {
        // Default retry: reload the page
        window.location.reload();
      }
    } finally {
      setIsRetrying(false);
    }
  }, [onRetry]);

  const handleSwitchToEnglish = useCallback(() => {
    setLanguage('en');
    setIsDismissed(true);
  }, [setLanguage]);

  // Don't show if dismissed or no error
  if (isDismissed) {
    return null;
  }

  // Only show when there's a translation error and language is not English
  if (language === 'en' && !hasTranslationError) {
    return null;
  }

  // Determine effective error type
  const effectiveErrorType = !isOnline ? 'offline' : errorType;

  // Get message based on error type
  const getErrorMessage = (): { title: string; description: string } => {
    if (customMessage) {
      return { title: 'Translation Notice', description: customMessage };
    }

    switch (effectiveErrorType) {
      case 'offline':
        return {
          title: language === 'ur' ? 'آف لائن' : 'You are offline',
          description:
            language === 'ur'
              ? 'انٹرنیٹ کنکشن نہیں ہے۔ مواد انگریزی میں دکھایا جا رہا ہے۔'
              : 'No internet connection. Content is displayed in English.',
        };
      case 'loading_failed':
        return {
          title: language === 'ur' ? 'ترجمہ لوڈ نہیں ہو سکا' : 'Translation failed to load',
          description:
            language === 'ur'
              ? `${contentName ? `"${contentName}" کا ` : ''}ترجمہ لوڈ کرنے میں خرابی۔ انگریزی مواد دکھایا جا رہا ہے۔`
              : `Failed to load ${contentName ? `"${contentName}" ` : ''}translation. Showing English content.`,
        };
      case 'in_progress':
        return {
          title: language === 'ur' ? 'ترجمہ جاری ہے' : 'Translation in progress',
          description:
            language === 'ur'
              ? `${contentName ? `"${contentName}" کا ` : ''}ترجمہ ابھی تیار ہو رہا ہے۔ انگریزی مواد دستیاب ہے۔`
              : `${contentName ? `"${contentName}" ` : ''}translation is being prepared. English content is available.`,
        };
      case 'unavailable':
      default:
        return {
          title: language === 'ur' ? 'ترجمہ دستیاب نہیں' : 'Translation unavailable',
          description:
            language === 'ur'
              ? `${contentName ? `"${contentName}" کا ` : ''}اردو ترجمہ ابھی دستیاب نہیں ہے۔ انگریزی مواد دکھایا جا رہا ہے۔`
              : `${contentName ? `"${contentName}" ` : ''}Urdu translation is not available. Showing English content.`,
        };
    }
  };

  const { title, description } = getErrorMessage();

  // Get icon based on error type
  const getIcon = (): string => {
    switch (effectiveErrorType) {
      case 'offline':
        return '📡';
      case 'loading_failed':
        return '⚠️';
      case 'in_progress':
        return '⏳';
      case 'unavailable':
      default:
        return '🌐';
    }
  };

  return (
    <div
      className={`${styles.banner} ${styles[effectiveErrorType]}`}
      role="alert"
      aria-live="polite"
    >
      <div className={styles.content}>
        <span className={styles.icon} aria-hidden="true">
          {getIcon()}
        </span>
        <div className={styles.message}>
          <strong className={styles.title}>{title}</strong>
          <p className={styles.description}>{description}</p>
        </div>
      </div>

      <div className={styles.actions}>
        {showRetry && effectiveErrorType !== 'in_progress' && (
          <button
            type="button"
            className={styles.retryButton}
            onClick={handleRetry}
            disabled={isRetrying || !isOnline}
            aria-label={language === 'ur' ? 'دوبارہ کوشش کریں' : 'Retry loading translation'}
          >
            {isRetrying ? (
              <span className={styles.spinner} aria-hidden="true" />
            ) : (
              <span>{language === 'ur' ? 'دوبارہ' : 'Retry'}</span>
            )}
          </button>
        )}

        {language !== 'en' && (
          <button
            type="button"
            className={styles.switchButton}
            onClick={handleSwitchToEnglish}
            aria-label={language === 'ur' ? 'انگریزی میں دیکھیں' : 'View in English'}
          >
            {language === 'ur' ? 'انگریزی میں دیکھیں' : 'View in English'}
          </button>
        )}

        <button
          type="button"
          className={styles.dismissButton}
          onClick={handleDismiss}
          aria-label={language === 'ur' ? 'بند کریں' : 'Dismiss'}
        >
          <span aria-hidden="true">×</span>
        </button>
      </div>
    </div>
  );
}

export default TranslationErrorBanner;
