/**
 * ConfidenceIndicator Component
 *
 * Displays confidence score with color-coded badge and tooltip.
 * Features:
 * - Color coding: red (<0.3), yellow (0.3-0.7), green (>0.7)
 * - Accessible tooltip with keyboard navigation
 * - ARIA live region for dynamic confidence updates
 * - Warning banner for low confidence (0.2-0.3 range)
 */

import React, { useState } from 'react';
import styles from './ChatbotWidget.module.css';

interface ConfidenceIndicatorProps {
  confidence: number; // 0.0 to 1.0
  showWarning?: boolean; // Show prominent warning for 0.2-0.3 range
}

/**
 * Determines confidence level category
 */
function getConfidenceLevel(confidence: number): 'high' | 'medium' | 'low' {
  if (confidence >= 0.7) return 'high';
  if (confidence >= 0.3) return 'medium';
  return 'low';
}

/**
 * Returns explanatory text for confidence score
 */
function getConfidenceExplanation(confidence: number): string {
  const level = getConfidenceLevel(confidence);

  switch (level) {
    case 'high':
      return 'High confidence - The answer is well-supported by textbook sources.';
    case 'medium':
      return 'Medium confidence - The answer may be partially relevant. Consider rephrasing your question.';
    case 'low':
      return 'Low confidence - The answer may not be reliable. Try rephrasing or asking about a different topic.';
  }
}

export default function ConfidenceIndicator({
  confidence,
  showWarning = false
}: ConfidenceIndicatorProps): React.ReactElement | null {
  const [isTooltipVisible, setIsTooltipVisible] = useState(false);
  const level = getConfidenceLevel(confidence);
  const percentage = Math.round(confidence * 100);
  const explanation = getConfidenceExplanation(confidence);

  return (
    <div className={styles.confidenceContainer}>
      {/* Warning banner for low confidence (0.2-0.3) */}
      {showWarning && confidence >= 0.2 && confidence < 0.3 && (
        <div
          className={styles.confidenceWarning}
          role="alert"
          aria-live="assertive"
        >
          <svg
            width="20"
            height="20"
            viewBox="0 0 20 20"
            fill="currentColor"
            aria-hidden="true"
            className={styles.warningIcon}
          >
            <path d="M10 0C4.48 0 0 4.48 0 10s4.48 10 10 10 10-4.48 10-10S15.52 0 10 0zm1 15H9v-2h2v2zm0-4H9V5h2v6z" />
          </svg>
          <span>
            This answer has low confidence. Consider rephrasing your question for better results.
          </span>
        </div>
      )}

      {/* Confidence badge with tooltip */}
      <div
        className={styles.confidenceBadgeWrapper}
        onMouseEnter={() => setIsTooltipVisible(true)}
        onMouseLeave={() => setIsTooltipVisible(false)}
        onFocus={() => setIsTooltipVisible(true)}
        onBlur={() => setIsTooltipVisible(false)}
      >
        <button
          className={`${styles.confidenceBadge} ${styles[`confidence-${level}`]}`}
          aria-label={`Confidence: ${percentage}%. ${explanation}`}
          aria-describedby="confidence-tooltip"
          // Make badge focusable for keyboard accessibility
          type="button"
        >
          <span className={styles.confidenceLabel}>Confidence:</span>
          <span className={styles.confidenceValue}>{percentage}%</span>
        </button>

        {/* Accessible tooltip */}
        {isTooltipVisible && (
          <div
            id="confidence-tooltip"
            role="tooltip"
            className={styles.confidenceTooltip}
            // ARIA live region for screen readers
            aria-live="polite"
          >
            {explanation}
          </div>
        )}
      </div>
    </div>
  );
}
