/**
 * GlossaryTooltip Component
 * Displays a hover tooltip with English term and definition for technical terms
 * Phase 5: Translation Feature - T038
 */

import React, { useState, useRef, useCallback } from 'react';
import { useGlossary } from '../../hooks/useGlossary';
import { useLanguageContext } from '../../context/LanguageContext';
import styles from './GlossaryTooltip.module.css';

export interface GlossaryTooltipProps {
  /** The term ID to display tooltip for */
  termId: string;
  /** Children to wrap with tooltip (the term text) */
  children: React.ReactNode;
  /** Optional className for styling */
  className?: string;
}

/**
 * GlossaryTooltip wraps a technical term and shows a tooltip with the
 * English term and its definition when hovered or focused.
 *
 * @example
 * ```tsx
 * <GlossaryTooltip termId="ros-2">
 *   ROS 2
 * </GlossaryTooltip>
 * ```
 */
export function GlossaryTooltip({
  termId,
  children,
  className,
}: GlossaryTooltipProps): React.ReactElement {
  const { getTermById, getRelatedTerms } = useGlossary();
  const { language } = useLanguageContext();
  const [isVisible, setIsVisible] = useState(false);
  const containerRef = useRef<HTMLSpanElement>(null);
  const tooltipRef = useRef<HTMLDivElement>(null);
  const hideTimeoutRef = useRef<NodeJS.Timeout | null>(null);

  const term = getTermById(termId);

  const showTooltip = useCallback(() => {
    if (hideTimeoutRef.current) {
      clearTimeout(hideTimeoutRef.current);
      hideTimeoutRef.current = null;
    }
    setIsVisible(true);
  }, []);

  const hideTooltip = useCallback(() => {
    // Add small delay to allow moving to tooltip
    hideTimeoutRef.current = setTimeout(() => {
      setIsVisible(false);
    }, 100);
  }, []);

  const handleKeyDown = useCallback((event: React.KeyboardEvent) => {
    if (event.key === 'Escape') {
      setIsVisible(false);
    }
  }, []);

  // If term not found, just render children
  if (!term) {
    return <span className={className}>{children}</span>;
  }

  const relatedTerms = getRelatedTerms(termId);

  return (
    <span
      ref={containerRef}
      className={`${styles.container} ${className || ''}`}
      onMouseEnter={showTooltip}
      onMouseLeave={hideTooltip}
      onFocus={showTooltip}
      onBlur={hideTooltip}
      onKeyDown={handleKeyDown}
    >
      <span
        className={styles.term}
        tabIndex={0}
        role="button"
        aria-describedby={isVisible ? `tooltip-${termId}` : undefined}
        data-technical-term
      >
        {children}
      </span>

      {isVisible && (
        <div
          ref={tooltipRef}
          id={`tooltip-${termId}`}
          role="tooltip"
          className={styles.tooltip}
          onMouseEnter={showTooltip}
          onMouseLeave={hideTooltip}
        >
          <div className={styles.tooltipHeader}>
            <span className={styles.englishTerm}>{term.english}</span>
            {language === 'ur' && (
              <span className={styles.urduTerm}>{term.urduTransliteration}</span>
            )}
          </div>

          <p className={styles.definition}>
            {language === 'ur' ? term.definitionUrdu : term.definition}
          </p>

          {term.category && (
            <span className={styles.category}>{term.category}</span>
          )}

          {relatedTerms.length > 0 && (
            <div className={styles.relatedTerms}>
              <span className={styles.relatedLabel}>
                {language === 'ur' ? 'متعلقہ:' : 'Related:'}
              </span>
              {relatedTerms.map((related, index) => (
                <span key={related.id} className={styles.relatedTerm}>
                  {related.english}
                  {index < relatedTerms.length - 1 && ', '}
                </span>
              ))}
            </div>
          )}

          <div className={styles.tooltipArrow} aria-hidden="true" />
        </div>
      )}
    </span>
  );
}

export default GlossaryTooltip;
