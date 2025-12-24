/**
 * SourceCitations Component
 *
 * Displays list of source citations with clickable chapter links.
 * Features:
 * - Collapsible by default to reduce visual clutter
 * - Clickable chapter links (relative URLs to /docs/)
 * - Relevance score badges (color-coded)
 * - Position indicators (1-5 ranking)
 * - Text excerpts for context
 * - Keyboard navigation and screen reader support
 */

import React, { useState } from 'react';
import styles from './ChatbotWidget.module.css';
import type { SourceCitation } from './types';

interface SourceCitationsProps {
  sources: SourceCitation[];
  /** Whether to start in expanded state */
  defaultExpanded?: boolean;
}

/**
 * Formats chapter_id to URL path
 *
 * Handles multiple formats:
 * - Qdrant format: "..-..-..-docs-module-2-isaac-sim-chapter-1-introduction"
 *   -> "/docs/module-2-isaac-sim/chapter-1-introduction"
 * - Legacy format: "module-1-chapter-2" -> "/docs/module-1/chapter-2"
 */
function getChapterUrl(chapterId: string): string {
  // Handle Qdrant format: ..-..-..-docs-module-X-name-chapter-Y-name
  if (chapterId.startsWith('..')) {
    // Remove leading parent directory references (..-..-..- or similar)
    const cleaned = chapterId.replace(/^(\.\.?-)+/, '');

    // Format: docs-module-X-name-chapter-Y-name
    // Split on "-chapter-" to separate module path from chapter file
    if (cleaned.startsWith('docs-')) {
      const withoutDocs = cleaned.substring(5); // Remove "docs-"
      const chapterIdx = withoutDocs.indexOf('-chapter-');
      if (chapterIdx > 0) {
        const modulePart = withoutDocs.substring(0, chapterIdx);
        const chapterPart = withoutDocs.substring(chapterIdx + 1);
        return `/docs/${modulePart}/${chapterPart}`;
      }
    }
  }

  // Legacy format: module-1-chapter-2 -> /docs/module-1/chapter-2
  const parts = chapterId.split('-');
  if (parts.length >= 4 && parts[0] === 'module' && parts[2] === 'chapter') {
    const moduleNum = parts[1];
    const chapterNum = parts[3];
    return `/docs/module-${moduleNum}/chapter-${chapterNum}`;
  }

  // Fallback: use chapter_id as-is
  return `/docs/${chapterId}`;
}

/**
 * Returns color class based on relevance score
 */
function getRelevanceClass(score: number): string {
  if (score >= 0.8) return 'relevance-high';
  if (score >= 0.5) return 'relevance-medium';
  return 'relevance-low';
}

export default function SourceCitations({
  sources,
  defaultExpanded = false
}: SourceCitationsProps): JSX.Element {
  const [isExpanded, setIsExpanded] = useState(defaultExpanded);

  if (sources.length === 0) {
    return null;
  }

  const toggleExpanded = () => {
    setIsExpanded(prev => !prev);
  };

  return (
    <div className={styles.sourcesContainer}>
      {/* Collapsible header */}
      <button
        onClick={toggleExpanded}
        className={styles.sourcesToggle}
        aria-expanded={isExpanded}
        aria-controls="sources-list"
        type="button"
      >
        <svg
          width="16"
          height="16"
          viewBox="0 0 16 16"
          fill="currentColor"
          aria-hidden="true"
          className={`${styles.toggleIcon} ${isExpanded ? styles.toggleIconExpanded : ''}`}
        >
          <path d="M4 6l4 4 4-4z" />
        </svg>
        <span className={styles.sourcesTitle}>
          Sources ({sources.length})
        </span>
      </button>

      {/* Collapsible content */}
      {isExpanded && (
        <ol
          id="sources-list"
          className={styles.sourcesList}
          aria-label="Source citations"
        >
          {sources.map((source, index) => (
            <li key={`${source.chapter_id}-${index}`} className={styles.sourceItem}>
              {/* Position indicator */}
              <span className={styles.sourcePosition} aria-label={`Rank ${source.position}`}>
                {source.position}
              </span>

              <div className={styles.sourceContent}>
                {/* Chapter link */}
                <a
                  href={getChapterUrl(source.chapter_id)}
                  className={styles.sourceLink}
                  aria-label={`Navigate to ${source.chapter_title}`}
                  // Open in same tab for seamless navigation
                >
                  {source.chapter_title}
                </a>

                {/* Relevance score badge */}
                <span
                  className={`${styles.relevanceBadge} ${styles[getRelevanceClass(source.relevance_score)]}`}
                  aria-label={`Relevance score: ${Math.round(source.relevance_score * 100)}%`}
                >
                  {Math.round(source.relevance_score * 100)}%
                </span>

                {/* Excerpt preview */}
                <p className={styles.sourceExcerpt}>
                  {source.excerpt}
                </p>
              </div>
            </li>
          ))}
        </ol>
      )}
    </div>
  );
}
