/**
 * RecommendationCard component for displaying personalized chapter recommendations.
 * Shows chapter title, relevance score, difficulty level, and recommendation reason.
 *
 * Implementation: T058 - User Story 3 (Smart Chapter Recommendations)
 */

import React from 'react';
import type { ChapterRecommendationResponse } from '../services/personalization-api';
import './RecommendationCard.module.css';

interface RecommendationCardProps {
  /** Recommendation data from API */
  recommendation: ChapterRecommendationResponse;

  /** Click handler for navigating to chapter */
  onClick?: () => void;

  /** Custom CSS class */
  className?: string;

  /** Whether to show relevance score (default: true) */
  showScore?: boolean;
}

/**
 * Get difficulty level display text and color
 */
function getDifficultyInfo(level?: string): { text: string; color: string } {
  switch (level) {
    case 'beginner':
      return { text: 'Beginner', color: '#28a745' }; // Green
    case 'intermediate':
      return { text: 'Intermediate', color: '#ffc107' }; // Yellow
    case 'advanced':
      return { text: 'Advanced', color: '#dc3545' }; // Red
    default:
      return { text: 'All Levels', color: '#6c757d' }; // Gray
  }
}

/**
 * Format relevance score as percentage
 */
function formatScore(score: number): string {
  return `${Math.round(score * 100)}%`;
}

/**
 * RecommendationCard displays a single chapter recommendation with metadata.
 *
 * Features:
 * - Displays chapter title and difficulty level
 * - Shows relevance score and reason
 * - Keyboard accessible (clickable via Enter/Space)
 * - WCAG 2.1 AA compliant
 *
 * @example
 * ```tsx
 * <RecommendationCard
 *   recommendation={recommendation}
 *   onClick={() => navigate(`/docs/${recommendation.chapter_id}`)}
 *   showScore={true}
 * />
 * ```
 */
export default function RecommendationCard({
  recommendation,
  onClick,
  className = '',
  showScore = true,
}: RecommendationCardProps): React.ReactElement {
  const difficultyInfo = getDifficultyInfo(recommendation.difficulty_level);

  const handleClick = () => {
    if (onClick) {
      onClick();
    }
  };

  const handleKeyDown = (event: React.KeyboardEvent) => {
    if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault();
      handleClick();
    }
  };

  return (
    <div
      className={`recommendation-card ${className}`}
      onClick={handleClick}
      onKeyDown={handleKeyDown}
      role="button"
      tabIndex={0}
      aria-label={`Recommended chapter: ${recommendation.title || recommendation.chapter_id}`}
    >
      {/* Card header with title and difficulty */}
      <div className="recommendation-card__header">
        <h3 className="recommendation-card__title">
          {recommendation.title || recommendation.chapter_id}
        </h3>
        {recommendation.difficulty_level && (
          <span
            className="recommendation-card__difficulty"
            style={{
              backgroundColor: difficultyInfo.color,
              color: '#fff',
              padding: '0.25rem 0.5rem',
              borderRadius: '0.25rem',
              fontSize: '0.875rem',
              fontWeight: 'bold',
            }}
            aria-label={`Difficulty: ${difficultyInfo.text}`}
          >
            {difficultyInfo.text}
          </span>
        )}
      </div>

      {/* Relevance score bar */}
      {showScore && (
        <div className="recommendation-card__score-container" aria-label="Relevance score">
          <div className="recommendation-card__score-label">
            Match: {formatScore(recommendation.relevance_score)}
          </div>
          <div className="recommendation-card__score-bar">
            <div
              className="recommendation-card__score-fill"
              style={{
                width: `${recommendation.relevance_score * 100}%`,
                backgroundColor: '#007bff',
                height: '0.5rem',
                borderRadius: '0.25rem',
                transition: 'width 0.3s ease',
              }}
              aria-hidden="true"
            />
          </div>
        </div>
      )}

      {/* Recommendation reason */}
      <div className="recommendation-card__reason">
        <span className="recommendation-card__reason-icon" aria-hidden="true">
          💡
        </span>
        <p className="recommendation-card__reason-text">{recommendation.reason}</p>
      </div>

      {/* Module number (if available) */}
      {recommendation.module_number && (
        <div className="recommendation-card__meta" aria-label="Module information">
          <span className="recommendation-card__module">
            Module {recommendation.module_number}
          </span>
        </div>
      )}
    </div>
  );
}
