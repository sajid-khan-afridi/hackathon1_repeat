/**
 * ProgressTracker component for displaying chapter progress status.
 * Shows started/completed/bookmarked states with visual indicators.
 */

import React from 'react';
import type { ChapterProgressResponse } from '../types/progress';

interface ProgressTrackerProps {
  /** Current progress data */
  progress: ChapterProgressResponse | null;

  /** Whether data is loading */
  isLoading?: boolean;

  /** Time spent on chapter (in milliseconds) */
  timeSpent?: number;

  /** Scroll depth (0-1) */
  scrollDepth?: number;

  /** Whether to show detailed stats */
  showStats?: boolean;

  /** Custom CSS class */
  className?: string;
}

/**
 * Format milliseconds to human-readable time.
 */
function formatTime(ms: number): string {
  const totalSeconds = Math.floor(ms / 1000);
  const hours = Math.floor(totalSeconds / 3600);
  const minutes = Math.floor((totalSeconds % 3600) / 60);
  const seconds = totalSeconds % 60;

  if (hours > 0) {
    return `${hours}h ${minutes}m`;
  } else if (minutes > 0) {
    return `${minutes}m ${seconds}s`;
  } else {
    return `${seconds}s`;
  }
}

/**
 * ProgressTracker component displays chapter progress with visual indicators.
 *
 * @example
 * ```tsx
 * <ProgressTracker
 *   progress={progress}
 *   timeSpent={timeSpent}
 *   scrollDepth={scrollDepth}
 *   showStats={true}
 * />
 * ```
 */
export default function ProgressTracker({
  progress,
  isLoading = false,
  timeSpent = 0,
  scrollDepth = 0,
  showStats = true,
  className = '',
}: ProgressTrackerProps): React.ReactElement {
  if (isLoading) {
    return (
      <div className={`progress-tracker progress-tracker--loading ${className}`}>
        <span className="progress-tracker__spinner" aria-live="polite">
          Loading progress...
        </span>
      </div>
    );
  }

  if (!progress) {
    return (
      <div className={`progress-tracker progress-tracker--not-started ${className}`}>
        <span className="progress-tracker__status" aria-label="Chapter not started">
          📖 Not started
        </span>
      </div>
    );
  }

  const isCompleted = progress.status === 'completed';
  const isBookmarked = progress.is_bookmarked;

  return (
    <div
      className={`progress-tracker progress-tracker--${progress.status} ${className}`}
      role="status"
      aria-live="polite"
    >
      {/* Status indicator */}
      <div className="progress-tracker__status-row">
        <span
          className={`progress-tracker__status progress-tracker__status--${progress.status}`}
          aria-label={`Chapter status: ${progress.status}`}
        >
          {isCompleted ? '✓ Completed' : '▶ In Progress'}
        </span>

        {isBookmarked && (
          <span
            className="progress-tracker__bookmark"
            aria-label="Chapter is bookmarked"
            title="Bookmarked"
          >
            🔖
          </span>
        )}
      </div>

      {/* Stats (optional) */}
      {showStats && (
        <div className="progress-tracker__stats">
          {timeSpent > 0 && (
            <div className="progress-tracker__stat" aria-label={`Time spent: ${formatTime(timeSpent)}`}>
              <span className="progress-tracker__stat-icon" aria-hidden="true">⏱</span>
              <span className="progress-tracker__stat-value">{formatTime(timeSpent)}</span>
            </div>
          )}

          {scrollDepth > 0 && (
            <div
              className="progress-tracker__stat"
              aria-label={`Scroll progress: ${Math.floor(scrollDepth * 100)}%`}
            >
              <span className="progress-tracker__stat-icon" aria-hidden="true">📊</span>
              <span className="progress-tracker__stat-value">{Math.floor(scrollDepth * 100)}%</span>
            </div>
          )}

          {progress.started_at && (
            <div
              className="progress-tracker__stat"
              aria-label={`Started: ${new Date(progress.started_at).toLocaleDateString()}`}
            >
              <span className="progress-tracker__stat-icon" aria-hidden="true">📅</span>
              <span className="progress-tracker__stat-value">
                {new Date(progress.started_at).toLocaleDateString()}
              </span>
            </div>
          )}

          {progress.completed_at && (
            <div
              className="progress-tracker__stat"
              aria-label={`Completed: ${new Date(progress.completed_at).toLocaleDateString()}`}
            >
              <span className="progress-tracker__stat-icon" aria-hidden="true">🎉</span>
              <span className="progress-tracker__stat-value">
                {new Date(progress.completed_at).toLocaleDateString()}
              </span>
            </div>
          )}
        </div>
      )}

      {/* Inline styles (can be moved to CSS file) */}
      <style jsx>{`
        .progress-tracker {
          padding: 1rem;
          border-radius: 0.5rem;
          background-color: var(--ifm-background-surface-color);
          border: 1px solid var(--ifm-color-emphasis-300);
          margin-bottom: 1rem;
        }

        .progress-tracker--loading {
          text-align: center;
          color: var(--ifm-color-emphasis-600);
        }

        .progress-tracker__spinner {
          display: inline-block;
          animation: pulse 1.5s ease-in-out infinite;
        }

        @keyframes pulse {
          0%, 100% { opacity: 1; }
          50% { opacity: 0.5; }
        }

        .progress-tracker__status-row {
          display: flex;
          align-items: center;
          justify-content: space-between;
          margin-bottom: 0.5rem;
        }

        .progress-tracker__status {
          font-weight: 600;
          font-size: 1rem;
        }

        .progress-tracker__status--started {
          color: var(--ifm-color-primary);
        }

        .progress-tracker__status--completed {
          color: var(--ifm-color-success);
        }

        .progress-tracker__bookmark {
          font-size: 1.25rem;
          cursor: default;
        }

        .progress-tracker__stats {
          display: flex;
          flex-wrap: wrap;
          gap: 1rem;
          font-size: 0.875rem;
          color: var(--ifm-color-emphasis-700);
        }

        .progress-tracker__stat {
          display: flex;
          align-items: center;
          gap: 0.25rem;
        }

        .progress-tracker__stat-icon {
          font-size: 1rem;
        }

        .progress-tracker__stat-value {
          font-weight: 500;
        }

        @media (max-width: 768px) {
          .progress-tracker__stats {
            flex-direction: column;
            gap: 0.5rem;
          }
        }
      `}</style>
    </div>
  );
}
