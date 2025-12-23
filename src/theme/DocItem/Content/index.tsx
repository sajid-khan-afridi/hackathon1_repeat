import React, { type ReactNode, useEffect } from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type { WrapperProps } from '@docusaurus/types';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import ProgressTracker from '@site/src/components/ProgressTracker';
import useChapterProgress from '@site/src/hooks/useChapterProgress';
import { useAuth } from '@site/src/hooks/useAuth';

type Props = WrapperProps<typeof ContentType>;

/**
 * Wrapper for DocItem/Content to add chapter progress tracking.
 * Automatically tracks when users start reading, displays progress, and provides bookmark functionality.
 */
export default function ContentWrapper(props: Props): ReactNode {
  const location = useLocation();
  const { siteConfig } = useDocusaurusContext();
  const { isAuthenticated } = useAuth();

  // Extract chapter ID from URL path (e.g., /docs/module-1/intro → "module-1/intro")
  const chapterId = location.pathname
    .replace(/^\/docs\//, '') // Remove /docs/ prefix
    .replace(/\/$/, ''); // Remove trailing slash

  // Only enable progress tracking for authenticated users on /docs/ pages
  const isDocsPage = location.pathname.startsWith('/docs/') && chapterId !== '';
  const shouldTrack = isAuthenticated && isDocsPage;

  // Use progress tracking hook
  const {
    progress,
    isLoading,
    timeSpent,
    scrollDepth,
    markCompleted,
    toggleBookmark,
    isBookmarked,
    shouldAutoComplete,
  } = useChapterProgress({
    chapterId,
    autoMarkStarted: shouldTrack, // Auto-mark as started when page loads
    autoMarkCompleted: false, // Don't auto-complete (let user manually mark)
  });

  // Auto-suggest completion when thresholds are met
  useEffect(() => {
    if (shouldAutoComplete && progress?.status !== 'completed') {
      // Optional: Show a toast/notification suggesting to mark as complete
      console.log('Chapter completion threshold reached! Consider marking as complete.');
    }
  }, [shouldAutoComplete, progress?.status]);

  return (
    <>
      {/* Progress Tracker - Show above content for authenticated users on docs pages */}
      {shouldTrack && (
        <div className="chapter-progress-header" style={{ marginBottom: '1.5rem' }}>
          <ProgressTracker
            progress={progress}
            isLoading={isLoading}
            timeSpent={timeSpent}
            scrollDepth={scrollDepth}
            showStats={true}
          />

          {/* Action Buttons */}
          <div className="chapter-actions" style={{
            display: 'flex',
            gap: '0.5rem',
            marginTop: '1rem',
            flexWrap: 'wrap',
          }}>
            {/* Bookmark Button */}
            <button
              onClick={toggleBookmark}
              className="button button--secondary button--sm"
              aria-label={isBookmarked ? 'Remove bookmark' : 'Add bookmark'}
              title={isBookmarked ? 'Remove bookmark' : 'Add bookmark'}
            >
              {isBookmarked ? '🔖 Bookmarked' : '🔖 Bookmark'}
            </button>

            {/* Mark Complete Button (only show if not already completed) */}
            {progress?.status !== 'completed' && (
              <button
                onClick={markCompleted}
                className={`button button--sm ${
                  shouldAutoComplete ? 'button--primary' : 'button--outline button--primary'
                }`}
                aria-label="Mark chapter as completed"
                title={
                  shouldAutoComplete
                    ? 'You have met the completion criteria!'
                    : 'Mark this chapter as completed'
                }
              >
                ✓ {shouldAutoComplete ? 'Mark as Complete (Ready!)' : 'Mark as Complete'}
              </button>
            )}

            {/* Completion Badge (if already completed) */}
            {progress?.status === 'completed' && (
              <span
                className="badge badge--success"
                style={{ padding: '0.5rem 1rem', fontSize: '0.875rem' }}
                aria-label="Chapter completed"
              >
                ✓ Completed
              </span>
            )}
          </div>
        </div>
      )}

      {/* Original Doc Content */}
      <Content {...props} />
    </>
  );
}

