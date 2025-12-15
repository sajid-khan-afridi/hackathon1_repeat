import React from 'react';
import clsx from 'clsx';
import { SourceCitationsProps } from './types';

export const SourceCitations: React.FC<SourceCitationsProps> = ({ sources, className }) => {
  const formatCitation = (source: any) => {
    const parts = [`Chapter ${source.chapter}`];
    if (source.section) parts.push(source.section);
    if (source.module) parts.push(`Module ${source.module}`);
    return parts.join(' · ');
  };

  const getRelevanceColor = (score: number) => {
    if (score >= 0.8) return 'high';
    if (score >= 0.6) return 'medium';
    return 'low';
  };

  return (
    <div className={clsx('source-citations', className)}>
      <div className="citations-header">
        <span className="citations-icon">📚</span>
        <span className="citations-title">
          Source{sources.length > 1 ? 's' : ''} ({sources.length})
        </span>
      </div>
      <div className="citations-list">
        {sources.map((source, index) => (
          <div key={index} className="citation-item">
            <div className="citation-content">
              <p className="citation-text">{source.content}</p>
              <div className="citation-meta">
                <span className="citation-location">{formatCitation(source)}</span>
                <span
                  className={clsx('relevance-score', getRelevanceColor(source.relevanceScore))}
                  title={`Relevance: ${Math.round(source.relevanceScore * 100)}%`}
                >
                  {Math.round(source.relevanceScore * 100)}%
                </span>
              </div>
              {source.pageUrl && (
                <a
                  href={source.pageUrl}
                  target="_blank"
                  rel="noopener noreferrer"
                  className="citation-link"
                  aria-label={`Read more in chapter ${source.chapter}`}
                >
                  Read more →
                </a>
              )}
            </div>
          </div>
        ))}
      </div>

      <style jsx>{`
        .source-citations {
          margin-top: 0.75rem;
          padding: 0.75rem;
          background-color: var(--ifm-color-emphasis-100);
          border-radius: 8px;
          border-left: 3px solid var(--ifm-color-primary);
        }

        .citations-header {
          display: flex;
          align-items: center;
          gap: 0.5rem;
          margin-bottom: 0.5rem;
          font-weight: 600;
          color: var(--ifm-color-emphasis-800);
        }

        .citations-icon {
          font-size: 1.1em;
        }

        .citations-title {
          font-size: 0.875rem;
        }

        .citations-list {
          display: flex;
          flex-direction: column;
          gap: 0.75rem;
        }

        .citation-item {
          position: relative;
        }

        .citation-content {
          background-color: var(--ifm-background-color);
          padding: 0.75rem;
          border-radius: 6px;
          border: 1px solid var(--ifm-color-emphasis-200);
        }

        .citation-text {
          margin: 0 0 0.5rem 0;
          font-size: 0.9em;
          line-height: 1.4;
          color: var(--ifm-font-color-base);
          font-style: italic;
        }

        .citation-meta {
          display: flex;
          justify-content: space-between;
          align-items: center;
          font-size: 0.75rem;
          color: var(--ifm-color-emphasis-700);
        }

        .citation-location {
          font-weight: 500;
        }

        .relevance-score {
          padding: 0.125rem 0.375rem;
          border-radius: 12px;
          font-weight: 600;
          font-size: 0.7rem;
          background-color: var(--ifm-color-emphasis-200);
        }

        .relevance-score.high {
          background-color: var(--ifm-color-success-contrast-background);
          color: var(--ifm-color-success-contrast-foreground);
        }

        .relevance-score.medium {
          background-color: var(--ifm-color-warning-contrast-background);
          color: var(--ifm-color-warning-contrast-foreground);
        }

        .relevance-score.low {
          background-color: var(--ifm-color-danger-contrast-background);
          color: var(--ifm-color-danger-contrast-foreground);
        }

        .citation-link {
          display: inline-block;
          margin-top: 0.5rem;
          font-size: 0.875rem;
          color: var(--ifm-color-primary);
          text-decoration: none;
          font-weight: 500;
          transition: opacity 0.2s ease;
        }

        .citation-link:hover {
          opacity: 0.8;
          text-decoration: underline;
        }

        /* Dark mode adjustments */
        [data-theme='dark'] .source-citations {
          background-color: var(--ifm-color-emphasis-200);
        }

        [data-theme='dark'] .citation-content {
          background-color: var(--ifm-background-surface-color);
          border-color: var(--ifm-color-emphasis-300);
        }

        /* Mobile adjustments */
        @media (max-width: 768px) {
          .source-citations {
            padding: 0.625rem;
          }

          .citation-content {
            padding: 0.625rem;
          }

          .citation-text {
            font-size: 0.875em;
          }
        }
      `}</style>
    </div>
  );
};
