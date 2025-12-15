import React from 'react';
import clsx from 'clsx';
import { ConfidenceIndicatorProps } from './types';

export const ConfidenceIndicator: React.FC<ConfidenceIndicatorProps> = ({
  confidence,
  showLabel = true,
  size = 'medium',
}) => {
  // Convert confidence to percentage
  const percentage = Math.round(confidence * 100);

  // Determine color and label based on confidence level
  const getConfidenceLevel = () => {
    if (confidence >= 0.7) {
      return {
        label: 'High confidence',
        color: 'success',
        description: 'Very confident in this answer',
      };
    } else if (confidence >= 0.3) {
      return {
        label: 'Medium confidence',
        color: 'warning',
        description: 'Moderately confident in this answer',
      };
    } else {
      return {
        label: 'Low confidence',
        color: 'danger',
        description: 'Not very confident in this answer',
      };
    }
  };

  const level = getConfidenceLevel();

  // Size configurations
  const sizeConfig = {
    small: {
      barHeight: 4,
      fontSize: '0.75rem',
      labelPadding: '0.25rem 0.5rem',
    },
    medium: {
      barHeight: 6,
      fontSize: '0.875rem',
      labelPadding: '0.375rem 0.625rem',
    },
    large: {
      barHeight: 8,
      fontSize: '1rem',
      labelPadding: '0.5rem 0.75rem',
    },
  };

  const config = sizeConfig[size];

  return (
    <div
      className={clsx('confidence-indicator', `confidence-${level.color}`, size)}
      title={level.description}
      aria-label={`Confidence: ${percentage}% - ${level.label}`}
    >
      {showLabel && <span className="confidence-label">{percentage}%</span>}
      <div className="confidence-bar-container">
        <div
          className="confidence-bar"
          style={{ width: `${percentage}%` }}
          role="progressbar"
          aria-valuenow={percentage}
          aria-valuemin={0}
          aria-valuemax={100}
          aria-label={`Confidence level: ${percentage}%`}
        />
      </div>

      <style jsx>{`
        .confidence-indicator {
          display: flex;
          align-items: center;
          gap: 0.5rem;
        }

        .confidence-label {
          font-size: ${config.fontSize};
          font-weight: 600;
          padding: ${config.labelPadding};
          border-radius: 16px;
          background-color: var(--ifm-color-emphasis-100);
          color: var(--ifm-color-emphasis-800);
          white-space: nowrap;
        }

        .confidence-bar-container {
          width: 60px;
          height: ${config.barHeight}px;
          background-color: var(--ifm-color-emphasis-200);
          border-radius: ${config.barHeight / 2}px;
          overflow: hidden;
          position: relative;
        }

        .confidence-bar {
          height: 100%;
          transition: width 0.3s ease;
          border-radius: ${config.barHeight / 2}px;
          position: absolute;
          left: 0;
          top: 0;
        }

        /* Confidence level colors */
        .confidence-success .confidence-label {
          background-color: var(--ifm-color-success-contrast-background);
          color: var(--ifm-color-success-contrast-foreground);
        }

        .confidence-success .confidence-bar {
          background-color: var(--ifm-color-success);
        }

        .confidence-warning .confidence-label {
          background-color: var(--ifm-color-warning-contrast-background);
          color: var(--ifm-color-warning-contrast-foreground);
        }

        .confidence-warning .confidence-bar {
          background-color: var(--ifm-color-warning);
        }

        .confidence-danger .confidence-label {
          background-color: var(--ifm-color-danger-contrast-background);
          color: var(--ifm-color-danger-contrast-foreground);
        }

        .confidence-danger .confidence-bar {
          background-color: var(--ifm-color-danger);
        }

        /* Size variations */
        .small .confidence-bar-container {
          width: 40px;
        }

        .large .confidence-bar-container {
          width: 80px;
        }

        /* Animation on load */
        @keyframes fillBar {
          from {
            width: 0;
          }
        }

        .confidence-bar {
          animation: fillBar 0.5s ease-out;
        }

        /* Dark mode adjustments */
        [data-theme='dark'] .confidence-label {
          background-color: var(--ifm-color-emphasis-200);
        }

        [data-theme='dark'] .confidence-bar-container {
          background-color: var(--ifm-color-emphasis-300);
        }

        /* High contrast mode support */
        @media (prefers-contrast: high) {
          .confidence-bar {
            border: 1px solid var(--ifm-color-emphasis-800);
          }

          .confidence-bar-container {
            border: 1px solid var(--ifm-color-emphasis-600);
          }
        }

        /* Reduced motion support */
        @media (prefers-reduced-motion: reduce) {
          .confidence-bar {
            animation: none;
            transition: none;
          }
        }
      `}</style>
    </div>
  );
};
