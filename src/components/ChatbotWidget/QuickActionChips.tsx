/**
 * QuickActionChips Component
 *
 * Pre-defined conversation starters that help users get started with the chatbot.
 * Features:
 * - Clickable chips that pre-fill the input field
 * - Guided conversation starters
 * - Responsive grid layout
 * - Keyboard accessible
 * - ARIA labels for screen readers
 */

import React from 'react';
import styles from './QuickActionChips.module.css';

interface QuickAction {
  /** Display text for the chip */
  label: string;
  /** Query text to be sent when clicked */
  query: string;
  /** Optional icon (emoji or SVG path data) */
  icon?: string;
}

interface QuickActionChipsProps {
  /** Array of quick actions to display */
  actions?: QuickAction[];
  /** Callback when an action is clicked */
  onActionClick: (query: string) => void;
}

/**
 * Default quick actions for robotics textbook chatbot
 */
const DEFAULT_ACTIONS: QuickAction[] = [
  {
    label: 'Ask a question',
    query: 'What can you help me with?',
    icon: '💬'
  },
  {
    label: 'Find a module',
    query: 'Show me the available modules',
    icon: '📚'
  },
  {
    label: 'ROS 2 help',
    query: 'Explain ROS 2 basics',
    icon: '🤖'
  },
  {
    label: 'Search docs',
    query: 'How do I search the documentation?',
    icon: '🔍'
  }
];

export default function QuickActionChips({
  actions = DEFAULT_ACTIONS,
  onActionClick
}: QuickActionChipsProps): React.ReactElement {
  return (
    <div className={styles.quickActionChips} role="group" aria-label="Quick actions">
      {actions.map((action, index) => (
        <button
          key={index}
          type="button"
          className={styles.chip}
          onClick={() => onActionClick(action.query)}
          aria-label={`Quick action: ${action.label}`}
        >
          {action.icon && (
            <span className={styles.chipIcon} aria-hidden="true">
              {action.icon}
            </span>
          )}
          <span className={styles.chipLabel}>{action.label}</span>
        </button>
      ))}
    </div>
  );
}
