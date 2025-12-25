/**
 * ModuleFilter Component
 *
 * Dropdown selector for filtering search results by textbook module (1-10).
 * Features:
 * - Module selection with "All Modules" option
 * - Clear filter button
 * - Accessible keyboard navigation
 * - WCAG 2.1 AA compliant
 */

import React from 'react';
import styles from './ChatbotWidget.module.css';

interface ModuleFilterProps {
  /** Currently selected module (null = all modules) */
  selectedModule: number | null;
  /** Callback when module selection changes */
  onModuleChange: (module: number | null) => void;
}

/**
 * Module metadata for dropdown labels
 */
const MODULE_INFO: Record<number, string> = {
  1: 'Module 1: ROS 2 Fundamentals',
  2: 'Module 2: Isaac Sim',
  3: 'Module 3: Applications'
};

export default function ModuleFilter({
  selectedModule,
  onModuleChange
}: ModuleFilterProps): React.ReactElement {
  /**
   * Handle dropdown selection change
   */
  const handleSelectChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const value = event.target.value;
    onModuleChange(value === 'all' ? null : parseInt(value, 10));
  };

  /**
   * Clear filter button handler
   */
  const handleClearFilter = () => {
    onModuleChange(null);
  };

  return (
    <div className={styles.moduleFilter} role="search">
      <label htmlFor="module-filter-select" className={styles.moduleFilterLabel}>
        Filter by module:
      </label>

      <div className={styles.moduleFilterControls}>
        <select
          id="module-filter-select"
          value={selectedModule === null ? 'all' : selectedModule.toString()}
          onChange={handleSelectChange}
          className={styles.moduleFilterSelect}
          aria-label="Select module to filter search results"
        >
          <option value="all">All Modules</option>
          {Object.entries(MODULE_INFO).map(([moduleNum, moduleTitle]) => (
            <option key={moduleNum} value={moduleNum}>
              {moduleTitle}
            </option>
          ))}
        </select>

        {selectedModule !== null && (
          <button
            type="button"
            onClick={handleClearFilter}
            className={styles.clearFilterButton}
            aria-label="Clear module filter"
            title="Clear filter"
          >
            <svg
              width="16"
              height="16"
              viewBox="0 0 16 16"
              fill="currentColor"
              aria-hidden="true"
            >
              <path d="M8 0C3.58 0 0 3.58 0 8s3.58 8 8 8 8-3.58 8-8-3.58-8-8-8zm4 10.59L10.59 12 8 9.41 5.41 12 4 10.59 6.59 8 4 5.41 5.41 4 8 6.59 10.59 4 12 5.41 9.41 8 12 10.59z"/>
            </svg>
          </button>
        )}
      </div>
    </div>
  );
}
