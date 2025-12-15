import React, { useState } from 'react';
import clsx from 'clsx';
import { ModuleFilterProps, FilterParams } from './types';

export const ModuleFilter: React.FC<ModuleFilterProps> = ({ onFilterChange, filters }) => {
  const [isOpen, setIsOpen] = useState(false);

  const modules = Array.from({ length: 10 }, (_, i) => i + 1);
  const difficulties = ['beginner', 'intermediate', 'advanced'] as const;

  const handleModuleChange = (module: number) => {
    const newFilters: FilterParams = {
      ...filters,
      module: filters.module === module ? undefined : module,
    };
    onFilterChange(newFilters);
  };

  const handleDifficultyChange = (difficulty: string) => {
    const newFilters: FilterParams = {
      ...filters,
      difficulty: filters.difficulty === difficulty ? undefined : (difficulty as any),
    };
    onFilterChange(newFilters);
  };

  const clearFilters = () => {
    onFilterChange({});
  };

  const getActiveFiltersCount = () => {
    return Object.values(filters).filter(Boolean).length;
  };

  return (
    <div className="module-filter">
      <button
        className="filter-toggle"
        onClick={() => setIsOpen(!isOpen)}
        aria-expanded={isOpen}
        aria-haspopup="true"
      >
        <span className="filter-icon">🔍</span>
        <span className="filter-text">
          {getActiveFiltersCount() > 0
            ? `${getActiveFiltersCount()} filter${getActiveFiltersCount() > 1 ? 's' : ''} active`
            : 'Filter search'}
        </span>
        <span className={clsx('chevron', { 'is-open': isOpen })}>▼</span>
      </button>

      {isOpen && (
        <div className="filter-dropdown">
          <div className="filter-section">
            <h4>Module</h4>
            <div className="module-grid">
              {modules.map((module) => (
                <button
                  key={module}
                  className={clsx('module-button', {
                    active: filters.module === module,
                  })}
                  onClick={() => handleModuleChange(module)}
                  aria-pressed={filters.module === module}
                >
                  {module}
                </button>
              ))}
            </div>
          </div>

          <div className="filter-section">
            <h4>Difficulty</h4>
            <div className="difficulty-options">
              {difficulties.map((difficulty) => (
                <label
                  key={difficulty}
                  className={clsx('difficulty-option', {
                    active: filters.difficulty === difficulty,
                  })}
                >
                  <input
                    type="radio"
                    name="difficulty"
                    value={difficulty}
                    checked={filters.difficulty === difficulty}
                    onChange={() => handleDifficultyChange(difficulty)}
                  />
                  <span className="difficulty-label">{difficulty}</span>
                </label>
              ))}
            </div>
          </div>

          <div className="filter-actions">
            <button
              className="clear-filters-button"
              onClick={clearFilters}
              disabled={getActiveFiltersCount() === 0}
            >
              Clear All
            </button>
            <button className="apply-filters-button" onClick={() => setIsOpen(false)}>
              Apply Filters
            </button>
          </div>
        </div>
      )}

      <style jsx>{`
        .module-filter {
          position: relative;
        }

        .filter-toggle {
          display: flex;
          align-items: center;
          gap: 0.5rem;
          padding: 0.5rem 1rem;
          background-color: var(--ifm-color-emphasis-100);
          border: 1px solid var(--ifm-color-emphasis-200);
          border-radius: 8px;
          cursor: pointer;
          transition: all 0.2s ease;
          font-size: 0.875rem;
          color: var(--ifm-font-color-base);
        }

        .filter-toggle:hover {
          background-color: var(--ifm-color-emphasis-200);
        }

        .filter-icon {
          font-size: 1.1em;
        }

        .filter-text {
          flex: 1;
          text-align: left;
        }

        .chevron {
          transition: transform 0.2s ease;
          font-size: 0.8em;
        }

        .chevron.is-open {
          transform: rotate(180deg);
        }

        .filter-dropdown {
          position: absolute;
          top: 100%;
          left: 0;
          right: 0;
          z-index: 1000;
          margin-top: 0.5rem;
          padding: 1rem;
          background-color: var(--ifm-background-color);
          border: 1px solid var(--ifm-color-emphasis-200);
          border-radius: 8px;
          box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
        }

        .filter-section {
          margin-bottom: 1rem;
        }

        .filter-section:last-child {
          margin-bottom: 0;
        }

        .filter-section h4 {
          margin: 0 0 0.75rem 0;
          font-size: 0.875rem;
          font-weight: 600;
          color: var(--ifm-color-emphasis-800);
        }

        .module-grid {
          display: grid;
          grid-template-columns: repeat(5, 1fr);
          gap: 0.5rem;
        }

        .module-button {
          padding: 0.5rem;
          background-color: var(--ifm-color-emphasis-100);
          border: 1px solid var(--ifm-color-emphasis-200);
          border-radius: 6px;
          cursor: pointer;
          transition: all 0.2s ease;
          font-size: 0.875rem;
          font-weight: 500;
          color: var(--ifm-font-color-base);
        }

        .module-button:hover {
          background-color: var(--ifm-color-emphasis-200);
        }

        .module-button.active {
          background-color: var(--ifm-color-primary);
          color: white;
          border-color: var(--ifm-color-primary);
        }

        .module-button:focus {
          outline: 2px solid var(--ifm-color-primary);
          outline-offset: 2px;
        }

        .difficulty-options {
          display: flex;
          gap: 1rem;
        }

        .difficulty-option {
          display: flex;
          align-items: center;
          cursor: pointer;
          padding: 0.5rem 1rem;
          border: 1px solid var(--ifm-color-emphasis-200);
          border-radius: 6px;
          transition: all 0.2s ease;
          background-color: var(--ifm-color-emphasis-100);
        }

        .difficulty-option:hover {
          background-color: var(--ifm-color-emphasis-200);
        }

        .difficulty-option.active {
          background-color: var(--ifm-color-primary);
          border-color: var(--ifm-color-primary);
          color: white;
        }

        .difficulty-option input[type='radio'] {
          display: none;
        }

        .difficulty-label {
          font-size: 0.875rem;
          font-weight: 500;
          text-transform: capitalize;
        }

        .filter-actions {
          display: flex;
          justify-content: space-between;
          margin-top: 1.5rem;
          padding-top: 1rem;
          border-top: 1px solid var(--ifm-color-emphasis-200);
        }

        .clear-filters-button,
        .apply-filters-button {
          padding: 0.5rem 1.25rem;
          border-radius: 6px;
          font-size: 0.875rem;
          font-weight: 500;
          cursor: pointer;
          transition: all 0.2s ease;
        }

        .clear-filters-button {
          background: none;
          border: 1px solid var(--ifm-color-emphasis-300);
          color: var(--ifm-color-emphasis-700);
        }

        .clear-filters-button:hover:not(:disabled) {
          border-color: var(--ifm-color-danger);
          color: var(--ifm-color-danger);
          background-color: var(--ifm-color-danger-contrast-background);
        }

        .clear-filters-button:disabled {
          opacity: 0.5;
          cursor: not-allowed;
        }

        .apply-filters-button {
          background-color: var(--ifm-color-primary);
          color: white;
          border: none;
        }

        .apply-filters-button:hover {
          background-color: var(--ifm-color-primary-dark);
        }

        /* Dark mode adjustments */
        [data-theme='dark'] .filter-dropdown {
          box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
        }

        /* Mobile adjustments */
        @media (max-width: 768px) {
          .module-grid {
            grid-template-columns: repeat(5, 1fr);
            gap: 0.375rem;
          }

          .module-button {
            padding: 0.375rem;
            font-size: 0.8rem;
          }

          .difficulty-options {
            flex-direction: column;
            gap: 0.5rem;
          }

          .difficulty-option {
            padding: 0.375rem 0.75rem;
          }

          .filter-actions {
            flex-direction: column;
            gap: 0.5rem;
          }

          .clear-filters-button,
          .apply-filters-button {
            width: 100%;
            padding: 0.625rem;
          }
        }

        /* Keyboard accessibility */
        .filter-toggle:focus,
        .module-button:focus,
        .difficulty-option:focus {
          outline: 2px solid var(--ifm-color-primary);
          outline-offset: 2px;
        }
      `}</style>
    </div>
  );
};
