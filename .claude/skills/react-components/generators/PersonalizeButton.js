const BaseGenerator = require('./BaseGenerator');

class PersonalizeButtonGenerator extends BaseGenerator {
  async generate({ props_schema, styling, accessibility_level }) {
    const componentName = 'PersonalizeButton';

    // Generate interfaces
    const interfaces = this.generateInterfaces(componentName, props_schema);

    // Generate component code
    const code = this.generateComponentCode(componentName, props_schema, styling, accessibility_level);

    // Generate styles
    const styles = this.generatePersonalizeButtonStyles(styling);

    // Generate tests
    const test = this.generateTest(componentName);

    // Generate Storybook story
    const story = this.generateStory(componentName);

    return {
      code: interfaces + '\n\n' + code,
      styles,
      test,
      story
    };
  }

  generateComponentCode(componentName, customProps, styling, accessibility) {
    const imports = this.generateImports({
      hasState: true,
      hasCallback: true,
      styling
    });

    const accessibilityAttrs = this.generateAccessibilityAttributes(accessibility);

    return `${imports}
const ${componentName}: React.FC<${componentName}Props> = ({
  chapterId,
  onPersonalize,
  disabled = false,
  userLevel = 'intermediate',
  ...props
}) => {
  const [state, setState] = useState<'default' | 'loading' | 'success' | 'error'>('default');
  const [error, setError] = useState<string | null>(null);

  const handleClick = useCallback(async () => {
    if (disabled || state === 'loading') return;

    setState('loading');
    setError(null);

    try {
      const response = await fetch('/api/personalize', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          chapterId,
          userLevel,
          context: document.querySelector('.chapter-content')?.textContent || ''
        })
      });

      if (!response.ok) {
        throw new Error('Failed to personalize content');
      }

      const { adaptedContent } = await response.json();
      onPersonalize(adaptedContent);
      setState('success');

      // Reset state after 3 seconds
      setTimeout(() => setState('default'), 3000);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'An error occurred';
      setError(errorMessage);
      setState('error');

      // Reset state after 3 seconds
      setTimeout(() => setState('default'), 3000);
    }
  }, [chapterId, userLevel, onPersonalize, disabled, state]);

  const getLevelColor = (level: string) => {
    const colors = {
      beginner: '#10b981',  // green
      intermediate: '#3b82f6',  // blue
      advanced: '#8b5cf6'  // purple
    };
    return colors[level as keyof typeof colors] || colors.intermediate;
  };

  const getButtonContent = () => {
    switch (state) {
      case 'loading':
        return (
          <>
            <span className={styles.spinner} aria-hidden="true"></span>
            <span>Adapting...</span>
          </>
        );
      case 'success':
        return (
          <>
            <span className={styles.checkmark} aria-hidden="true">✓</span>
            <span>Personalized!</span>
          </>
        );
      case 'error':
        return (
          <>
            <span className={styles.errorIcon} aria-hidden="true">!</span>
            <span>Try again</span>
          </>
        );
      default:
        return (
          <>
            <span
              className={styles.levelBadge}
              style={{ backgroundColor: getLevelColor(userLevel) }}
              title={\`Current level: \${userLevel}\`}
            >
              {userLevel.charAt(0).toUpperCase()}
            </span>
            <span>Personalize for my level</span>
          </>
        );
    }
  };

  const getButtonClassName = () => {
    const baseClasses = styles.button;
    const stateClasses = styles[state];
    const disabledClasses = disabled ? styles.disabled : '';

    return \`\${baseClasses} \${stateClasses} \${disabledClasses}\`.trim();
  };

  return (
    <div className={styles.container}>
      <button
        className={getButtonClassName()}
        onClick={handleClick}
        disabled={disabled || state === 'loading'}
        title={disabled ? 'Please sign in to personalize content' : 'Adapt content to your experience level'}
        aria-label={disabled
          ? 'Sign in required to personalize content'
          : state === 'loading'
            ? 'Personalizing content...'
            : state === 'success'
              ? 'Content personalized successfully'
              : state === 'error'
                ? 'Personalization failed, try again'
                : \`Personalize content for \${userLevel} level\`
        }
        aria-busy={state === 'loading'}
        aria-live="polite"
        aria-atomic="true"
        ${accessibilityAttrs.join(' ')}
      >
        {getButtonContent()}
      </button>

      {error && (
        <div className={styles.errorMessage} role="alert">
          {error}
        </div>
      )}

      <div className={styles.tooltip}>
        <div className={styles.tooltipContent}>
          <h4>Personalization</h4>
          <p>Adapts the chapter content to match your experience level:</p>
          <ul>
            <li><strong>Beginner:</strong> More explanations and simpler examples</li>
            <li><strong>Intermediate:</strong> Balanced approach with some depth</li>
            <li><strong>Advanced:</strong> Technical details and complex concepts</li>
          </ul>
        </div>
      </div>
    </div>
  );
};

export default ${componentName};`;
  }

  generatePersonalizeButtonStyles(styling) {
    if (styling !== 'css_modules') return '';

    return `
.container {
  position: relative;
  display: inline-block;
}

.button {
  display: inline-flex;
  align-items: center;
  gap: 8px;
  padding: 10px 20px;
  background: #0066cc;
  color: white;
  border: none;
  border-radius: 8px;
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s ease;
  min-height: 44px;
  min-width: 44px;
  white-space: nowrap;
  position: relative;
  overflow: hidden;
}

.button:hover:not(.disabled):not(.loading) {
  background: #0052a3;
  transform: translateY(-1px);
  box-shadow: 0 4px 12px rgba(0, 102, 204, 0.3);
}

.button:focus {
  outline: 2px solid #0066cc;
  outline-offset: 2px;
}

.button:active:not(.disabled) {
  transform: translateY(0);
}

.button.disabled {
  background: #9ca3af;
  cursor: not-allowed;
  opacity: 0.6;
}

.button.loading {
  cursor: wait;
  background: #0066cc;
}

.button.success {
  background: #10b981;
  animation: pulse 0.5s ease;
}

.button.error {
  background: #ef4444;
  animation: shake 0.5s ease;
}

@keyframes pulse {
  0% { transform: scale(1); }
  50% { transform: scale(1.05); }
  100% { transform: scale(1); }
}

@keyframes shake {
  0%, 100% { transform: translateX(0); }
  25% { transform: translateX(-4px); }
  75% { transform: translateX(4px); }
}

.levelBadge {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 24px;
  height: 24px;
  border-radius: 50%;
  font-size: 12px;
  font-weight: 600;
  background: #3b82f6;
  color: white;
  border: 2px solid rgba(255, 255, 255, 0.3);
}

.spinner {
  width: 16px;
  height: 16px;
  border: 2px solid rgba(255, 255, 255, 0.3);
  border-top-color: white;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  to { transform: rotate(360deg); }
}

.checkmark {
  color: white;
  font-weight: bold;
}

.errorIcon {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 16px;
  height: 16px;
  background: white;
  color: #ef4444;
  border-radius: 50%;
  font-size: 12px;
  font-weight: bold;
}

.errorMessage {
  position: absolute;
  top: calc(100% + 8px);
  left: 0;
  right: 0;
  padding: 8px 12px;
  background: #fef2f2;
  border: 1px solid #fecaca;
  border-radius: 6px;
  color: #dc2626;
  font-size: 12px;
  z-index: 10;
  animation: slideDown 0.2s ease;
}

@keyframes slideDown {
  from {
    opacity: 0;
    transform: translateY(-4px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

/* Tooltip */
.tooltip {
  position: absolute;
  bottom: calc(100% + 10px);
  left: 50%;
  transform: translateX(-50%);
  z-index: 1000;
  opacity: 0;
  visibility: hidden;
  transition: all 0.2s ease;
}

.container:hover .tooltip {
  opacity: 1;
  visibility: visible;
}

.tooltipContent {
  background: #1f2937;
  color: white;
  padding: 12px 16px;
  border-radius: 8px;
  font-size: 12px;
  line-height: 1.4;
  max-width: 280px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
}

.tooltipContent h4 {
  margin: 0 0 8px 0;
  font-size: 14px;
  font-weight: 600;
}

.tooltipContent p {
  margin: 0 0 8px 0;
}

.tooltipContent ul {
  margin: 0;
  padding-left: 16px;
}

.tooltipContent li {
  margin-bottom: 4px;
}

.tooltipContent li:last-child {
  margin-bottom: 0;
}

/* Dark theme */
[data-theme='dark'] .button {
  background: #1e40af;
}

[data-theme='dark'] .button:hover:not(.disabled):not(.loading) {
  background: #1e3a8a;
}

[data-theme='dark'] .errorMessage {
  background: #7f1d1d;
  border-color: #991b1b;
  color: #fca5a5;
}

/* Mobile responsive */
@media (max-width: 640px) {
  .button {
    font-size: 13px;
    padding: 8px 16px;
  }

  .tooltipContent {
    max-width: 240px;
  }
}`;
  }

  generateTest(componentName) {
    return `import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import '@testing-library/jest-dom';
import ${componentName} from './index';

// Mock fetch
global.fetch = jest.fn();

const mockFetch = fetch as jest.MockedFunction<typeof fetch>;

describe('${componentName}', () => {
  const mockOnPersonalize = jest.fn();

  beforeEach(() => {
    mockFetch.mockClear();
    mockOnPersonalize.mockClear();
  });

  it('renders the personalize button', () => {
    render(
      <${componentName}
        chapterId="chapter-1"
        onPersonalize={mockOnPersonalize}
      />
    );

    expect(screen.getByRole('button', { name: /personalize content for intermediate level/i })).toBeInTheDocument();
    expect(screen.getByText('Personalize for my level')).toBeInTheDocument();
  });

  it('shows user level badge', () => {
    render(
      <${componentName}
        chapterId="chapter-1"
        onPersonalize={mockOnPersonalize}
        userLevel="beginner"
      />
    );

    const badge = screen.getByTitle('Current level: beginner');
    expect(badge).toHaveTextContent('B');
  });

  it('is disabled when disabled prop is true', () => {
    render(
      <${componentName}
        chapterId="chapter-1"
        onPersonalize={mockOnPersonalize}
        disabled={true}
      />
    );

    const button = screen.getByRole('button');
    expect(button).toBeDisabled();
    expect(button).toHaveAttribute('title', 'Please sign in to personalize content');
  });

  it('shows loading state when clicked', async () => {
    const user = userEvent.setup();
    mockFetch.mockImplementationOnce(() => new Promise(resolve => setTimeout(resolve, 100)));

    render(
      <${componentName}
        chapterId="chapter-1"
        onPersonalize={mockOnPersonalize}
      />
    );

    const button = screen.getByRole('button');
    await user.click(button);

    expect(screen.getByText('Adapting...')).toBeInTheDocument();
    expect(button).toHaveAttribute('aria-busy', 'true');
  });

  it('calls onPersonalize with adapted content on success', async () => {
    const user = userEvent.setup();
    const mockContent = 'This is personalized content for beginners';
    mockFetch.mockResolvedValueOnce({
      ok: true,
      json: async () => ({ adaptedContent: mockContent })
    });

    render(
      <${componentName}
        chapterId="chapter-1"
        onPersonalize={mockOnPersonalize}
        userLevel="beginner"
      />
    );

    const button = screen.getByRole('button');
    await user.click(button);

    await waitFor(() => {
      expect(mockFetch).toHaveBeenCalledWith('/api/personalize', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: expect.stringContaining('"userLevel":"beginner"')
      });
    });

    await waitFor(() => {
      expect(mockOnPersonalize).toHaveBeenCalledWith(mockContent);
    });

    await waitFor(() => {
      expect(screen.getByText('Personalized!')).toBeInTheDocument();
    });
  });

  it('shows error state on failure', async () => {
    const user = userEvent.setup();
    mockFetch.mockRejectedValueOnce(new Error('API error'));

    render(
      <${componentName}
        chapterId="chapter-1"
        onPersonalize={mockOnPersonalize}
      />
    );

    const button = screen.getByRole('button');
    await user.click(button);

    await waitFor(() => {
      expect(screen.getByText('Try again')).toBeInTheDocument();
      expect(screen.getByText('API error')).toBeInTheDocument();
    });
  });

  it('resets to default state after success', async () => {
    const user = userEvent.setup();
    mockFetch.mockResolvedValueOnce({
      ok: true,
      json: async () => ({ adaptedContent: 'Content' })
    });

    jest.useFakeTimers();

    render(
      <${componentName}
        chapterId="chapter-1"
        onPersonalize={mockOnPersonalize}
      />
    );

    const button = screen.getByRole('button');
    await user.click(button);

    // Wait for success state
    await waitFor(() => {
      expect(screen.getByText('Personalized!')).toBeInTheDocument();
    });

    // Fast-forward 3 seconds
    jest.advanceTimersByTime(3000);

    await waitFor(() => {
      expect(screen.getByText('Personalize for my level')).toBeInTheDocument();
    });

    jest.useRealTimers();
  });

  it('shows tooltip on hover', async () => {
    render(
      <${componentName}
        chapterId="chapter-1"
        onPersonalize={mockOnPersonalize}
      />
    );

    const button = screen.getByRole('button');
    await fireEvent.mouseEnter(button);

    expect(screen.getByText('Personalization')).toBeInTheDocument();
    expect(screen.getByText(/adapts the chapter content/i)).toBeInTheDocument();
  });
});`;
  }

  generateStory(componentName) {
    return `import type { Meta, StoryObj } from '@storybook/react';
import { ${componentName} } from './index';

const meta: Meta<typeof ${componentName}> = {
  title: 'Components/${componentName}',
  component: ${componentName},
  parameters: {
    layout: 'centered',
  },
  tags: ['autodocs'],
  argTypes: {
    userLevel: {
      control: 'select',
      options: ['beginner', 'intermediate', 'advanced'],
    },
    disabled: {
      control: 'boolean',
    },
  },
};

export default meta;
type Story = StoryObj<typeof meta>;

export const Default: Story = {
  args: {
    chapterId: 'chapter-1',
    onPersonalize: (content) => console.log('Personalized:', content),
  },
};

export const Beginner: Story = {
  args: {
    chapterId: 'chapter-1',
    onPersonalize: (content) => console.log('Personalized:', content),
    userLevel: 'beginner',
  },
};

export const Advanced: Story = {
  args: {
    chapterId: 'chapter-1',
    onPersonalize: (content) => console.log('Personalized:', content),
    userLevel: 'advanced',
  },
};

export const Disabled: Story = {
  args: {
    chapterId: 'chapter-1',
    onPersonalize: (content) => console.log('Personalized:', content),
    disabled: true,
  },
};

export const WithCallback: Story = {
  args: {
    chapterId: 'chapter-2',
    onPersonalize: (content) => {
      alert('Content personalized! Check console for details.');
      console.log('Received personalized content:', content);
    },
    userLevel: 'intermediate',
  },
};`;
  }
}

module.exports = PersonalizeButtonGenerator;