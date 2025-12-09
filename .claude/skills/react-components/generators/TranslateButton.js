const BaseGenerator = require('./BaseGenerator');

class TranslateButtonGenerator extends BaseGenerator {
  async generate({ props_schema, styling, accessibility_level }) {
    const componentName = 'TranslateButton';

    // Generate interfaces
    const interfaces = this.generateInterfaces(componentName, props_schema);

    // Generate component code
    const code = this.generateComponentCode(componentName, props_schema, styling, accessibility_level);

    // Generate styles
    const styles = this.generateTranslateButtonStyles(styling);

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
  originalContent,
  onTranslate,
  targetLanguage = 'ur',
  ...props
}) => {
  const [state, setState] = useState<'default' | 'loading' | 'translated' | 'error'>('default');
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [cache] = useState(new Map<string, string>());

  const handleTranslate = useCallback(async () => {
    if (state === 'loading') return;

    // Check cache first
    const cacheKey = \`\${chapterId}-\${targetLanguage}\`;
    if (cache.has(cacheKey)) {
      const cachedContent = cache.get(cacheKey)!;
      setTranslatedContent(cachedContent);
      onTranslate(cachedContent);
      setState('translated');
      return;
    }

    setState('loading');
    setError(null);

    try {
      const response = await fetch('/api/translate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          content: originalContent,
          targetLanguage,
          preserveCodeBlocks: true
        })
      });

      if (!response.ok) {
        throw new Error('Translation failed');
      }

      const { translatedText } = await response.json();

      // Cache the result
      cache.set(cacheKey, translatedText);
      setTranslatedContent(translatedText);
      onTranslate(translatedText);
      setState('translated');
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'An error occurred';
      setError(errorMessage);
      setState('error');
    }
  }, [chapterId, originalContent, targetLanguage, onTranslate, state, cache]);

  const handleToggle = () => {
    if (state === 'translated') {
      // Show original
      onTranslate(originalContent);
      setState('default');
    } else {
      // Translate
      handleTranslate();
    }
  };

  const getButtonText = () => {
    switch (state) {
      case 'loading':
        return 'Translating...';
      case 'translated':
        return targetLanguage === 'ur' ? 'Read in English' : 'Read in Urdu';
      case 'error':
        return 'Try Again';
      default:
        return targetLanguage === 'ur' ? 'اردو میں پڑھیں' : 'Read in Urdu';
    }
  };

  const getButtonClassName = () => {
    const baseClasses = styles.button;
    const stateClasses = styles[state];

    return \`\${baseClasses} \${stateClasses}\`.trim();
  };

  const isRTL = targetLanguage === 'ur' && state === 'translated';

  return (
    <div className={styles.container} dir={isRTL ? 'rtl' : 'ltr'}>
      <button
        className={getButtonClassName()}
        onClick={handleToggle}
        disabled={state === 'loading'}
        title={state === 'loading'
          ? 'Translating content...'
          : state === 'translated'
            ? 'Switch to original language'
            : 'Translate content to Urdu'
        }
        aria-label={state === 'loading'
          ? 'Translation in progress'
          : state === 'translated'
            ? 'Show original English content'
            : 'Translate to Urdu language'
        }
        aria-busy={state === 'loading'}
        aria-live="polite"
        ${accessibilityAttrs.join(' ')}
      >
        {state === 'loading' ? (
          <>
            <span className={styles.spinner} aria-hidden="true"></span>
            <span>{getButtonText()}</span>
          </>
        ) : (
          <span>{getButtonText()}</span>
        )}
      </button>

      {error && (
        <div className={styles.errorMessage} role="alert">
          {error}
        </div>
      )}

      {/* Language indicator when translated */}
      {state === 'translated' && (
        <div className={styles.languageIndicator}>
          <span className={styles.languageFlag}>🇵🇰</span>
          <span>Urdu Translation Active</span>
        </div>
      )}
    </div>
  );
};

export default ${componentName};`;
  }

  generateTranslateButtonStyles(styling) {
    if (styling !== 'css_modules') return '';

    return `
.container {
  position: relative;
  display: inline-block;
}

.button {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  padding: 8px 16px;
  background: #10b981;
  color: white;
  border: none;
  border-radius: 6px;
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s ease;
  min-height: 44px;
  min-width: 44px;
  white-space: nowrap;
  font-family: inherit;
}

.button:hover:not(.loading) {
  background: #059669;
  transform: translateY(-1px);
  box-shadow: 0 4px 12px rgba(16, 185, 129, 0.3);
}

.button:focus {
  outline: 2px solid #10b981;
  outline-offset: 2px;
}

.button:active {
  transform: translateY(0);
}

.button.loading {
  background: #10b981;
  cursor: wait;
}

.button.translated {
  background: #dc2626;
}

.button.translated:hover {
  background: #b91c1c;
}

.button.error {
  background: #f59e0b;
}

.button.error:hover {
  background: #d97706;
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

.languageIndicator {
  display: inline-flex;
  align-items: center;
  gap: 6px;
  margin-left: 12px;
  padding: 4px 8px;
  background: #f0fdf4;
  color: #166534;
  border-radius: 4px;
  font-size: 12px;
  font-weight: 500;
  animation: fadeIn 0.3s ease;
}

.languageFlag {
  font-size: 16px;
}

@keyframes fadeIn {
  from {
    opacity: 0;
    transform: scale(0.9);
  }
  to {
    opacity: 1;
    transform: scale(1);
  }
}

/* RTL Support */
.container[dir='rtl'] {
  text-align: right;
}

.container[dir='rtl'] .languageIndicator {
  margin-left: 0;
  margin-right: 12px;
}

/* Dark theme */
[data-theme='dark'] .button {
  background: #059669;
}

[data-theme='dark'] .button:hover:not(.loading) {
  background: #047857;
}

[data-theme='dark'] .button.translated {
  background: #b91c1c;
}

[data-theme='dark'] .button.translated:hover {
  background: #991b1b;
}

[data-theme='dark'] .errorMessage {
  background: #7f1d1d;
  border-color: #991b1b;
  color: #fca5a5;
}

[data-theme='dark'] .languageIndicator {
  background: #14532d;
  color: #86efac;
}

/* Mobile responsive */
@media (max-width: 640px) {
  .button {
    font-size: 13px;
    padding: 6px 12px;
  }

  .languageIndicator {
    display: none; /* Hide on small screens to save space */
  }
}

/* Print styles */
@media print {
  .container {
    display: none;
  }
}

/* High contrast mode */
@media (prefers-contrast: high) {
  .button {
    border: 2px solid currentColor;
  }
}

/* Reduced motion */
@media (prefers-reduced-motion: reduce) {
  .button,
  .spinner,
  .languageIndicator,
  .errorMessage {
    animation: none;
    transition: none;
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
  const mockOnTranslate = jest.fn();
  const originalContent = 'This is the original content in English.';

  beforeEach(() => {
    mockFetch.mockClear();
    mockOnTranslate.mockClear();
  });

  it('renders the translate button with Urdu text', () => {
    render(
      <${componentName}
        chapterId="chapter-1"
        originalContent={originalContent}
        onTranslate={mockOnTranslate}
      />
    );

    expect(screen.getByRole('button', { name: /translate to urdu language/i })).toBeInTheDocument();
    expect(screen.getByText('اردو میں پڑھیں')).toBeInTheDocument();
  });

  it('starts translation when clicked', async () => {
    const user = userEvent.setup();
    mockFetch.mockImplementationOnce(() => new Promise(resolve => setTimeout(resolve, 100)));

    render(
      <${componentName}
        chapterId="chapter-1"
        originalContent={originalContent}
        onTranslate={mockOnTranslate}
      />
    );

    const button = screen.getByRole('button');
    await user.click(button);

    expect(screen.getByText('Translating...')).toBeInTheDocument();
    expect(button).toHaveAttribute('aria-busy', 'true');
  });

  it('displays translated content on success', async () => {
    const user = userEvent.setup();
    const translatedText = 'یہ اردو میں ترجمہ شدہ مواد ہے۔';
    mockFetch.mockResolvedValueOnce({
      ok: true,
      json: async () => ({ translatedText })
    });

    render(
      <${componentName}
        chapterId="chapter-1"
        originalContent={originalContent}
        onTranslate={mockOnTranslate}
      />
    );

    const button = screen.getByRole('button');
    await user.click(button);

    await waitFor(() => {
      expect(mockFetch).toHaveBeenCalledWith('/api/translate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          content: originalContent,
          targetLanguage: 'ur',
          preserveCodeBlocks: true
        })
      });
    });

    await waitFor(() => {
      expect(mockOnTranslate).toHaveBeenCalledWith(translatedText);
    });

    await waitFor(() => {
      expect(screen.getByText('Read in English')).toBeInTheDocument();
      expect(screen.getByText('Urdu Translation Active')).toBeInTheDocument();
    });
  });

  it('shows original content when clicked again after translation', async () => {
    const user = userEvent.setup();
    const translatedText = 'یہ اردو میں ترجمہ شدہ مواد ہے۔';
    mockFetch.mockResolvedValueOnce({
      ok: true,
      json: async () => ({ translatedText })
    });

    render(
      <${componentName}
        chapterId="chapter-1"
        originalContent={originalContent}
        onTranslate={mockOnTranslate}
      />
    );

    const button = screen.getByRole('button');

    // First click - translate
    await user.click(button);
    await waitFor(() => {
      expect(screen.getByText('Read in English')).toBeInTheDocument();
    });

    // Second click - show original
    await user.click(button);
    await waitFor(() => {
      expect(mockOnTranslate).toHaveBeenCalledWith(originalContent);
    });

    expect(screen.getByText('اردو میں پڑھیں')).toBeInTheDocument();
  });

  it('caches translated content', async () => {
    const user = userEvent.setup();
    const translatedText = 'یہ اردو میں ترجمہ شدہ مواد ہے۔';
    mockFetch.mockResolvedValueOnce({
      ok: true,
      json: async () => ({ translatedText })
    });

    const { rerender } = render(
      <${componentName}
        chapterId="chapter-1"
        originalContent={originalContent}
        onTranslate={mockOnTranslate}
      />
    );

    // First click - makes API call
    const button = screen.getByRole('button');
    await user.click(button);
    await waitFor(() => {
      expect(mockFetch).toHaveBeenCalledTimes(1);
    });

    // Re-render component (simulating navigation back)
    rerender(
      <${componentName}
        chapterId="chapter-1"
        originalContent={originalContent}
        onTranslate={mockOnTranslate}
      />
    );

    // Click again - should use cache, no API call
    await user.click(button);
    await waitFor(() => {
      expect(mockOnTranslate).toHaveBeenCalledWith(translatedText);
    });

    // No additional API calls
    expect(mockFetch).toHaveBeenCalledTimes(1);
  });

  it('shows error state on failure', async () => {
    const user = userEvent.setup();
    mockFetch.mockRejectedValueOnce(new Error('Translation API error'));

    render(
      <${componentName}
        chapterId="chapter-1"
        originalContent={originalContent}
        onTranslate={mockOnTranslate}
      />
    );

    const button = screen.getByRole('button');
    await user.click(button);

    await waitFor(() => {
      expect(screen.getByText('Try Again')).toBeInTheDocument();
      expect(screen.getByText('Translation API error')).toBeInTheDocument();
    });
  });

  it('preserves code blocks during translation', async () => {
    const contentWithCode = \`Here is some code:
\`\`\`javascript
const x = 10;
\`\`\`
And some more text.\`;

    const user = userEvent.setup();
    mockFetch.mockResolvedValueOnce({
      ok: true,
      json: async () => ({ translatedText: 'Translated content' })
    });

    render(
      <${componentName}
        chapterId="chapter-1"
        originalContent={contentWithCode}
        onTranslate={mockOnTranslate}
      />
    );

    await user.click(screen.getByRole('button'));

    await waitFor(() => {
      expect(mockFetch).toHaveBeenCalledWith('/api/translate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          content: contentWithCode,
          targetLanguage: 'ur',
          preserveCodeBlocks: true
        })
      });
    });
  });

  it('has correct RTL direction for Urdu', async () => {
    const user = userEvent.setup();
    const translatedText = 'یہ اردو میں ترجمہ شدہ مواد ہے۔';
    mockFetch.mockResolvedValueOnce({
      ok: true,
      json: async () => ({ translatedText })
    });

    render(
      <${componentName}
        chapterId="chapter-1"
        originalContent={originalContent}
        onTranslate={mockOnTranslate}
      />
    );

    const button = screen.getByRole('button');
    await user.click(button);

    await waitFor(() => {
      expect(button.closest('.container')).toHaveAttribute('dir', 'rtl');
    });
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
    targetLanguage: {
      control: 'select',
      options: ['ur'],
    },
  },
};

export default meta;
type Story = StoryObj<typeof meta>;

const sampleContent = \`# Introduction to React

React is a JavaScript library for building user interfaces. It was developed by Facebook and is now maintained by Meta.

\`\`\`jsx
function Welcome(props) {
  return <h1>Hello, {props.name}</h1>;
}
\`\`\`

This component displays a welcome message with the provided name.\`;

export const Default: Story = {
  args: {
    chapterId: 'react-intro',
    originalContent: sampleContent,
    onTranslate: (content) => console.log('Translated content:', content),
  },
};

export const LongContent: Story = {
  args: {
    chapterId: 'advanced-concepts',
    originalContent: sampleContent.repeat(5),
    onTranslate: (content) => console.log('Translated content:', content),
  },
};

export const WithCodeBlocks: Story = {
  args: {
    chapterId: 'code-examples',
    originalContent: sampleContent,
    onTranslate: (content) => console.log('Translated content:', content),
  },
};

export const WithCallback: Story = {
  args: {
    chapterId: 'interactive-demo',
    originalContent: sampleContent,
    onTranslate: (content) => {
      alert('Content translated! Check console for details.');
      console.log('Received translated content:', content);
    },
  },
};`;
  }
}

module.exports = TranslateButtonGenerator;