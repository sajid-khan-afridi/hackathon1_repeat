const BaseGenerator = require('./BaseGenerator');

class TextSelectorGenerator extends BaseGenerator {
  async generate({ props_schema, styling, accessibility_level }) {
    const componentName = 'TextSelector';

    // Generate interfaces
    const interfaces = this.generateInterfaces(componentName, props_schema);

    // Generate component code
    const code = this.generateComponentCode(componentName, props_schema, styling, accessibility_level);

    // Generate styles
    const styles = this.generateTextSelectorStyles(styling);

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
      hasEffects: true,
      hasRef: true,
      hasCallback: true,
      styling
    });

    const accessibilityAttrs = this.generateAccessibilityAttributes(accessibility);

    return `${imports}
const ${componentName}: React.FC<${componentName}Props> = ({
  onSelect,
  minLength = 10,
  maxLength = 500,
  ...props
}) => {
  const [selection, setSelection] = useState<{
    text: string;
    rect: DOMRect;
  } | null>(null);
  const [popupPosition, setPopupPosition] = useState<{
    top: number;
    left: number;
  } | null>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const popupRef = useRef<HTMLDivElement>(null);

  const handleSelection = useCallback(() => {
    const selectionObj = window.getSelection();
    if (!selectionObj || selectionObj.isCollapsed) {
      setSelection(null);
      setPopupPosition(null);
      return;
    }

    const selectedText = selectionObj.toString().trim();

    // Check minimum length
    if (selectedText.length < minLength) {
      setSelection(null);
      setPopupPosition(null);
      return;
    }

    // Truncate if too long
    const displayText = selectedText.length > maxLength
      ? selectedText.substring(0, maxLength) + '...'
      : selectedText;

    const range = selectionObj.getRangeAt(0);
    const rect = range.getBoundingClientRect();

    setSelection({
      text: displayText,
      rect
    });

    // Calculate popup position
    if (containerRef.current) {
      const containerRect = containerRef.current.getBoundingClientRect();
      const popupWidth = 200; // Approximate width of popup
      const popupHeight = 40; // Approximate height of popup

      let left = rect.left + rect.width / 2 - popupWidth / 2;
      let top = rect.top - popupHeight - 8;

      // Adjust if popup goes outside container
      if (left < containerRect.left) {
        left = rect.left;
      }
      if (left + popupWidth > containerRect.right) {
        left = rect.right - popupWidth;
      }
      if (top < containerRect.top) {
        top = rect.bottom + 8;
      }

      setPopupPosition({
        top: top - containerRect.top,
        left: left - containerRect.left
      });
    }
  }, [minLength, maxLength]);

  const handleAskAboutThis = useCallback(() => {
    if (!selection) return;

    const actualSelection = window.getSelection();
    const fullText = actualSelection?.toString() || '';

    onSelect(fullText, selection.rect);

    // Clear selection after handling
    window.getSelection()?.removeAllRanges();
    setSelection(null);
    setPopupPosition(null);
  }, [selection, onSelect]);

  const handleClickOutside = useCallback((e: MouseEvent) => {
    if (popupRef.current && !popupRef.current.contains(e.target as Node)) {
      setSelection(null);
      setPopupPosition(null);
    }
  }, []);

  const handleKeyDown = useCallback((e: KeyboardEvent) => {
    if (e.key === 'Escape') {
      setSelection(null);
      setPopupPosition(null);
      window.getSelection()?.removeAllRanges();
    }

    if (e.key === 'Enter' && selection && popupRef.current?.contains(document.activeElement)) {
      e.preventDefault();
      handleAskAboutThis();
    }
  }, [selection, handleAskAboutThis]);

  // Setup event listeners
  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    container.addEventListener('mouseup', handleSelection);
    container.addEventListener('keyup', handleSelection);
    document.addEventListener('mousedown', handleClickOutside);
    document.addEventListener('keydown', handleKeyDown);

    return () => {
      container.removeEventListener('mouseup', handleSelection);
      container.removeEventListener('keyup', handleSelection);
      document.removeEventListener('mousedown', handleClickOutside);
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, [handleSelection, handleClickOutside, handleKeyDown]);

  // Cleanup selection on unmount
  useEffect(() => {
    return () => {
      window.getSelection()?.removeAllRanges();
    };
  }, []);

  return (
    <div
      ref={containerRef}
      className={styles.container}
      role="region"
      aria-label="Text selection area"
      ${accessibilityAttrs.join(' ')}
    >
      {/* This component wraps content to make it selectable */}
      {props.children}

      {/* Popup for asking about selected text */}
      {selection && popupPosition && (
        <div
          ref={popupRef}
          className={styles.popup}
          style={{
            position: 'absolute',
            top: popupPosition.top,
            left: popupPosition.left,
          }}
          role="tooltip"
          aria-live="polite"
        >
          <div className={styles.popupContent}>
            <span className={styles.selectedText}>
              {selection.text}
            </span>
            <button
              className={styles.askButton}
              onClick={handleAskAboutThis}
              aria-label={\`Ask about: \${selection.text}\`}
              title="Ask about this text"
            >
              <svg
                className={styles.askIcon}
                fill="none"
                stroke="currentColor"
                viewBox="0 0 24 24"
                aria-hidden="true"
              >
                <path
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth={2}
                  d="M8.228 9c.549-1.165 2.03-2 3.772-2 2.21 0 4 1.343 4 3 0 1.4-1.278 2.575-3.006 2.907-.542.104-.994.54-.994 1.093m0 3h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z"
                />
              </svg>
              Ask about this
            </button>
          </div>
          <div className={styles.popupArrow} />
        </div>
      )}
    </div>
  );
};

export default ${componentName};`;
  }

  generateTextSelectorStyles(styling) {
    if (styling !== 'css_modules') return '';

    return `
.container {
  position: relative;
  display: contents;
  /* This allows the component to wrap content without affecting layout */
}

/* Highlight selected text */
.container ::selection {
  background: #fef3c7;
  color: #92400e;
}

.container ::-moz-selection {
  background: #fef3c7;
  color: #92400e;
}

/* Popup styles */
.popup {
  background: white;
  border: 1px solid #d1d5db;
  border-radius: 8px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  z-index: 100;
  animation: popupAppear 0.2s ease-out;
  max-width: 300px;
}

@keyframes popupAppear {
  from {
    opacity: 0;
    transform: scale(0.95) translateY(4px);
  }
  to {
    opacity: 1;
    transform: scale(1) translateY(0);
  }
}

.popupContent {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 8px 12px;
}

.selectedText {
  font-size: 14px;
  color: #374151;
  flex: 1;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
  font-style: italic;
}

.askButton {
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 6px 12px;
  background: #3b82f6;
  color: white;
  border: none;
  border-radius: 6px;
  font-size: 13px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s ease;
  white-space: nowrap;
  min-height: 32px;
}

.askButton:hover {
  background: #2563eb;
  transform: translateY(-1px);
  box-shadow: 0 2px 4px rgba(59, 130, 246, 0.3);
}

.askButton:focus {
  outline: 2px solid #3b82f6;
  outline-offset: 2px;
}

.askButton:active {
  transform: translateY(0);
}

.askIcon {
  width: 16px;
  height: 16px;
}

.popupArrow {
  position: absolute;
  bottom: -6px;
  left: 50%;
  transform: translateX(-50%);
  width: 12px;
  height: 12px;
  background: white;
  border-right: 1px solid #d1d5db;
  border-bottom: 1px solid #d1d5db;
  transform: translateX(-50%) rotate(45deg);
}

/* Position adjustments for arrow when popup is at bottom */
.popup.arrowTop .popupArrow {
  top: -6px;
  bottom: auto;
  transform: translateX(-50%) rotate(-135deg);
}

/* Dark theme */
[data-theme='dark'] .popup {
  background: #374151;
  border-color: #4b5563;
}

[data-theme='dark'] .selectedText {
  color: #d1d5db;
}

[data-theme='dark'] .askButton {
  background: #2563eb;
}

[data-theme='dark'] .askButton:hover {
  background: #1d4ed8;
}

[data-theme='dark'] .popupArrow {
  background: #374151;
  border-right-color: #4b5563;
  border-bottom-color: #4b5563;
}

/* High contrast mode */
@media (prefers-contrast: high) {
  .popup {
    border-width: 2px;
    border-color: currentColor;
  }

  .askButton {
    border: 2px solid currentColor;
  }
}

/* Reduced motion */
@media (prefers-reduced-motion: reduce) {
  .popup {
    animation: none;
  }

  .askButton {
    transition: none;
  }
}

/* Mobile responsive */
@media (max-width: 640px) {
  .popup {
    max-width: calc(100vw - 32px);
  }

  .popupContent {
    padding: 6px 10px;
  }

  .selectedText {
    font-size: 13px;
  }

  .askButton {
    font-size: 12px;
    padding: 4px 8px;
  }
}

/* Print styles */
@media print {
  .popup {
    display: none;
  }
}`;
  }

  generateTest(componentName) {
    return `import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import '@testing-library/jest-dom';
import ${componentName} from './index';

// Mock getSelection
const mockGetSelection = jest.fn(() => ({
  toString: () => '',
  isCollapsed: true,
  getRangeAt: jest.fn(() => ({
    getBoundingClientRect: () => ({
      top: 100,
      left: 100,
      right: 200,
      bottom: 120,
      width: 100,
      height: 20
    })
  })),
  removeAllRanges: jest.fn()
}));

Object.defineProperty(window, 'getSelection', {
  value: mockGetSelection,
  writable: true
});

describe('${componentName}', () => {
  const mockOnSelect = jest.fn();

  beforeEach(() => {
    mockOnSelect.mockClear();
    mockGetSelection.mockClear();
  });

  it('renders without crashing', () => {
    render(
      <${componentName} onSelect={mockOnSelect}>
        <p>This is test content</p>
      </${componentName}>
    );
  });

  it('does not show popup for selections shorter than minLength', async () => {
    const mockSelection = {
      toString: () => 'short',
      isCollapsed: false,
      getRangeAt: jest.fn(() => ({
        getBoundingClientRect: () => ({ top: 100, left: 100, right: 150, bottom: 120, width: 50, height: 20 })
      }))
    };

    mockGetSelection.mockReturnValue(mockSelection);

    render(
      <${componentName} onSelect={mockOnSelect} minLength={10}>
        <p>This is test content</p>
      </${componentName}>
    );

    const container = screen.getByRole('region');
    fireEvent.mouseUp(container);

    expect(screen.queryByRole('tooltip')).not.toBeInTheDocument();
  });

  it('shows popup for valid selections', async () => {
    const mockSelection = {
      toString: () => 'This is a valid selection',
      isCollapsed: false,
      getRangeAt: jest.fn(() => ({
        getBoundingClientRect: () => ({ top: 100, left: 100, right: 300, bottom: 120, width: 200, height: 20 })
      }))
    };

    mockGetSelection.mockReturnValue(mockSelection);

    render(
      <${componentName} onSelect={mockOnSelect} minLength={10}>
        <p>This is test content</p>
      </${componentName}>
    );

    const container = screen.getByRole('region');
    fireEvent.mouseUp(container);

    await waitFor(() => {
      expect(screen.getByRole('tooltip')).toBeInTheDocument();
    });

    expect(screen.getByText('This is a valid selection')).toBeInTheDocument();
    expect(screen.getByRole('button', { name: /ask about this/i })).toBeInTheDocument();
  });

  it('truncates selections longer than maxLength', async () => {
    const longText = 'a'.repeat(100);
    const mockSelection = {
      toString: () => longText,
      isCollapsed: false,
      getRangeAt: jest.fn(() => ({
        getBoundingClientRect: () => ({ top: 100, left: 100, right: 300, bottom: 120, width: 200, height: 20 })
      }))
    };

    mockGetSelection.mockReturnValue(mockSelection);

    render(
      <${componentName} onSelect={mockOnSelect} maxLength={50}>
        <p>This is test content</p>
      </${componentName}>
    );

    const container = screen.getByRole('region');
    fireEvent.mouseUp(container);

    await waitFor(() => {
      expect(screen.getByText(\`\${'a'.repeat(47)}...\`)).toBeInTheDocument();
    });
  });

  it('calls onSelect when Ask button is clicked', async () => {
    const selectedText = 'React is a JavaScript library';
    const mockSelection = {
      toString: () => selectedText,
      isCollapsed: false,
      getRangeAt: jest.fn(() => ({
        getBoundingClientRect: () => ({ top: 100, left: 100, right: 400, bottom: 120, width: 300, height: 20 })
      }))
    };

    mockGetSelection.mockReturnValue(mockSelection);

    render(
      <${componentName} onSelect={mockOnSelect}>
        <p>This is test content</p>
      </${componentName}>
    );

    const container = screen.getByRole('region');
    fireEvent.mouseUp(container);

    await waitFor(() => {
      expect(screen.getByRole('button', { name: /ask about this/i })).toBeInTheDocument();
    });

    const askButton = screen.getByRole('button', { name: /ask about this/i });
    await userEvent.click(askButton);

    expect(mockOnSelect).toHaveBeenCalledWith(selectedText, expect.any(DOMRect));
  });

  it('hides popup when clicking outside', async () => {
    const mockSelection = {
      toString: () => 'Valid selection',
      isCollapsed: false,
      getRangeAt: jest.fn(() => ({
        getBoundingClientRect: () => ({ top: 100, left: 100, right: 200, bottom: 120, width: 100, height: 20 })
      }))
    };

    mockGetSelection.mockReturnValue(mockSelection);

    render(
      <${componentName} onSelect={mockOnSelect}>
        <p>This is test content</p>
      </${componentName}>
    );

    const container = screen.getByRole('region');
    fireEvent.mouseUp(container);

    await waitFor(() => {
      expect(screen.getByRole('tooltip')).toBeInTheDocument();
    });

    // Click outside
    fireEvent.mouseDown(document.body);

    await waitFor(() => {
      expect(screen.queryByRole('tooltip')).not.toBeInTheDocument();
    });
  });

  it('hides popup when Escape key is pressed', async () => {
    const mockSelection = {
      toString: () => 'Valid selection',
      isCollapsed: false,
      getRangeAt: jest.fn(() => ({
        getBoundingClientRect: () => ({ top: 100, left: 100, right: 200, bottom: 120, width: 100, height: 20 })
      }))
    };

    mockGetSelection.mockReturnValue(mockSelection);

    render(
      <${componentName} onSelect={mockOnSelect}>
        <p>This is test content</p>
      </${componentName}>
    );

    const container = screen.getByRole('region');
    fireEvent.mouseUp(container);

    await waitFor(() => {
      expect(screen.getByRole('tooltip')).toBeInTheDocument();
    });

    // Press Escape
    fireEvent.keyDown(document, { key: 'Escape' });

    await waitFor(() => {
      expect(screen.queryByRole('tooltip')).not.toBeInTheDocument();
    });
  });

  it('submits with Enter key when popup is focused', async () => {
    const selectedText = 'React is a JavaScript library';
    const mockSelection = {
      toString: () => selectedText,
      isCollapsed: false,
      getRangeAt: jest.fn(() => ({
        getBoundingClientRect: () => ({ top: 100, left: 100, right: 400, bottom: 120, width: 300, height: 20 })
      }))
    };

    mockGetSelection.mockReturnValue(mockSelection);

    render(
      <${componentName} onSelect={mockOnSelect}>
        <p>This is test content</p>
      </${componentName}>
    );

    const container = screen.getByRole('region');
    fireEvent.mouseUp(container);

    await waitFor(() => {
      expect(screen.getByRole('tooltip')).toBeInTheDocument();
    });

    const askButton = screen.getByRole('button', { name: /ask about this/i });
    askButton.focus();

    // Press Enter
    fireEvent.keyDown(document, { key: 'Enter' });

    expect(mockOnSelect).toHaveBeenCalledWith(selectedText, expect.any(DOMRect));
  });

  it('clears selection on unmount', () => {
    const mockRemoveAllRanges = jest.fn();
    mockGetSelection.mockReturnValue({
      toString: () => '',
      isCollapsed: true,
      getRangeAt: jest.fn(),
      removeAllRanges: mockRemoveAllRanges
    });

    const { unmount } = render(
      <${componentName} onSelect={mockOnSelect}>
        <p>This is test content</p>
      </${componentName}>
    );

    unmount();

    expect(mockRemoveAllRanges).toHaveBeenCalled();
  });
});`;
  }

  generateStory(componentName) {
    return `import type { Meta, StoryObj } from '@storybook/react';
import { ${componentName} } from './index';

const sampleContent = \`
# Understanding React Hooks

React Hooks revolutionized the way we write React components. They allow you to use state and other React features without writing a class.

## useState Hook

The useState hook lets you add React state to function components. When you call useState, you declare a state variable.

\`\`\`jsx
import React, { useState } from 'react';

function Counter() {
  const [count, setCount] = useState(0);

  return (
    <div>
      <p>You clicked {count} times</p>
      <button onClick={() => setCount(count + 1)}>
        Click me
      </button>
    </div>
  );
}
\`\`\`

## useEffect Hook

The useEffect hook lets you perform side effects in function components. It serves the same purpose as componentDidMount, componentDidUpdate, and componentWillUnmount combined.

## Custom Hooks

You can also build your own Hooks to reuse stateful behavior between different components.
\`;

const meta: Meta<typeof ${componentName}> = {
  title: 'Components/${componentName}',
  component: ${componentName},
  parameters: {
    layout: 'centered',
  },
  tags: ['autodocs'],
  argTypes: {
    minLength: {
      control: 'number',
      description: 'Minimum selection length to trigger popup',
    },
    maxLength: {
      control: 'number',
      description: 'Maximum selection length before truncation',
    },
  },
  decorators: [
    (Story) => (
      <div style={{ maxWidth: '600px', padding: '20px', fontFamily: 'system-ui, -apple-system, sans-serif' }}>
        <Story />
      </div>
    ),
  ],
};

export default meta;
type Story = StoryObj<typeof meta>;

export const Default: Story = {
  args: {
    minLength: 10,
    maxLength: 100,
    onSelect: (text, rect) => {
      console.log('Selected text:', text);
      console.log('Selection position:', rect);
    },
  },
  render: (args) => (
    <${componentName} {...args}>
      <div dangerouslySetInnerHTML={{ __html: sampleContent.replace(/\\n/g, '<br />').replace(/##/g, '<h2>').replace(/# /g, '<h1>').replace(/\`\`\`jsx/g, '<pre><code>').replace(/\`\`\`/g, '</code></pre>') }} />
    </${componentName}>
  ),
};

export const WithLongContent: Story = {
  args: {
    minLength: 5,
    maxLength: 50,
    onSelect: (text, rect) => {
      alert(\`Selected: \${text}\`);
      console.log('Selection position:', rect);
    },
  },
  render: (args) => (
    <${componentName} {...args}>
      <p>
        This is a long paragraph with plenty of text to select.
        You can select short or long portions of this text to see how the component behaves.
        Try selecting different amounts of text to test the minLength and maxLength features.
      </p>
      <p>
        React Hooks are functions that let you "hook into" React state and lifecycle features from function components.
        Hooks don't work inside classes — they let you use React without classes.
      </p>
    </${componentName}>
  ),
};

export const CustomHandler: Story = {
  args: {
    minLength: 8,
    maxLength: 200,
    onSelect: (text, rect) => {
      // Simulate passing to a chatbot
      const modal = document.createElement('div');
      modal.style.cssText = \`
        position: fixed;
        top: 50%;
        left: 50%;
        transform: translate(-50%, -50%);
        background: white;
        padding: 20px;
        border-radius: 8px;
        box-shadow: 0 4px 12px rgba(0,0,0,0.2);
        z-index: 1000;
        max-width: 400px;
      \`;
      modal.innerHTML = \`
        <h3>Question about:</h3>
        <p style="font-style: italic; background: #f5f5f5; padding: 10px; border-radius: 4px;">\${text}</p>
        <button onclick="this.parentElement.remove()" style="margin-top: 10px;">Close</button>
      \`;
      document.body.appendChild(modal);
    },
  },
  render: (args) => (
    <${componentName} {...args}>
      <div style={{ lineHeight: 1.6 }}>
        <p>Select any text in this paragraph to ask a question about it. The handler will show a modal with your selection.</p>
        <p>JavaScript is a programming language that enables interactive web pages. It is an essential part of web applications alongside HTML and CSS.</p>
        <p>Try selecting different portions of text to see how the component handles various selection lengths.</p>
      </div>
    </${componentName}>
  ),
};`;
  }
}

module.exports = TextSelectorGenerator;