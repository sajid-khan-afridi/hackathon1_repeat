const BaseGenerator = require('./BaseGenerator');

class ChatbotWidgetGenerator extends BaseGenerator {
  async generate({ props_schema, styling, accessibility_level }) {
    const componentName = 'ChatbotWidget';

    // Generate interfaces
    const interfaces = this.generateInterfaces(componentName, props_schema);

    // Generate component code
    const code = this.generateComponentCode(componentName, props_schema, styling, accessibility_level);

    // Generate styles
    const styles = this.generateChatbotStyles(styling);

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
      styling
    });

    const accessibilityAttrs = this.generateAccessibilityAttributes(accessibility);

    return `${imports}
const ${componentName}: React.FC<${componentName}Props> = ({
  position = 'bottom-right',
  defaultOpen = false,
  apiEndpoint,
  theme = 'auto',
  ...props
}) => {
  const [isOpen, setIsOpen] = useState(defaultOpen);
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [selectedContext, setSelectedContext] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen) {
      inputRef.current?.focus();
    }
  }, [isOpen]);

  const handleSendMessage = async (content: string) => {
    if (!content.trim()) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content,
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(apiEndpoint, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          message: content,
          context: selectedContext,
          history: messages.slice(-10) // Last 10 messages
        })
      });

      if (!response.ok) throw new Error('Failed to send message');

      const data = await response.json();

      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: data.content,
        timestamp: new Date(),
        sources: data.sources
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage(e.currentTarget.value);
      e.currentTarget.value = '';
    }
  };

  const copyMessage = (content: string) => {
    navigator.clipboard.writeText(content);
  };

  const handleFeedback = (messageId: string, feedback: 'up' | 'down') => {
    // Implement feedback logic
    console.log(\`Feedback \${feedback} for message \${messageId}\`);
  };

  const positionClasses = {
    'bottom-right': styles.bottomRight,
    'bottom-left': styles.bottomLeft,
    'sidebar': styles.sidebar
  };

  const themeClasses = {
    light: styles.lightTheme,
    dark: styles.darkTheme,
    auto: styles.autoTheme
  };

  return (
    <div
      className={\`\${styles.container} \${positionClasses[position]} \${themeClasses[theme]}\`}
      role="complementary"
      aria-label="Chat assistant"
      ${accessibilityAttrs.join(' ')}
    >
      {/* Toggle Button */}
      <button
        className={styles.toggleButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
        aria-expanded={isOpen}
      >
        {isOpen ? (
          <svg className={styles.icon} fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
          </svg>
        ) : (
          <svg className={styles.icon} fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M8 12h.01M12 12h.01M16 12h.01M21 12c0 4.418-4.03 8-9 8a9.863 9.863 0 01-4.255-.949L3 20l1.395-3.72C3.512 15.042 3 13.574 3 12c0-4.418 4.03-8 9-8s9 3.582 9 8z" />
          </svg>
        )}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.header}>
            <h3 className={styles.title}>Chat Assistant</h3>
            {selectedContext && (
              <div className={styles.contextIndicator}>
                <span>Context: {selectedContext.substring(0, 50)}...</span>
                <button
                  onClick={() => setSelectedContext(null)}
                  aria-label="Clear context"
                  className={styles.clearContext}
                >
                  ×
                </button>
              </div>
            )}
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer} role="log" aria-live="polite">
            {messages.map((message) => (
              <div
                key={message.id}
                className={\`\${styles.message} \${styles[message.role]}\`}
              >
                <div className={styles.messageContent}>
                  {message.content}
                </div>
                {message.sources && message.sources.length > 0 && (
                  <div className={styles.sources}>
                    <span className={styles.sourcesLabel}>Sources:</span>
                    {message.sources.map((source, index) => (
                      <a
                        key={index}
                        href={source.url}
                        target="_blank"
                        rel="noopener noreferrer"
                        className={styles.sourceLink}
                      >
                        {source.title}
                      </a>
                    ))}
                  </div>
                )}
                <div className={styles.messageActions}>
                  <button
                    onClick={() => copyMessage(message.content)}
                    className={styles.actionButton}
                    aria-label="Copy message"
                  >
                    📋
                  </button>
                  {message.role === 'assistant' && (
                    <>
                      <button
                        onClick={() => handleFeedback(message.id, 'up')}
                        className={styles.actionButton}
                        aria-label="Thumbs up"
                      >
                        👍
                      </button>
                      <button
                        onClick={() => handleFeedback(message.id, 'down')}
                        className={styles.actionButton}
                        aria-label="Thumbs down"
                      >
                        👎
                      </button>
                    </>
                  )}
                </div>
              </div>
            ))}
            {isLoading && (
              <div className={styles.typingIndicator}>
                <span></span>
                <span></span>
                <span></span>
              </div>
            )}
            {error && (
              <div className={styles.errorMessage} role="alert">
                {error}
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <div className={styles.inputContainer}>
            <input
              ref={inputRef}
              type="text"
              placeholder="Ask a question..."
              className={styles.input}
              onKeyDown={handleKeyDown}
              disabled={isLoading}
              aria-label="Type your message"
            />
            <button
              onClick={() => {
                const input = inputRef.current;
                if (input?.value) {
                  handleSendMessage(input.value);
                  input.value = '';
                }
              }}
              disabled={isLoading}
              className={styles.sendButton}
              aria-label="Send message"
            >
              <svg className={styles.icon} fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 19l9 2-9-18-9 18 9-2zm0 0v-8" />
              </svg>
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default ${componentName};`;
  }

  generateChatbotStyles(styling) {
    if (styling !== 'css_modules') return '';

    return `
.container {
  position: fixed;
  z-index: 1000;
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
}

.bottomRight {
  bottom: 20px;
  right: 20px;
}

.bottomLeft {
  bottom: 20px;
  left: 20px;
}

.sidebar {
  position: relative;
  width: 100%;
  height: 100vh;
}

.toggleButton {
  position: absolute;
  bottom: 0;
  right: 0;
  width: 60px;
  height: 60px;
  border-radius: 50%;
  background: #0066cc;
  color: white;
  border: none;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  transition: all 0.2s ease;
}

.toggleButton:hover {
  background: #0052a3;
  transform: scale(1.05);
}

.toggleButton:focus {
  outline: 2px solid #0066cc;
  outline-offset: 2px;
}

.icon {
  width: 24px;
  height: 24px;
}

.chatWindow {
  position: absolute;
  bottom: 80px;
  right: 0;
  width: 380px;
  height: 500px;
  background: white;
  border-radius: 12px;
  box-shadow: 0 8px 24px rgba(0, 0, 0, 0.15);
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

.sidebar .chatWindow {
  position: relative;
  bottom: auto;
  right: auto;
  width: 100%;
  height: 100%;
  border-radius: 0;
}

.header {
  padding: 16px;
  background: #f8f9fa;
  border-bottom: 1px solid #e9ecef;
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.title {
  margin: 0;
  font-size: 18px;
  font-weight: 600;
}

.contextIndicator {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 12px;
  color: #6c757d;
  background: #e9ecef;
  padding: 4px 8px;
  border-radius: 4px;
}

.clearContext {
  background: none;
  border: none;
  cursor: pointer;
  font-size: 16px;
  padding: 0;
  margin-left: 4px;
}

.messagesContainer {
  flex: 1;
  overflow-y: auto;
  padding: 16px;
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.message {
  max-width: 80%;
  padding: 8px 12px;
  border-radius: 12px;
  position: relative;
}

.message.user {
  align-self: flex-end;
  background: #0066cc;
  color: white;
}

.message.assistant {
  align-self: flex-start;
  background: #f8f9fa;
  color: #212529;
}

.messageContent {
  line-height: 1.4;
  white-space: pre-wrap;
}

.sources {
  margin-top: 8px;
  font-size: 12px;
  opacity: 0.8;
}

.sourcesLabel {
  font-weight: 600;
  margin-right: 4px;
}

.sourceLink {
  color: inherit;
  text-decoration: underline;
  margin-right: 8px;
}

.messageActions {
  display: flex;
  gap: 4px;
  margin-top: 4px;
  opacity: 0.6;
  transition: opacity 0.2s;
}

.message:hover .messageActions {
  opacity: 1;
}

.actionButton {
  background: none;
  border: none;
  cursor: pointer;
  padding: 4px;
  font-size: 14px;
  border-radius: 4px;
  transition: background 0.2s;
}

.actionButton:hover {
  background: rgba(0, 0, 0, 0.1);
}

.typingIndicator {
  display: flex;
  gap: 4px;
  padding: 12px;
  align-self: flex-start;
  background: #f8f9fa;
  border-radius: 12px;
}

.typingIndicator span {
  width: 8px;
  height: 8px;
  background: #6c757d;
  border-radius: 50%;
  animation: typing 1.4s infinite ease-in-out;
}

.typingIndicator span:nth-child(2) {
  animation-delay: 0.2s;
}

.typingIndicator span:nth-child(3) {
  animation-delay: 0.4s;
}

@keyframes typing {
  0%, 80%, 100% {
    transform: scale(0.8);
    opacity: 0.5;
  }
  40% {
    transform: scale(1);
    opacity: 1;
  }
}

.errorMessage {
  background: #f8d7da;
  color: #721c24;
  padding: 12px;
  border-radius: 8px;
  align-self: flex-start;
}

.inputContainer {
  display: flex;
  gap: 8px;
  padding: 16px;
  background: #f8f9fa;
  border-top: 1px solid #e9ecef;
}

.input {
  flex: 1;
  padding: 8px 12px;
  border: 1px solid #ced4da;
  border-radius: 20px;
  font-size: 14px;
  outline: none;
  transition: border-color 0.2s;
}

.input:focus {
  border-color: #0066cc;
}

.sendButton {
  background: #0066cc;
  color: white;
  border: none;
  border-radius: 50%;
  width: 36px;
  height: 36px;
  display: flex;
  align-items: center;
  justify-content: center;
  cursor: pointer;
  transition: background 0.2s;
}

.sendButton:hover {
  background: #0052a3;
}

.sendButton:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

/* Dark theme */
[data-theme='dark'] .chatWindow {
  background: #2d3748;
  color: #e2e8f0;
}

[data-theme='dark'] .header,
[data-theme='dark'] .inputContainer {
  background: #1a202c;
  border-color: #4a5568;
}

[data-theme='dark'] .message.assistant {
  background: #4a5568;
  color: #e2e8f0;
}

[data-theme='dark'] .input {
  background: #4a5568;
  border-color: #718096;
  color: #e2e8f0;
}

[data-theme='dark'] .input:focus {
  border-color: #0066cc;
}

/* Mobile responsive */
@media (max-width: 480px) {
  .chatWindow {
    width: calc(100vw - 40px);
    right: -10px;
    height: 60vh;
  }

  .sidebar .chatWindow {
    width: 100%;
    height: 100vh;
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
  beforeEach(() => {
    mockFetch.mockClear();
  });

  it('renders the toggle button', () => {
    render(<${componentName} apiEndpoint="/api/chat" />);

    const toggleButton = screen.getByRole('button', { name: /open chat/i });
    expect(toggleButton).toBeInTheDocument();
  });

  it('opens the chat window when toggle is clicked', async () => {
    const user = userEvent.setup();
    render(<${componentName} apiEndpoint="/api/chat" />);

    const toggleButton = screen.getByRole('button', { name: /open chat/i });
    await user.click(toggleButton);

    expect(screen.getByText('Chat Assistant')).toBeInTheDocument();
  });

  it('closes the chat window when toggle is clicked again', async () => {
    const user = userEvent.setup();
    render(<${componentName} apiEndpoint="/api/chat" />);

    const toggleButton = screen.getByRole('button', { name: /open chat/i });
    await user.click(toggleButton); // Open
    await user.click(toggleButton); // Close

    expect(screen.queryByText('Chat Assistant')).not.toBeInTheDocument();
  });

  it('sends a message when Enter is pressed', async () => {
    const user = userEvent.setup();
    mockFetch.mockResolvedValueOnce({
      ok: true,
      json: async () => ({ content: 'Test response' })
    });

    render(<${componentName} apiEndpoint="/api/chat" />);

    // Open chat
    const toggleButton = screen.getByRole('button', { name: /open chat/i });
    await user.click(toggleButton);

    // Type and send message
    const input = screen.getByRole('textbox', { name: /type your message/i });
    await user.type(input, 'Hello');
    await user.keyboard('{Enter}');

    await waitFor(() => {
      expect(mockFetch).toHaveBeenCalledWith('/api/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: expect.stringContaining('Hello')
      });
    });
  });

  it('displays error when fetch fails', async () => {
    const user = userEvent.setup();
    mockFetch.mockRejectedValueOnce(new Error('Network error'));

    render(<${componentName} apiEndpoint="/api/chat" />);

    // Open chat and send message
    const toggleButton = screen.getByRole('button', { name: /open chat/i });
    await user.click(toggleButton);

    const input = screen.getByRole('textbox', { name: /type your message/i });
    await user.type(input, 'Hello');
    await user.keyboard('{Enter}');

    await waitFor(() => {
      expect(screen.getByText(/network error/i)).toBeInTheDocument();
    });
  });

  it('shows typing indicator while loading', async () => {
    const user = userEvent.setup();
    mockFetch.mockImplementationOnce(() => new Promise(resolve => setTimeout(resolve, 100)));

    render(<${componentName} apiEndpoint="/api/chat" />);

    // Open chat and send message
    const toggleButton = screen.getByRole('button', { name: /open chat/i });
    await user.click(toggleButton);

    const input = screen.getByRole('textbox', { name: /type your message/i });
    await user.type(input, 'Hello');
    await user.keyboard('{Enter}');

    // Should show typing indicator
    expect(screen.getByTestId('typing-indicator')).toBeInTheDocument();
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
    position: {
      control: 'select',
      options: ['bottom-right', 'bottom-left', 'sidebar'],
    },
    theme: {
      control: 'select',
      options: ['light', 'dark', 'auto'],
    },
    defaultOpen: {
      control: 'boolean',
    },
  },
};

export default meta;
type Story = StoryObj<typeof meta>;

export const Default: Story = {
  args: {
    apiEndpoint: '/api/chat',
  },
};

export const BottomLeft: Story = {
  args: {
    apiEndpoint: '/api/chat',
    position: 'bottom-left',
  },
};

export const Sidebar: Story = {
  args: {
    apiEndpoint: '/api/chat',
    position: 'sidebar',
  },
};

export const DefaultOpen: Story = {
  args: {
    apiEndpoint: '/api/chat',
    defaultOpen: true,
  },
};

export const DarkTheme: Story = {
  args: {
    apiEndpoint: '/api/chat',
    theme: 'dark',
  },
  parameters: {
    backgrounds: {
      default: 'dark',
    },
  },
};`;
  }
}

module.exports = ChatbotWidgetGenerator;