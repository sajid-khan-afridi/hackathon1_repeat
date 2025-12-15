import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import { ChatInputProps } from './types';

export const ChatInput: React.FC<ChatInputProps> = ({
  onSendMessage,
  onClearHistory,
  disabled = false,
  placeholder = 'Ask about robotics...',
  className,
}) => {
  const [message, setMessage] = useState('');
  const [isComposing, setIsComposing] = useState(false);
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  // Auto-resize textarea
  useEffect(() => {
    const textarea = textareaRef.current;
    if (textarea) {
      textarea.style.height = 'auto';
      textarea.style.height = `${Math.min(textarea.scrollHeight, 120)}px`;
    }
  }, [message]);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();

    if (message.trim() && !disabled && !isComposing) {
      onSendMessage(message.trim());
      setMessage('');

      // Reset textarea height
      if (textareaRef.current) {
        textareaRef.current.style.height = 'auto';
      }
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey && !isComposing) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  const handleClearHistory = () => {
    setMessage('');
    onClearHistory();
  };

  return (
    <div className={clsx('chat-input-container', className)}>
      <form onSubmit={handleSubmit} className="chat-input-form">
        <div className="input-wrapper">
          <textarea
            ref={textareaRef}
            value={message}
            onChange={(e) => setMessage(e.target.value)}
            onKeyDown={handleKeyDown}
            onCompositionStart={() => setIsComposing(true)}
            onCompositionEnd={() => setIsComposing(false)}
            placeholder={placeholder}
            disabled={disabled}
            className="chat-textarea"
            rows={1}
            aria-label="Type your question"
            aria-describedby="input-help"
          />
          <button
            type="submit"
            disabled={!message.trim() || disabled}
            className="send-button"
            aria-label="Send message"
            title={message.trim() ? 'Send message (Enter)' : 'Type a message to send'}
          >
            <svg
              width="24"
              height="24"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
              className={clsx('send-icon', { 'has-text': message.trim() })}
            >
              <line x1="22" y1="2" x2="11" y2="13"></line>
              <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
            </svg>
          </button>
        </div>
        <div id="input-help" className="input-help">
          Press <kbd>Enter</kbd> to send, <kbd>Shift+Enter</kbd> for new line
          <button
            type="button"
            onClick={handleClearHistory}
            className="clear-history-button"
            aria-label="Clear chat history"
            title="Clear all messages"
          >
            Clear History
          </button>
        </div>
      </form>

      <style jsx>{`
        .chat-input-container {
          border-top: 1px solid var(--ifm-color-emphasis-200);
          padding: 1rem;
          background-color: var(--ifm-background-color);
        }

        .chat-input-form {
          display: flex;
          flex-direction: column;
          gap: 0.5rem;
        }

        .input-wrapper {
          position: relative;
          display: flex;
          align-items: flex-end;
          gap: 0.5rem;
        }

        .chat-textarea {
          flex: 1;
          min-height: 44px;
          max-height: 120px;
          padding: 0.75rem 3rem 0.75rem 1rem;
          border: 2px solid var(--ifm-color-emphasis-200);
          border-radius: 8px;
          font-size: 1rem;
          line-height: 1.5;
          resize: none;
          outline: none;
          transition: all 0.2s ease;
          background-color: var(--ifm-background-color);
          color: var(--ifm-font-color-base);
        }

        .chat-textarea:focus {
          border-color: var(--ifm-color-primary);
          box-shadow: 0 0 0 3px rgba(var(--ifm-color-primary-rgb), 0.1);
        }

        .chat-textarea:disabled {
          opacity: 0.6;
          cursor: not-allowed;
        }

        .chat-textarea::placeholder {
          color: var(--ifm-color-emphasis-600);
        }

        .send-button {
          position: absolute;
          right: 0.5rem;
          bottom: 0.75rem;
          width: 36px;
          height: 36px;
          padding: 0;
          border: none;
          border-radius: 50%;
          background-color: transparent;
          cursor: pointer;
          transition: all 0.2s ease;
          display: flex;
          align-items: center;
          justify-content: center;
        }

        .send-button:not(:disabled):hover {
          background-color: var(--ifm-color-primary-lightest);
        }

        .send-button:disabled {
          cursor: not-allowed;
          opacity: 0.4;
        }

        .send-icon {
          color: var(--ifm-color-emphasis-600);
          transition: transform 0.2s ease;
        }

        .send-icon.has-text {
          color: var(--ifm-color-primary);
          transform: rotate(-45deg);
        }

        .send-button:not(:disabled):hover .send-icon {
          transform: rotate(-45deg) scale(1.1);
        }

        .input-help {
          display: flex;
          justify-content: space-between;
          align-items: center;
          font-size: 0.875rem;
          color: var(--ifm-color-emphasis-600);
        }

        kbd {
          display: inline-block;
          padding: 0.2rem 0.4rem;
          font-size: 0.75rem;
          font-family: var(--ifm-font-family-monospace);
          line-height: 1;
          color: var(--ifm-color-emphasis-800);
          background-color: var(--ifm-color-emphasis-200);
          border-radius: 4px;
          border: 1px solid var(--ifm-color-emphasis-300);
        }

        .clear-history-button {
          padding: 0.25rem 0.5rem;
          font-size: 0.875rem;
          color: var(--ifm-color-emphasis-700);
          background: none;
          border: 1px solid var(--ifm-color-emphasis-300);
          border-radius: 4px;
          cursor: pointer;
          transition: all 0.2s ease;
        }

        .clear-history-button:hover {
          color: var(--ifm-color-danger);
          border-color: var(--ifm-color-danger);
          background-color: var(--ifm-color-danger-contrast-background);
        }

        /* Dark mode adjustments */
        [data-theme='dark'] .chat-textarea {
          border-color: var(--ifm-color-emphasis-300);
        }

        [data-theme='dark'] .chat-textarea:focus {
          box-shadow: 0 0 0 3px rgba(var(--ifm-color-primary-rgb), 0.2);
        }

        [data-theme='dark'] kbd {
          color: var(--ifm-color-emphasis-300);
          background-color: var(--ifm-color-emphasis-800);
          border-color: var(--ifm-color-emphasis-700);
        }

        /* Mobile adjustments */
        @media (max-width: 768px) {
          .chat-input-container {
            padding: 0.75rem;
          }

          .input-help {
            font-size: 0.75rem;
          }

          .send-button {
            width: 44px;
            height: 44px;
            right: 0.75rem;
            bottom: 0.75rem;
          }
        }
      `}</style>
    </div>
  );
};
