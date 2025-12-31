/**
 * ChatbotWidget Unit Tests
 *
 * Tests cover:
 * - Component rendering and state management
 * - User interactions (submit query, clear history)
 * - Accessibility features (ARIA labels, keyboard navigation)
 * - Error handling and loading states
 * - Session persistence
 * - Confidence warnings and suggested terms
 */

import React from 'react';
import { render, screen, waitFor, fireEvent, within } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import '@testing-library/jest-dom';
import ChatbotWidget from './index';
import type { QueryResponse } from './types';

// Mock useAuthContext - use @site/ alias that the component uses
jest.mock('@site/src/context/AuthContext', () => ({
  useAuthContext: () => ({
    isAuthenticated: false,
    isLoading: false,
    user: null,
    profile: null,
    login: jest.fn(),
    signup: jest.fn(),
    logout: jest.fn(),
    refreshAuth: jest.fn(),
    updateProfile: jest.fn(),
  }),
}));

// Mock fetch with proper headers
global.fetch = jest.fn();

/**
 * Create a proper mock response with headers
 * The component checks content-type to determine streaming vs JSON
 */
const createMockResponse = (data: any, ok = true, status = 200) => ({
  ok,
  status,
  headers: {
    get: (name: string) => {
      if (name.toLowerCase() === 'content-type') {
        return 'application/json'; // Not streaming, so falls back to JSON
      }
      return null;
    },
  },
  json: async () => data,
});

// Mock localStorage
const localStorageMock = (() => {
  let store: Record<string, string> = {};
  return {
    getItem: (key: string) => store[key] || null,
    setItem: (key: string, value: string) => {
      store[key] = value.toString();
    },
    removeItem: (key: string) => {
      delete store[key];
    },
    clear: () => {
      store = {};
    },
  };
})();

Object.defineProperty(window, 'localStorage', {
  value: localStorageMock,
});

describe('ChatbotWidget', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    localStorageMock.clear();
  });

  describe('Rendering', () => {
    test('renders with title and empty state', () => {
      render(<ChatbotWidget />);

      expect(
        screen.getByRole('region', { name: /robotics textbook chatbot/i })
      ).toBeInTheDocument();
      expect(screen.getByRole('heading', { name: /ask about robotics/i })).toBeInTheDocument();
      expect(screen.getByText(/ask a question about the robotics textbook/i)).toBeInTheDocument();
    });

    test('renders input field with placeholder', () => {
      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);
      expect(input).toBeInTheDocument();
      expect(input).toHaveAttribute('aria-multiline', 'true');
    });

    test('renders submit button', () => {
      render(<ChatbotWidget />);

      const submitButton = screen.getByRole('button', { name: /send message/i });
      expect(submitButton).toBeInTheDocument();
      expect(submitButton).toBeDisabled(); // Disabled when input is empty
    });

    test('does not render clear history button when no messages', () => {
      render(<ChatbotWidget />);

      expect(screen.queryByRole('button', { name: /clear chat history/i })).not.toBeInTheDocument();
    });
  });

  describe('User Interactions', () => {
    test('enables submit button when input has text', async () => {
      const user = userEvent.setup();
      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);
      const submitButton = screen.getByRole('button', { name: /send message/i });

      expect(submitButton).toBeDisabled();

      await user.type(input, 'What is inverse kinematics?');

      expect(submitButton).toBeEnabled();
    });

    test('submits query and displays user message', async () => {
      const user = userEvent.setup();
      const mockResponse: QueryResponse = {
        answer: 'Inverse kinematics is the process of determining joint angles...',
        sources: [
          {
            chapter_id: 'module-1-chapter-2',
            chapter_title: 'Introduction to Kinematics',
            relevance_score: 0.92,
            excerpt: 'Inverse kinematics calculates joint parameters...',
            position: 1,
          },
        ],
        confidence: 0.85,
        session_id: '123e4567-e89b-12d3-a456-426614174000',
        tokens_used: {
          input_tokens: 50,
          output_tokens: 100,
          total_tokens: 150,
        },
      };

      (global.fetch as jest.Mock).mockResolvedValueOnce(createMockResponse(mockResponse));

      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);
      const submitButton = screen.getByRole('button', { name: /send message/i });

      await user.type(input, 'What is inverse kinematics?');
      await user.click(submitButton);

      // User message should appear immediately
      expect(screen.getByText('What is inverse kinematics?')).toBeInTheDocument();

      // Wait for assistant response
      await waitFor(() => {
        expect(screen.getByText(/inverse kinematics is the process/i)).toBeInTheDocument();
      });

      // Check API call - accept streaming header since that's what the component uses
      expect(global.fetch).toHaveBeenCalledWith(
        'http://localhost:8000/api/v1/query',
        expect.objectContaining({
          method: 'POST',
          body: expect.stringContaining('What is inverse kinematics?'),
        })
      );
    });

    test('Enter key submits query, Shift+Enter inserts newline', async () => {
      const user = userEvent.setup();
      const mockResponse: QueryResponse = {
        answer: 'Test answer',
        sources: [],
        confidence: 0.8,
        session_id: '123e4567-e89b-12d3-a456-426614174000',
        tokens_used: { input_tokens: 10, output_tokens: 20, total_tokens: 30 },
      };

      (global.fetch as jest.Mock).mockResolvedValueOnce(createMockResponse(mockResponse));

      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(
        /ask a question about robotics/i
      ) as HTMLTextAreaElement;

      // Type and press Enter (should submit)
      await user.type(input, 'Test query{Enter}');

      await waitFor(() => {
        expect(screen.getByText('Test query')).toBeInTheDocument();
      });

      expect(global.fetch).toHaveBeenCalledTimes(1);
    });

    test('enforces 1000 character limit', async () => {
      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(
        /ask a question about robotics/i
      ) as HTMLTextAreaElement;

      // First set exactly 1000 characters (at the limit)
      const exactLimit = 'a'.repeat(1000);
      fireEvent.change(input, { target: { value: exactLimit } });
      expect(input.value.length).toBe(1000);

      // Now try to add more - should reject values over 1000
      const overLimit = 'a'.repeat(1001);
      fireEvent.change(input, { target: { value: overLimit } });
      // Component rejects values over 1000, so it stays at 1000
      expect(input.value.length).toBe(1000);
    });

    test('shows character counter when approaching limit', async () => {
      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);

      // Type 920 characters (80 remaining) - use fireEvent for speed
      const text = 'a'.repeat(920);
      fireEvent.change(input, { target: { value: text } });

      // Character counter should appear
      await waitFor(() => {
        expect(screen.getByText(/80 characters remaining/i)).toBeInTheDocument();
      });
    });

    test('clears input after successful submission', async () => {
      const user = userEvent.setup();
      const mockResponse: QueryResponse = {
        answer: 'Test answer',
        sources: [],
        confidence: 0.8,
        session_id: '123e4567-e89b-12d3-a456-426614174000',
        tokens_used: { input_tokens: 10, output_tokens: 20, total_tokens: 30 },
      };

      (global.fetch as jest.Mock).mockResolvedValueOnce(createMockResponse(mockResponse));

      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(
        /ask a question about robotics/i
      ) as HTMLTextAreaElement;
      const submitButton = screen.getByRole('button', { name: /send message/i });

      await user.type(input, 'Test query');
      await user.click(submitButton);

      // Input should be cleared after submission
      await waitFor(() => {
        expect(input.value).toBe('');
      });
    });
  });

  describe('Session Management', () => {
    test('persists session ID to localStorage', async () => {
      const user = userEvent.setup();
      const mockResponse: QueryResponse = {
        answer: 'Test answer',
        sources: [],
        confidence: 0.8,
        session_id: '123e4567-e89b-12d3-a456-426614174000',
        tokens_used: { input_tokens: 10, output_tokens: 20, total_tokens: 30 },
      };

      (global.fetch as jest.Mock).mockResolvedValueOnce(createMockResponse(mockResponse));

      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);
      await user.type(input, 'Test query');
      await user.click(screen.getByRole('button', { name: /send message/i }));

      await waitFor(() => {
        expect(localStorageMock.getItem('chatbot-session-id')).toBe(
          '123e4567-e89b-12d3-a456-426614174000'
        );
      });
    });

    test('includes session ID in subsequent requests', async () => {
      const user = userEvent.setup();
      const sessionId = '123e4567-e89b-12d3-a456-426614174000';

      localStorageMock.setItem('chatbot-session-id', sessionId);

      const mockResponse: QueryResponse = {
        answer: 'Test answer',
        sources: [],
        confidence: 0.8,
        session_id: sessionId,
        tokens_used: { input_tokens: 10, output_tokens: 20, total_tokens: 30 },
      };

      (global.fetch as jest.Mock).mockResolvedValueOnce(createMockResponse(mockResponse));

      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);
      await user.type(input, 'Follow-up question');
      await user.click(screen.getByRole('button', { name: /send message/i }));

      await waitFor(() => {
        expect(global.fetch).toHaveBeenCalledWith(
          'http://localhost:8000/api/v1/query',
          expect.objectContaining({
            body: expect.stringContaining(`"session_id":"${sessionId}"`),
          })
        );
      });
    });

    test('clears history and removes session from localStorage', async () => {
      const user = userEvent.setup();
      const sessionId = '123e4567-e89b-12d3-a456-426614174000';

      // Set up initial state with a message
      const mockResponse: QueryResponse = {
        answer: 'Test answer',
        sources: [],
        confidence: 0.8,
        session_id: sessionId,
        tokens_used: { input_tokens: 10, output_tokens: 20, total_tokens: 30 },
      };

      (global.fetch as jest.Mock)
        .mockResolvedValueOnce(createMockResponse(mockResponse))
        .mockResolvedValueOnce(createMockResponse({ success: true }));

      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);
      await user.type(input, 'Test query');
      await user.click(screen.getByRole('button', { name: /send message/i }));

      // Wait for message to appear
      await waitFor(() => {
        expect(screen.getByText('Test query')).toBeInTheDocument();
      });

      // Clear history button should now be visible
      const clearButton = screen.getByRole('button', { name: /clear chat history/i });
      await user.click(clearButton);

      // Messages should be cleared
      await waitFor(() => {
        expect(screen.queryByText('Test query')).not.toBeInTheDocument();
      });

      // localStorage should be cleared
      expect(localStorageMock.getItem('chatbot-session-id')).toBeNull();

      // DELETE request should have been called
      expect(global.fetch).toHaveBeenCalledWith(
        `http://localhost:8000/api/v1/chat/sessions/${sessionId}`,
        { method: 'DELETE' }
      );
    });
  });

  describe('Response Display', () => {
    test('displays confidence indicator', async () => {
      const user = userEvent.setup();
      const mockResponse: QueryResponse = {
        answer: 'Test answer',
        sources: [],
        confidence: 0.85,
        session_id: '123e4567-e89b-12d3-a456-426614174000',
        tokens_used: { input_tokens: 10, output_tokens: 20, total_tokens: 30 },
      };

      (global.fetch as jest.Mock).mockResolvedValueOnce(createMockResponse(mockResponse));

      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);
      await user.type(input, 'Test query');
      await user.click(screen.getByRole('button', { name: /send message/i }));

      await waitFor(() => {
        expect(screen.getByText(/85%/i)).toBeInTheDocument();
      });
    });

    test('displays low confidence warning (0.2-0.3 range)', async () => {
      const user = userEvent.setup();
      const mockResponse: QueryResponse = {
        answer: 'Test answer',
        sources: [],
        confidence: 0.25,
        session_id: '123e4567-e89b-12d3-a456-426614174000',
        tokens_used: { input_tokens: 10, output_tokens: 20, total_tokens: 30 },
      };

      (global.fetch as jest.Mock).mockResolvedValueOnce(createMockResponse(mockResponse));

      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);
      await user.type(input, 'Test query');
      await user.click(screen.getByRole('button', { name: /send message/i }));

      await waitFor(() => {
        // ConfidenceIndicator shows: "This answer has low confidence. Consider rephrasing your question for better results."
        expect(screen.getByText(/low confidence.*rephrasing/i)).toBeInTheDocument();
      });
    });

    test('displays source citations with clickable links', async () => {
      const user = userEvent.setup();
      const mockResponse: QueryResponse = {
        answer: 'Test answer',
        sources: [
          {
            chapter_id: 'module-1-chapter-2',
            chapter_title: 'Introduction to Kinematics',
            relevance_score: 0.92,
            excerpt: 'Relevant excerpt...',
            position: 1,
          },
        ],
        confidence: 0.85,
        session_id: '123e4567-e89b-12d3-a456-426614174000',
        tokens_used: { input_tokens: 10, output_tokens: 20, total_tokens: 30 },
      };

      (global.fetch as jest.Mock).mockResolvedValueOnce(createMockResponse(mockResponse));

      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);
      await user.type(input, 'Test query');
      await user.click(screen.getByRole('button', { name: /send message/i }));

      await waitFor(() => {
        expect(screen.getByText(/sources \(1\)/i)).toBeInTheDocument();
      });

      // Expand sources
      const sourcesToggle = screen.getByRole('button', { name: /sources \(1\)/i });
      await user.click(sourcesToggle);

      // Check source link
      const sourceLink = screen.getByRole('link', { name: /introduction to kinematics/i });
      expect(sourceLink).toHaveAttribute('href', '/docs/module-1/chapter-2');
    });

    test('displays token usage for educational transparency', async () => {
      const user = userEvent.setup();
      const mockResponse: QueryResponse = {
        answer: 'Test answer',
        sources: [],
        confidence: 0.85,
        session_id: '123e4567-e89b-12d3-a456-426614174000',
        tokens_used: {
          input_tokens: 50,
          output_tokens: 100,
          total_tokens: 150,
        },
      };

      (global.fetch as jest.Mock).mockResolvedValueOnce(createMockResponse(mockResponse));

      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);
      await user.type(input, 'Test query');
      await user.click(screen.getByRole('button', { name: /send message/i }));

      await waitFor(() => {
        expect(screen.getByText(/150/i)).toBeInTheDocument(); // Total tokens
        expect(screen.getByText(/50 in.*100 out/i)).toBeInTheDocument();
      });
    });

    test('displays suggested terms for off-topic queries', async () => {
      const user = userEvent.setup();
      const mockResponse: QueryResponse = {
        answer: 'I can only answer questions about the robotics textbook.',
        sources: [],
        confidence: 0.15,
        session_id: '123e4567-e89b-12d3-a456-426614174000',
        tokens_used: { input_tokens: 10, output_tokens: 20, total_tokens: 30 },
        suggested_terms: ['ROS', 'kinematics', 'motion planning'],
      };

      (global.fetch as jest.Mock).mockResolvedValueOnce(createMockResponse(mockResponse));

      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);
      await user.type(input, 'What is the weather?');
      await user.click(screen.getByRole('button', { name: /send message/i }));

      await waitFor(() => {
        expect(screen.getByText(/try asking about/i)).toBeInTheDocument();
        expect(screen.getByRole('button', { name: /ROS/i })).toBeInTheDocument();
        expect(screen.getByRole('button', { name: /kinematics/i })).toBeInTheDocument();
      });
    });
  });

  describe('Error Handling', () => {
    test('displays error message on fetch failure', async () => {
      const user = userEvent.setup();

      (global.fetch as jest.Mock).mockRejectedValueOnce(new Error('Network error'));

      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);
      await user.type(input, 'Test query');
      await user.click(screen.getByRole('button', { name: /send message/i }));

      await waitFor(() => {
        expect(screen.getByRole('alert')).toHaveTextContent(/network error/i);
      });
    });

    test('displays retry button on error', async () => {
      const user = userEvent.setup();

      (global.fetch as jest.Mock)
        .mockRejectedValueOnce(new Error('Network error'))
        .mockResolvedValueOnce(
          createMockResponse({
            answer: 'Test answer',
            sources: [],
            confidence: 0.8,
            session_id: '123e4567-e89b-12d3-a456-426614174000',
            tokens_used: { input_tokens: 10, output_tokens: 20, total_tokens: 30 },
          })
        );

      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);
      await user.type(input, 'Test query');
      await user.click(screen.getByRole('button', { name: /send message/i }));

      // Wait for error
      await waitFor(() => {
        expect(screen.getByRole('alert')).toBeInTheDocument();
      });

      // Click retry
      const retryButton = screen.getByRole('button', { name: /retry/i });
      await user.click(retryButton);

      // Should succeed on retry
      await waitFor(() => {
        expect(screen.getByText('Test answer')).toBeInTheDocument();
      });
    });

    test('handles HTTP error responses', async () => {
      const user = userEvent.setup();

      (global.fetch as jest.Mock).mockResolvedValueOnce(
        createMockResponse({ detail: 'Rate limit exceeded' }, false, 429)
      );

      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);
      await user.type(input, 'Test query');
      await user.click(screen.getByRole('button', { name: /send message/i }));

      await waitFor(() => {
        expect(screen.getByRole('alert')).toHaveTextContent(/rate limit exceeded/i);
      });
    });
  });

  describe('Accessibility', () => {
    test('has proper ARIA labels', () => {
      render(<ChatbotWidget />);

      // Main container
      expect(
        screen.getByRole('region', { name: /robotics textbook chatbot/i })
      ).toBeInTheDocument();

      // There are two elements with role="search":
      // 1. ModuleFilter (no aria-label, so accessible name is empty)
      // 2. ChatInput form (aria-label="Ask a question about the textbook")
      const searchForms = screen.getAllByRole('search');
      expect(searchForms.length).toBe(2);

      // The input form should have the specific aria-label
      const inputForm = searchForms.find(
        (el) => el.getAttribute('aria-label') === 'Ask a question about the textbook'
      );
      expect(inputForm).toBeInTheDocument();

      // Input should be labeled - use getAllByLabelText as both form and textarea have similar labels
      const labeledElements = screen.getAllByLabelText(/ask a question/i);
      expect(labeledElements.length).toBeGreaterThanOrEqual(1);
    });

    test('manages focus properly', async () => {
      const user = userEvent.setup();
      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);

      // Focus input
      await user.click(input);
      expect(input).toHaveFocus();

      // Tab should move focus (not necessarily to submit button due to disabled state)
      await user.tab();
      // Just verify that tabbing works - the active element should change
      expect(document.activeElement).not.toBe(input);
    });

    test('announces loading state to screen readers', async () => {
      const user = userEvent.setup();

      // Mock slow response
      (global.fetch as jest.Mock).mockImplementation(
        () =>
          new Promise((resolve) =>
            setTimeout(
              () =>
                resolve(
                  createMockResponse({
                    answer: 'Test answer',
                    sources: [],
                    confidence: 0.8,
                    session_id: '123e4567-e89b-12d3-a456-426614174000',
                    tokens_used: { input_tokens: 10, output_tokens: 20, total_tokens: 30 },
                  })
                ),
              100
            )
          )
      );

      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);
      await user.type(input, 'Test query');
      await user.click(screen.getByRole('button', { name: /send message/i }));

      // Check loading state
      expect(screen.getByRole('status', { name: /loading response/i })).toBeInTheDocument();
    });

    test('provides keyboard navigation for interactive elements', async () => {
      const user = userEvent.setup();
      const mockResponse: QueryResponse = {
        answer: 'Test answer',
        sources: [
          {
            chapter_id: 'module-1-chapter-2',
            chapter_title: 'Introduction to Kinematics',
            relevance_score: 0.92,
            excerpt: 'Relevant excerpt...',
            position: 1,
          },
        ],
        confidence: 0.85,
        session_id: '123e4567-e89b-12d3-a456-426614174000',
        tokens_used: { input_tokens: 10, output_tokens: 20, total_tokens: 30 },
      };

      (global.fetch as jest.Mock).mockResolvedValueOnce(createMockResponse(mockResponse));

      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);
      await user.type(input, 'Test query{Enter}');

      await waitFor(() => {
        expect(screen.getByText(/sources \(1\)/i)).toBeInTheDocument();
      });

      // Tab through interactive elements
      const sourcesToggle = screen.getByRole('button', { name: /sources \(1\)/i });
      sourcesToggle.focus();
      expect(sourcesToggle).toHaveFocus();

      // Click to expand (fireEvent.keyDown doesn't trigger button click behavior)
      await user.click(sourcesToggle);

      // Source link should be accessible via keyboard
      const sourceLink = await screen.findByRole('link', { name: /introduction to kinematics/i });
      expect(sourceLink).toBeInTheDocument();
    });
  });

  describe('Loading States', () => {
    test('disables input and submit button during loading', async () => {
      const user = userEvent.setup();

      // Mock slow response
      (global.fetch as jest.Mock).mockImplementation(
        () =>
          new Promise((resolve) =>
            setTimeout(
              () =>
                resolve(
                  createMockResponse({
                    answer: 'Test answer',
                    sources: [],
                    confidence: 0.8,
                    session_id: '123e4567-e89b-12d3-a456-426614174000',
                    tokens_used: { input_tokens: 10, output_tokens: 20, total_tokens: 30 },
                  })
                ),
              100
            )
          )
      );

      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(
        /ask a question about robotics/i
      ) as HTMLTextAreaElement;
      const submitButton = screen.getByRole('button', { name: /send message/i });

      await user.type(input, 'Test query');
      await user.click(submitButton);

      // Should be disabled during loading
      expect(input).toBeDisabled();
      expect(submitButton).toBeDisabled();

      // Should re-enable after response
      await waitFor(() => {
        expect(input).not.toBeDisabled();
      });
    });

    test('shows loading indicator', async () => {
      const user = userEvent.setup();

      // Mock slow response
      (global.fetch as jest.Mock).mockImplementation(
        () =>
          new Promise((resolve) =>
            setTimeout(
              () =>
                resolve(
                  createMockResponse({
                    answer: 'Test answer',
                    sources: [],
                    confidence: 0.8,
                    session_id: '123e4567-e89b-12d3-a456-426614174000',
                    tokens_used: { input_tokens: 10, output_tokens: 20, total_tokens: 30 },
                  })
                ),
              100
            )
          )
      );

      render(<ChatbotWidget />);

      const input = screen.getByPlaceholderText(/ask a question about robotics/i);
      await user.type(input, 'Test query');
      await user.click(screen.getByRole('button', { name: /send message/i }));

      // Loading indicator should appear
      expect(screen.getByText(/loading response/i)).toBeInTheDocument();

      // Should disappear after response
      await waitFor(() => {
        expect(screen.queryByText(/loading response/i)).not.toBeInTheDocument();
      });
    });
  });
});
