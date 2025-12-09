/**
 * Example: Integrating Keyword Chips with a Chat Interface
 * This file demonstrates how to use the contextual keyword chips in various scenarios
 */

import { KeywordChips, QuickKeywordChips } from './index.js';

// Example 1: Basic Chat Message Component
export function ChatMessage({ message, isAI, onFollowUp }) {
  if (!isAI) {
    return (
      <div className="user-message">
        <p>{message}</p>
      </div>
    );
  }

  return (
    <div className="ai-message">
      <div className="message-content">
        <p>{message}</p>
        <KeywordChips
          aiAnswer={message}
          onChipClick={(chipText) => {
            // Create a contextual follow-up message
            const followUp = chipText.includes('?')
              ? chipText
              : `Tell me more about ${chipText}`;
            onFollowUp(followUp);
          }}
          maxChips={4}
          variant="compact"
          animated={true}
        />
      </div>
    </div>
  );
}

// Example 2: Document Analysis Interface
export function DocumentAnalysis({ document, onTopicSelect }) {
  return (
    <div className="document-analysis">
      <h3>{document.title}</h3>

      <div className="summary-section">
        <h4>Summary</h4>
        <p>{document.summary}</p>

        <KeywordChips
          aiAnswer={document.summary}
          onChipClick={onTopicSelect}
          showCategories={true}
          maxChips={8}
          variant="default"
        />
      </div>

      <div className="key-insights">
        <h4>Key Insights</h4>
        <QuickKeywordChips
          text={document.insights}
          onChipClick={onTopicSelect}
        />
      </div>
    </div>
  );
}

// Example 3: Code Assistant Integration
export function CodeSuggestion({ code, feedback, onApplySuggestion }) {
  return (
    <div className="code-suggestion">
      <pre><code>{code}</code></pre>

      <div className="feedback-section">
        <p>{feedback}</p>

        <KeywordChips
          aiAnswer={feedback}
          onChipClick={(suggestion) => {
            // Apply code suggestion
            onApplySuggestion({
              type: 'improvement',
              suggestion: suggestion,
              context: code
            });
          }}
          maxChips={5}
          variant="expanded"
        />
      </div>
    </div>
  );
}

// Example 4: Learning Module Interface
export function LearningModule({ content, onExplore }) {
  return (
    <div className="learning-module">
      <div className="lesson-content">
        {content.sections.map((section, index) => (
          <div key={index} className="section">
            <h3>{section.title}</h3>
            <p>{section.text}</p>

            <KeywordChips
              aiAnswer={section.text}
              onChipClick={(topic) => {
                onExplore({
                  topic: topic,
                  section: index,
                  context: section.title
                });
              }}
              maxChips={6}
              showCategories={true}
              className="section-chips"
            />
          </div>
        ))}
      </div>
    </div>
  );
}

// Example 5: FAQ Enhancement
export function EnhancedFAQ({ faq, onRelatedQuestion }) {
  return (
    <div className="faq-item">
      <h3 className="question">{faq.question}</h3>
      <div className="answer">
        <p>{faq.answer}</p>

        <KeywordChips
          aiAnswer={faq.answer}
          onChipClick={(topic) => {
            // Find related FAQs
            onRelatedQuestion(topic);
          }}
          maxChips={4}
          variant="compact"
          className="related-topics"
        />
      </div>
    </div>
  );
}

// Example 6: Smart Search Interface
export function SmartSearchResults({ results, query, onRefineSearch }) {
  return (
    <div className="search-results">
      {results.map((result, index) => (
        <div key={index} className="result-item">
          <h4><a href={result.url}>{result.title}</a></h4>
          <p>{result.snippet}</p>

          <KeywordChips
            aiAnswer={result.snippet}
            onChipClick={(keyword) => {
              // Refine search with keyword
              const refinedQuery = `${query} ${keyword}`;
              onRefineSearch(refinedQuery);
            }}
            maxChips={3}
            variant="compact"
          />
        </div>
      ))}
    </div>
  );
}

// Example 7: Integration Hook for Chat State
export function useKeywordChat() {
  const [messages, setMessages] = useState([]);
  const [contextStack, setContextStack] = useState([]);

  const handleChipClick = (chipText, currentMessage) => {
    const newMessage = {
      text: chipText.includes('?') ? chipText : `Can you explain ${chipText}?`,
      isUser: true,
      timestamp: new Date(),
      context: {
        relatedTo: currentMessage,
        chipType: 'follow-up'
      }
    };

    setMessages(prev => [...prev, newMessage]);

    // Track context for better AI responses
    setContextStack(prev => [...prev, {
      query: chipText,
      source: 'keyword-chip',
      timestamp: Date.now()
    }]);
  };

  return {
    messages,
    handleChipClick,
    contextStack
  };
}

// Example 8: Analytics Integration
export function withKeywordAnalytics(Component) {
  return function WrappedComponent(props) {
    const handleChipClick = (chipText) => {
      // Track chip clicks for analytics
      if (typeof gtag !== 'undefined') {
        gtag('event', 'keyword_chip_click', {
          chip_text: chipText,
          component: Component.name
        });
      }

      // Custom analytics
      console.log('Chip clicked:', chipText);

      // Call original handler
      if (props.onChipClick) {
        props.onChipClick(chipText);
      }
    };

    return <Component {...props} onChipClick={handleChipClick} />;
  };
}

// Example: Usage with analytics wrapper
const TrackedKeywordChips = withKeywordAnalytics(KeywordChips);

// Example 9: Accessibility Enhancement
export function AccessibleKeywordChips(props) {
  return (
    <KeywordChips
      {...props}
      aria-label="Suggested topics for further exploration"
      role="navigation"
    />
  );
}

// Example 10: Theme-aware Chips
export function ThemedKeywordChips({ theme, ...props }) {
  const themeClasses = {
    light: 'light-theme',
    dark: 'dark-theme',
    high-contrast: 'high-contrast-theme'
  };

  return (
    <KeywordChips
      {...props}
      className={`keyword-chips ${themeClasses[theme] || themeClasses.light}`}
    />
  );
}

// Export a comprehensive example showing everything together
export function ComprehensiveExample() {
  const [activeTab, setActiveTab] = useState('chat');

  return (
    <div className="keyword-chips-demo">
      <nav className="demo-nav">
        <button onClick={() => setActiveTab('chat')}>Chat</button>
        <button onClick={() => setActiveTab('docs')}>Documents</button>
        <button onClick={() => setActiveTab('code')}>Code</button>
      </nav>

      {activeTab === 'chat' && (
        <ChatMessage
          message="To optimize React performance, consider using React.memo for expensive components and useCallback for functions passed to children."
          isAI={true}
          onFollowUp={(text) => console.log('Follow-up:', text)}
        />
      )}

      {activeTab === 'docs' && (
        <DocumentAnalysis
          document={{
            title: "Machine Learning Basics",
            summary: "Machine learning is a subset of AI that enables systems to learn from data. Key concepts include supervised learning, neural networks, and model evaluation.",
            insights: "Deep learning has revolutionized computer vision and natural language processing."
          }}
          onTopicSelect={(topic) => console.log('Selected topic:', topic)}
        />
      )}

      {activeTab === 'code' && (
        <CodeSuggestion
          code="const data = fetch('/api/data')"
          feedback="Consider adding error handling and loading states for better UX. Use async/await for cleaner code structure."
          onApplySuggestion={(suggestion) => console.log('Apply:', suggestion)}
        />
      )}
    </div>
  );
}