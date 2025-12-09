# Contextual Keyword Chips

A skill that implements intelligent keyword chips beneath AI-generated answers, enabling users to quickly explore related topics with a single tap.

## Features

- **Smart Keyword Extraction**: Uses NLP to identify entities, concepts, actions, and technical terms
- **Dynamic Follow-up Generation**: Creates contextual questions based on the AI response
- **Rich Visual Feedback**: Color-coded chips with icons for different content types
- **Smooth Animations**: Framer Motion integration for polished interactions
- **Customizable**: Configure chip count, appearance, and behavior
- **Category View**: Optional grouping of chips by type

## Installation

```bash
npm install compromise framer-motion clsx
```

## Quick Start

### Basic Usage

```tsx
import { KeywordChips } from './KeywordChips';

function ChatMessage({ aiMessage }) {
  return (
    <div>
      <p>{aiMessage}</p>
      <KeywordChips
        aiAnswer={aiMessage}
        onChipClick={(chipText) => {
          // Handle chip click
          console.log('User clicked:', chipText);
        }}
      />
    </div>
  );
}
```

### With Hook

```tsx
import { useKeywordExtraction } from './useKeywordExtraction';

function KeywordSuggestion({ text }) {
  const { keywords, isLoading } = useKeywordExtraction(text, {
    maxChips: 4
  });

  if (isLoading) return <div>Loading suggestions...</div>;

  return (
    <div>
      {keywords.map((keyword, index) => (
        <button key={index} onClick={() => alert(keyword.text)}>
          {keyword.text}
        </button>
      ))}
    </div>
  );
}
```

## Configuration Options

### KeywordChips Props

| Prop | Type | Default | Description |
|------|------|---------|-------------|
| `aiAnswer` | string | - | The AI response text to analyze |
| `onChipClick` | function | - | Callback when chip is clicked |
| `maxChips` | number | 5 | Maximum number of chips to display |
| `variant` | `'default' \| 'compact' \| 'expanded'` | `'default'` | Chip size variant |
| `showCategories` | boolean | false | Group chips by type |
| `animated` | boolean | true | Enable animations |
| `className` | string | - | Additional CSS classes |

### useKeywordExtraction Options

```tsx
const { keywords, categories, isLoading } = useKeywordExtraction(text, {
  maxChips: 5,
  minWordLength: 3,
  excludeWords: ['custom', 'stop', 'words'],
  debounceMs: 300
});
```

## Chip Types

Chips are automatically categorized and styled:

- 🏷️ **Entities**: Named entities (people, places, organizations)
- 💡 **Concepts**: Nouns and ideas
- ⚡ **Actions**: Tasks and operations
- ❓ **Questions**: Follow-up queries
- ⚙️ **Technologies**: Frameworks, libraries, tools
- 📍 **Proper Nouns**: Capitalized terms
- ✨ **Adjectives**: Descriptive terms

## Integration Examples

### Chat Application

```tsx
function ChatInterface() {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');

  const handleChipClick = (chipText) => {
    const followUp = `Tell me more about: ${chipText}`;
    setInputValue(followUp);
    // Optionally auto-send
    sendMessage(followUp);
  };

  return (
    <div>
      {messages.map((msg, index) => (
        <div key={index} className={msg.isAI ? 'ai-message' : 'user-message'}>
          <p>{msg.text}</p>
          {msg.isAI && (
            <KeywordChips
              aiAnswer={msg.text}
              onChipClick={handleChipClick}
              variant="compact"
            />
          )}
        </div>
      ))}
    </div>
  );
}
```

### Document Assistant

```tsx
function DocumentAssistant({ document }) {
  return (
    <div>
      <h2>Document Analysis</h2>
      <p>{document.summary}</p>

      <KeywordChips
        aiAnswer={document.summary}
        onChipClick={(topic) => {
          // Search document for topic
          searchInDocument(topic);
        }}
        showCategories={true}
        maxChips={8}
      />
    </div>
  );
}
```

### Code Review Assistant

```tsx
function CodeReviewFeedback({ review }) {
  return (
    <div>
      <div className="review-text">{review.feedback}</div>

      <KeywordChips
        aiAnswer={review.feedback}
        onChipClick={(suggestion) => {
          // Get code examples for suggestion
          getCodeExample(suggestion);
        }}
        variant="expanded"
      />
    </div>
  );
}
```

## Advanced Usage

### Custom Styling

```css
.keyword-chip {
  /* Custom chip styles */
  border-radius: 8px;
  font-weight: 600;
}

.keyword-chip.tech {
  background: linear-gradient(45deg, #667eea, #764ba2);
  color: white;
}
```

### Custom Extractor

```javascript
import { KeywordExtractor } from './keywordExtractor';

const customExtractor = new KeywordExtractor({
  maxChips: 10,
  minWordLength: 2,
  excludeWords: ['custom', 'stop', 'words']
});

const keywords = customExtractor.extractKeywords(text);
```

## Performance Considerations

- Extraction is debounced by default (300ms) to avoid unnecessary re-computation
- Large texts are automatically truncated for performance
- Consider memoizing the `onChipClick` callback in React

## Browser Support

- Modern browsers with ES6+ support
- Chrome 70+, Firefox 65+, Safari 12+, Edge 79+

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests
5. Submit a pull request

## License

MIT License - see LICENSE file for details