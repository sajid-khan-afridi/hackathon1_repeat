# ChatbotWidget Component

AI-powered question answering chatbot widget for the robotics textbook Docusaurus site.

## Overview

The ChatbotWidget implements a complete RAG (Retrieval-Augmented Generation) interface with the following features:

- **Query Processing**: Submit natural language questions about robotics textbook content
- **Session Management**: Persistent conversation history across page reloads
- **Source Citations**: Clickable chapter links with relevance scores
- **Confidence Indicators**: Color-coded badges showing answer reliability
- **Accessibility**: WCAG 2.1 AA compliant (keyboard nav, ARIA labels, screen reader support)
- **Responsive Design**: Mobile-first with breakpoints for tablet (768px) and desktop (1024px)
- **Error Handling**: Graceful degradation with retry functionality
- **Educational Transparency**: Token usage display for all responses

## Component Structure

```
ChatbotWidget/
├── index.tsx                    # Main widget (useReducer state management)
├── types.ts                     # TypeScript interfaces matching backend models
├── ChatInput.tsx                # Textarea input with auto-resize and char counter
├── MessageList.tsx              # Scrollable message display with auto-scroll
├── SourceCitations.tsx          # Collapsible citations with clickable links
├── ConfidenceIndicator.tsx      # Color-coded confidence badge with tooltip
├── ChatbotWidget.module.css     # Responsive, accessible styles
├── ChatbotWidget.test.tsx       # React Testing Library unit tests
└── README.md                    # This file
```

## Installation

### 1. Backend Setup

Ensure the FastAPI backend is running with the `/api/v1/query` endpoint:

```bash
cd backend
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

### 2. Environment Configuration

Set the API base URL (optional, defaults to `http://localhost:8000`):

```javascript
// In Docusaurus config or client module
window.CHATBOT_API_URL = 'https://your-api-domain.com';
```

### 3. Import and Use

**Option A: Direct Import in MDX**

```mdx
---
title: Robotics Chapter 1
---

import ChatbotWidget from '@site/src/components/ChatbotWidget';

# Chapter 1: Introduction to Robotics

<ChatbotWidget />

Your chapter content here...
```

**Option B: Global Integration via Client Module**

Create `src/client-modules/chatbot.ts`:

```typescript
import { useEffect } from 'react';
import { useLocation } from '@docusaurus/router';

export function onRouteDidUpdate({ location }) {
  // Inject chatbot widget on all documentation pages
  if (location.pathname.startsWith('/docs/')) {
    const chatbotContainer = document.getElementById('chatbot-container');
    if (chatbotContainer) {
      import('@site/src/components/ChatbotWidget').then(({ default: ChatbotWidget }) => {
        ReactDOM.render(<ChatbotWidget />, chatbotContainer);
      });
    }
  }
}
```

Register in `docusaurus.config.ts`:

```typescript
export default {
  clientModules: ['./src/client-modules/chatbot'],
};
```

**Option C: Swizzle DocItem for All Pages**

```bash
npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --eject
```

Edit `src/theme/DocItem/Layout/index.tsx`:

```tsx
import ChatbotWidget from '@site/src/components/ChatbotWidget';

export default function DocItemLayout({ children }) {
  return (
    <>
      {children}
      <div style={{ marginTop: '2rem' }}>
        <ChatbotWidget />
      </div>
    </>
  );
}
```

## Usage

### Basic Usage

```tsx
import ChatbotWidget from '@site/src/components/ChatbotWidget';

<ChatbotWidget />;
```

### With Filters

```tsx
<ChatbotWidget moduleFilter={2} difficultyFilter="intermediate" />
```

## Props

| Prop               | Type                                         | Default     | Description                         |
| ------------------ | -------------------------------------------- | ----------- | ----------------------------------- |
| `moduleFilter`     | `number` (1-10)                              | `undefined` | Restrict queries to specific module |
| `difficultyFilter` | `'beginner' \| 'intermediate' \| 'advanced'` | `undefined` | Filter by difficulty level          |

## API Integration

### Request Format

```typescript
POST /api/v1/query
Content-Type: application/json

{
  "query": "What is inverse kinematics?",
  "session_id": "123e4567-e89b-12d3-a456-426614174000", // Optional
  "filters": {                                          // Optional
    "module": 2,
    "difficulty": "intermediate"
  },
  "top_k": 5
}
```

### Response Format

```typescript
{
  "answer": "Inverse kinematics is the process of determining joint angles...",
  "sources": [
    {
      "chapter_id": "module-1-chapter-2",
      "chapter_title": "Introduction to Kinematics",
      "relevance_score": 0.92,
      "excerpt": "Inverse kinematics calculates joint parameters...",
      "position": 1
    }
  ],
  "confidence": 0.85,
  "session_id": "123e4567-e89b-12d3-a456-426614174000",
  "tokens_used": {
    "input_tokens": 50,
    "output_tokens": 100,
    "total_tokens": 150
  },
  "filter_message": "Expanded search beyond Module 2 for better results",
  "suggested_terms": ["ROS", "kinematics", "motion planning"]
}
```

## Accessibility Features

### Keyboard Navigation

- **Tab**: Navigate through interactive elements
- **Enter**: Submit query (in textarea)
- **Shift+Enter**: Insert newline (in textarea)
- **Escape**: Close widget (if embedded in modal)

### Screen Reader Support

- **ARIA labels**: All interactive elements have descriptive labels
- **ARIA live regions**: Dynamic content changes are announced
- **Semantic HTML**: Proper use of `<article>`, `<form>`, `<button>`, etc.
- **Focus management**: Logical tab order and visible focus indicators

### WCAG 2.1 AA Compliance

- **Color contrast**: All text meets 4.5:1 ratio (7:1 for large text)
- **Touch targets**: 44x44px minimum for all interactive elements
- **Resize text**: Layout remains functional at 200% zoom
- **Motion sensitivity**: Respects `prefers-reduced-motion` setting
- **High contrast mode**: Enhanced borders and outlines

## Styling

### CSS Variables

The component uses CSS custom properties for theming:

```css
.chatbotWidget {
  --color-primary: #1a73e8;
  --color-text-primary: #202124;
  --color-bg-primary: #ffffff;
  /* ... see ChatbotWidget.module.css for full list */
}
```

### Dark Theme

Automatically adapts to Docusaurus dark theme:

```css
[data-theme='dark'] .chatbotWidget {
  --color-text-primary: #e8eaed;
  --color-bg-primary: #202124;
  /* ... */
}
```

### Customization

Override styles by targeting CSS module classes:

```css
/* In custom.css */
.chatbotWidget {
  --color-primary: #ff5722; /* Custom brand color */
}

.message-user .messageContent {
  background-color: var(--color-primary);
}
```

## State Management

### Chat State (useReducer)

```typescript
interface ChatState {
  messages: Message[];
  sessionId: string | null;
  isLoading: boolean;
  error: string | null;
}
```

### Actions

- `ADD_USER_MESSAGE`: Append user message to list
- `ADD_ASSISTANT_MESSAGE`: Append assistant response with metadata
- `SET_LOADING`: Toggle loading state
- `SET_ERROR`: Set error message
- `SET_SESSION_ID`: Update session identifier
- `CLEAR_HISTORY`: Reset all state

### Session Persistence

Session ID is stored in `localStorage` with key `chatbot-session-id`. This enables:

- Conversation continuity across page reloads
- Follow-up questions with context
- Chat history retrieval

## Testing

### Run Unit Tests

```bash
npm test -- ChatbotWidget.test.tsx
```

### Test Coverage

- Component rendering and state management
- User interactions (submit, clear history, retry)
- Accessibility features (ARIA, keyboard navigation)
- Error handling and loading states
- Session persistence
- API integration (mocked)

### Manual Testing Checklist

- [ ] Submit query and receive response
- [ ] View source citations and click chapter links
- [ ] Check confidence indicator tooltip
- [ ] Clear chat history
- [ ] Test keyboard navigation (Tab, Enter, Escape)
- [ ] Test with screen reader (NVDA, JAWS, VoiceOver)
- [ ] Verify responsive layout (mobile, tablet, desktop)
- [ ] Test dark theme compatibility
- [ ] Verify character limit (1000 chars)
- [ ] Test error handling (network failure, 429 rate limit)

## Performance Considerations

### Bundle Size

The component adds approximately:

- **TypeScript/React code**: ~15 KB (gzipped)
- **CSS**: ~5 KB (gzipped)
- **Total**: ~20 KB (minimal impact)

### Optimization Strategies

1. **Code Splitting**: Load ChatbotWidget asynchronously on demand
2. **Lazy Rendering**: Don't render until user clicks "Ask a Question" button
3. **Debounced Input**: Prevent excessive re-renders during typing
4. **Virtual Scrolling**: For long conversation histories (>100 messages)

### Example: Lazy Loading

```tsx
import React, { lazy, Suspense } from 'react';

const ChatbotWidget = lazy(() => import('@site/src/components/ChatbotWidget'));

export default function LazyChat() {
  return (
    <Suspense fallback={<div>Loading chatbot...</div>}>
      <ChatbotWidget />
    </Suspense>
  );
}
```

## Troubleshooting

### API Connection Issues

**Problem**: `Failed to fetch` error

**Solutions**:

- Verify backend is running: `curl http://localhost:8000/health`
- Check CORS configuration in FastAPI (allow frontend origin)
- Set correct `CHATBOT_API_URL` if not using `localhost:8000`

### Session Not Persisting

**Problem**: New session created on every page load

**Solutions**:

- Check browser's localStorage is enabled (not in private/incognito mode)
- Verify `chatbot-session-id` key exists: `localStorage.getItem('chatbot-session-id')`
- Check browser console for errors during session storage

### Styles Not Applied

**Problem**: Component renders but looks unstyled

**Solutions**:

- Ensure CSS module is imported: `import styles from './ChatbotWidget.module.css'`
- Verify Docusaurus CSS module support is enabled (default in v3.9+)
- Check for CSS variable conflicts with global styles

### Accessibility Warnings

**Problem**: Jest/Testing Library warnings about accessibility

**Solutions**:

- Run `npm run test` to see specific warnings
- Verify all interactive elements have ARIA labels
- Check color contrast ratios with browser DevTools

## Contributing

When modifying this component:

1. **Update Types**: Ensure `types.ts` matches backend models
2. **Maintain Accessibility**: Test with keyboard and screen reader
3. **Add Tests**: Cover new functionality in `ChatbotWidget.test.tsx`
4. **Document Changes**: Update this README with new props/behavior
5. **Check Performance**: Monitor bundle size and render performance

## References

- **Specification**: `specs/003-rag-chatbot-core/spec.md`
- **Backend Models**: `backend/app/models/query.py`
- **API Docs**: `specs/003-rag-chatbot-core/contracts/openapi.yaml`
- **WCAG 2.1 Guidelines**: https://www.w3.org/WAI/WCAG21/quickref/
- **React Testing Library**: https://testing-library.com/docs/react-testing-library/intro/

## License

MIT License - See project root LICENSE file for details.
