# React Components Skill Examples

This document provides practical examples of using the React Components skill to generate various components for your Docusaurus book.

## Table of Contents

1. [Basic Examples](#basic-examples)
2. [Advanced Examples](#advanced-examples)
3. [Integration Examples](#integration-examples)
4. [Custom Props Examples](#custom-props-examples)

## Basic Examples

### Example 1: Simple ChatbotWidget

Generate a basic chatbot widget with default settings.

```javascript
const reactComponents = require('./react-components');

const result = await reactComponents.execute({
  component_type: 'ChatbotWidget',
  props_schema: {
    apiEndpoint: 'https://api.example.com/chat'
  }
});
```

Generated files:
- `src/components/ChatbotWidget/index.tsx`
- `src/components/ChatbotWidget/ChatbotWidget.module.css`
- `src/components/ChatbotWidget/ChatbotWidget.test.tsx`
- `src/components/ChatbotWidget/ChatbotWidget.stories.tsx`

### Example 2: TranslateButton for Urdu

Create a translation button that converts content to Urdu.

```javascript
const result = await reactComponents.execute({
  component_type: 'TranslateButton',
  styling: 'css_modules',
  accessibility_level: 'AA',
  props_schema: {
    chapterId: 'chapter-1',
    originalContent: 'This is the content to translate.',
    targetLanguage: 'ur'
  }
});
```

### Example 3: ProfileBadge for Logged-in Users

Generate a profile badge showing user information.

```javascript
const result = await reactComponents.execute({
  component_type: 'ProfileBadge',
  props_schema: {
    showLevel: true,
    user: {
      id: '123',
      name: 'Jane Doe',
      email: 'jane@example.com',
      experienceLevel: 'intermediate'
    }
  }
});
```

## Advanced Examples

### Example 4: AuthModal with Multiple OAuth Providers

Create an authentication modal with Google and GitHub login options.

```javascript
const result = await reactComponents.execute({
  component_type: 'AuthModal',
  styling: 'tailwind',
  accessibility_level: 'AAA',
  props_schema: {
    mode: 'signup',
    providers: ['google', 'github'],
    onSuccess: (user) => {
      console.log('User signed up:', user);
      // Redirect to dashboard
    },
    onClose: () => {
      console.log('Modal closed');
    }
  }
});
```

### Example 5: PersonalizeButton with Custom Levels

Generate a personalization button with custom user levels.

```javascript
const result = await reactComponents.execute({
  component_type: 'PersonalizeButton',
  accessibility_level: 'AAA',
  props_schema: {
    chapterId: 'advanced-react-concepts',
    userLevel: 'advanced',
    onPersonalize: (adaptedContent) => {
      // Apply personalized content to the page
      document.getElementById('chapter-content').innerHTML = adaptedContent;
    }
  }
});
```

### Example 6: TextSelector with Custom Handlers

Create a text selector that integrates with a chatbot.

```javascript
let chatbotRef = null;

const result = await reactComponents.execute({
  component_type: 'TextSelector',
  props_schema: {
    minLength: 5,
    maxLength: 200,
    onSelect: (selectedText, position) => {
      // Pass selected text to chatbot
      if (chatbotRef) {
        chatbotRef.addContext(selectedText);
        chatbotRef.ask(`What is "${selectedText}"?`);
      }
    }
  }
});
```

## Integration Examples

### Example 7: Complete Docusaurus Page Integration

Here's how to integrate multiple components in a Docusaurus page:

```jsx
// src/pages/custom-learning-page.jsx
import React, { useState, useRef } from 'react';
import Layout from '@theme/Layout';
import ChatbotWidget from '@site/src/components/ChatbotWidget';
import PersonalizeButton from '@site/src/components/PersonalizeButton';
import TranslateButton from '@site/src/components/TranslateButton';
import TextSelector from '@site/src/components/TextSelector';
import ProfileBadge from '@site/src/components/ProfileBadge';

export default function CustomLearningPage() {
  const [user, setUser] = useState(null);
  const [content, setContent] = useState(defaultContent);
  const chatbotRef = useRef(null);

  const handlePersonalize = async (adaptedContent) => {
    setContent(adaptedContent);
  };

  const handleTranslate = (translatedContent) => {
    setContent(translatedContent);
  };

  const handleTextSelect = (selectedText, position) => {
    // Context menu action
    if (chatbotRef.current) {
      chatbotRef.current.askAbout(selectedText);
    }
  };

  return (
    <Layout title="Interactive Learning">
      <div className="container margin-vert--lg">
        <header className="hero">
          <div className="hero__content">
            <h1>Interactive Learning Experience</h1>
            <p>Personalized, accessible, and multilingual content</p>
          </div>
          <div className="hero__actions">
            <ProfileBadge
              user={user}
              onLogin={() => setShowAuthModal(true)}
            />
          </div>
        </header>

        <div className="row">
          <div className="col col--8">
            <TextSelector onSelect={handleTextSelect}>
              <article id="chapter-content" className="markdown">
                {content}
              </article>
            </TextSelector>

            <div className="actions margin-vert--md">
              <PersonalizeButton
                chapterId="chapter-1"
                onPersonalize={handlePersonalize}
                disabled={!user}
              />
              <TranslateButton
                chapterId="chapter-1"
                originalContent={defaultContent}
                onTranslate={handleTranslate}
              />
            </div>
          </div>

          <div className="col col--4">
            <ChatbotWidget
              ref={chatbotRef}
              apiEndpoint="/api/chat"
              position="sidebar"
            />
          </div>
        </div>
      </div>
    </Layout>
  );
}
```

### Example 8: Theme-Aware Component Implementation

Implement components with dark mode support:

```jsx
// src/theme/NavbarItem/index.js
import React from 'react';
import useThemeContext from '@theme/hooks/useThemeContext';
import ProfileBadge from '@site/src/components/ProfileBadge';

export default function NavbarProfileItem() {
  const { isDarkTheme } = useThemeContext();
  const [user, setUser] = React.useState(null);

  return (
    <ProfileBadge
      user={user}
      showLevel={true}
      theme={isDarkTheme ? 'dark' : 'light'}
      onClick={() => {
        // Handle profile click
      }}
    />
  );
}
```

## Custom Props Examples

### Example 9: ChatbotWidget with Custom Configuration

```javascript
const result = await reactComponents.execute({
  component_type: 'ChatbotWidget',
  styling: 'styled_components',
  props_schema: {
    position: 'bottom-left',
    defaultOpen: false,
    apiEndpoint: 'https://my-api.com/ai-assistant',
    theme: 'dark',
    welcomeMessage: 'Hello! I\'m your AI learning assistant.',
    suggestions: [
      'Explain this concept',
      'Show examples',
      'Quiz me on this'
    ],
    maxMessages: 50,
    enableVoiceInput: true,
    customStyles: {
      primaryColor: '#6366f1',
      borderRadius: '12px'
    }
  }
});
```

### Example 10: AuthModal with Custom Validation

```javascript
const result = await reactComponents.execute({
  component_type: 'AuthModal',
  props_schema: {
    mode: 'signup',
    showTermsCheckbox: true,
    passwordRequirements: {
      minLength: 8,
      requireUppercase: true,
      requireNumbers: true,
      requireSpecialChars: true
    },
    customFields: [
      {
        name: 'organization',
        label: 'Organization',
        type: 'text',
        required: true
      },
      {
        name: 'role',
        label: 'Role',
        type: 'select',
        options: ['Student', 'Teacher', 'Professional', 'Other'],
        required: true
      }
    ],
    onCustomValidation: (formData) => {
      // Custom validation logic
      if (formData.organization === 'Acme Corp') {
        return { valid: false, message: 'Organization not allowed' };
      }
      return { valid: true };
    }
  }
});
```

### Example 11: PersonalizeButton with Analytics

```javascript
const result = await reactComponents.execute({
  component_type: 'PersonalizeButton',
  props_schema: {
    chapterId: 'machine-learning-basics',
    onPersonalize: (adaptedContent) => {
      // Track personalization event
      analytics.track('Content Personalized', {
        chapterId: 'machine-learning-basics',
        originalLength: content.length,
        adaptedLength: adaptedContent.length
      });

      // Update content
      setContent(adaptedContent);
    },
    analytics: {
      trackClicks: true,
      trackSuccess: true,
      trackErrors: true
    },
    apiConfig: {
      endpoint: '/api/personalize',
      timeout: 10000,
      retries: 2
    }
  }
});
```

### Example 12: TextSelector with Custom Actions

```javascript
const result = await reactComponents.execute({
  component_type: 'TextSelector',
  props_schema: {
    minLength: 3,
    maxLength: 300,
    actions: [
      {
        id: 'ask',
        label: 'Ask about this',
        icon: '❓',
        handler: (text) => {
          chatbotRef.current?.ask(text);
        }
      },
      {
        id: 'explain',
        label: 'Explain simpler',
        icon: '💡',
        handler: (text) => {
          personalizeFor('beginner', text);
        }
      },
      {
        id: 'translate',
        label: 'Translate',
        icon: '🌐',
        handler: (text) => {
          translateText(text, 'es');
        }
      },
      {
        id: 'bookmark',
        label: 'Bookmark',
        icon: '🔖',
        handler: (text) => {
          saveBookmark({ text, timestamp: Date.now() });
        }
      }
    ],
    showCharacterCount: true,
    highlightColor: '#fef3c7'
  }
});
```

## Error Handling Examples

### Example 13: Handling Generation Errors

```javascript
async function generateComponent(componentType, options = {}) {
  try {
    const result = await reactComponents.execute({
      component_type: componentType,
      ...options
    });

    if (result.success) {
      console.log(`✅ ${componentType} generated successfully!`);
      console.log('Files created:', result.files_created);

      // Show success notification
      showNotification({
        type: 'success',
        message: `${componentType} component ready!`
      });

      return result;
    } else {
      console.error(`❌ Failed to generate ${componentType}:`, result.error);

      // Show error notification
      showNotification({
        type: 'error',
        message: `Failed to generate ${componentType}: ${result.error}`
      });

      return null;
    }
  } catch (error) {
    console.error(`Unexpected error generating ${componentType}:`, error);

    // Show generic error notification
    showNotification({
      type: 'error',
      message: 'An unexpected error occurred. Please try again.'
    });

    return null;
  }
}

// Usage
generateComponent('ChatbotWidget', {
  apiEndpoint: '/api/chat'
});
```

### Example 14: Validation Error Handling

```javascript
async function validateAndGenerate(params) {
  const requiredFields = {
    ChatbotWidget: ['apiEndpoint'],
    PersonalizeButton: ['chapterId', 'onPersonalize'],
    TranslateButton: ['chapterId', 'originalContent'],
    AuthModal: ['onSuccess', 'onClose']
  };

  const { component_type } = params;
  const required = requiredFields[component_type];

  if (required) {
    const missing = required.filter(field => !params.props_schema || params.props_schema[field] === undefined);

    if (missing.length > 0) {
      return {
        success: false,
        error: `Missing required props for ${component_type}: ${missing.join(', ')}`
      };
    }
  }

  // Proceed with generation
  return await reactComponents.execute(params);
}
```

## Performance Optimization Examples

### Example 15: Lazy Loading Components

```jsx
import React, { lazy, Suspense } from 'react';

const ChatbotWidget = lazy(() => import('@site/src/components/ChatbotWidget'));
const AuthModal = lazy(() => import('@site/src/components/AuthModal'));

export default function OptimizedPage() {
  const [showChat, setShowChat] = useState(false);
  const [showAuth, setShowAuth] = useState(false);

  return (
    <div>
      {/* Main content */}
      <h1>Interactive Learning</h1>
      <p>This page loads components on demand.</p>

      {/* Lazy load chatbot */}
      <button onClick={() => setShowChat(true)}>Open Chat</button>
      {showChat && (
        <Suspense fallback={<div>Loading chat...</div>}>
          <ChatbotWidget apiEndpoint="/api/chat" />
        </Suspense>
      )}

      {/* Lazy load auth modal */}
      <button onClick={() => setShowAuth(true)}>Sign Up</button>
      {showAuth && (
        <Suspense fallback={<div>Loading sign up...</div>}>
          <AuthModal onSuccess={handleAuth} onClose={() => setShowAuth(false)} />
        </Suspense>
      )}
    </div>
  );
}
```

### Example 16: Bundle Splitting

```javascript
// webpack.config.js (for custom webpack setup)
module.exports = {
  optimization: {
    splitChunks: {
      chunks: 'all',
      cacheGroups: {
        components: {
          test: /[\\/]src[\\/]components[\\/]/,
          name: 'components',
          chunks: 'all',
          priority: 20,
        },
        vendor: {
          test: /[\\/]node_modules[\\/]/,
          name: 'vendors',
          chunks: 'all',
          priority: 10,
        },
      },
    },
  },
};
```

These examples demonstrate the flexibility and power of the React Components skill. You can mix and match these patterns to create rich, interactive learning experiences in your Docusaurus book.