# React Components Skill - Quick Start Guide

Get up and running with the React Components skill in under 5 minutes!

## Prerequisites

- Node.js 14+ installed
- A Docusaurus project (or any React project)
- Basic knowledge of React and TypeScript

## Installation

1. **Navigate to your project directory:**
   ```bash
   cd your-docusaurus-project
   ```

2. **Ensure the skill is available in your `.claude/skills` directory**

## Quick Examples

### Generate Your First Component

```javascript
// In your Node.js script or REPL
const ReactComponentsSkill = require('./.claude/skills/react-components');
const skill = new ReactComponentsSkill();

// Generate a ChatbotWidget
const result = await skill.execute({
  component_type: 'ChatbotWidget',
  props_schema: {
    apiEndpoint: 'https://api.example.com/chat'
  }
});

console.log(result);
```

### Available Component Types

| Component | Use Case | Props Required |
|-----------|----------|----------------|
| `ChatbotWidget` | AI chat interface | `apiEndpoint` |
| `PersonalizeButton` | Content personalization | `chapterId`, `onPersonalize` |
| `TranslateButton` | Text translation | `chapterId`, `originalContent` |
| `AuthModal` | User authentication | `onSuccess`, `onClose` |
| `TextSelector` | Text selection detection | `onSelect` |
| `ProfileBadge` | User profile display | (optional) `user` |

## Basic Usage Patterns

### 1. Add a Chatbot to Your Page

```javascript
// Generate the component
await skill.execute({
  component_type: 'ChatbotWidget',
  props_schema: {
    apiEndpoint: '/api/chat',
    position: 'bottom-right'
  }
});

// Use it in your React component
import ChatbotWidget from './src/components/ChatbotWidget';

function MyPage() {
  return (
    <div>
      <h1>My Page</h1>
      <p>Page content here...</p>
      <ChatbotWidget apiEndpoint="/api/chat" />
    </div>
  );
}
```

### 2. Add Content Personalization

```javascript
// Generate the component
await skill.execute({
  component_type: 'PersonalizeButton',
  props_schema: {
    chapterId: 'intro-to-react',
    userLevel: 'beginner'
  }
});

// Use it
import PersonalizeButton from './src/components/PersonalizeButton';

function ChapterPage() {
  const [content, setContent] = useState(defaultContent);

  return (
    <div>
      <PersonalizeButton
        chapterId="intro-to-react"
        onPersonalize={setContent}
        userLevel="beginner"
      />
      <div>{content}</div>
    </div>
  );
}
```

### 3. Add Translation Support

```javascript
// Generate the component
await skill.execute({
  component_type: 'TranslateButton',
  props_schema: {
    chapterId: 'chapter-1',
    targetLanguage: 'ur'
  }
});

// Use it
import TranslateButton from './src/components/TranslateButton';

function BilingualChapter() {
  const [text, setText] = useState(englishText);

  return (
    <div>
      <TranslateButton
        chapterId="chapter-1"
        originalContent={englishText}
        onTranslate={setText}
      />
      <div>{text}</div>
    </div>
  );
}
```

## Configuration Options

### Styling Options

```javascript
// Choose your styling approach
await skill.execute({
  component_type: 'ChatbotWidget',
  styling: 'css_modules',  // or 'tailwind' or 'styled_components'
  props_schema: {
    apiEndpoint: '/api/chat'
  }
});
```

### Accessibility Level

```javascript
// Set accessibility compliance
await skill.execute({
  component_type: 'AuthModal',
  accessibility_level: 'AAA',  // or 'AA' (default)
  props_schema: {
    onSuccess: handleSuccess,
    onClose: handleClose
  }
});
```

## Common Patterns

### Pattern 1: Authentication Flow

```javascript
// 1. Generate AuthModal
await skill.execute({
  component_type: 'AuthModal',
  props_schema: {
    providers: ['google', 'github']
  }
});

// 2. Generate ProfileBadge
await skill.execute({
  component_type: 'ProfileBadge',
  props_schema: {
    showLevel: true
  }
});

// 3. Use together
import AuthModal from './src/components/AuthModal';
import ProfileBadge from './src/components/ProfileBadge';

function App() {
  const [user, setUser] = useState(null);
  const [showAuth, setShowAuth] = useState(false);

  return (
    <div>
      {user ? (
        <ProfileBadge user={user} />
      ) : (
        <button onClick={() => setShowAuth(true)}>Sign In</button>
      )}

      {showAuth && (
        <AuthModal
          onSuccess={setUser}
          onClose={() => setShowAuth(false)}
        />
      )}
    </div>
  );
}
```

### Pattern 2: Interactive Learning Page

```javascript
// Generate all needed components
await skill.execute({
  component_type: 'TextSelector',
  props_schema: {
    onSelect: (text) => chatbotRef.current?.ask(text)
  }
});

await skill.execute({
  component_type: 'PersonalizeButton',
  props_schema: {
    chapterId: 'react-hooks'
  }
});

await skill.execute({
  component_type: 'ChatbotWidget',
  props_schema: {
    position: 'sidebar'
  }
});

// Use together
import TextSelector from './src/components/TextSelector';
import PersonalizeButton from './src/components/PersonalizeButton';
import ChatbotWidget from './src/components/ChatbotWidget';

function LearningPage() {
  const chatbotRef = useRef(null);

  return (
    <div className="learning-page">
      <div className="content">
        <TextSelector onSelect={(text) => chatbotRef.current?.ask(text)}>
          <article>Learning content here...</article>
        </TextSelector>
        <PersonalizeButton chapterId="react-hooks" />
      </div>
      <ChatbotWidget ref={chatbotRef} position="sidebar" />
    </div>
  );
}
```

## What Gets Generated?

For each component, you'll get:

```
src/components/[ComponentName]/
├── index.tsx              # React component with TypeScript
├── [ComponentName].module.css    # Styles (if using CSS modules)
├── [ComponentName].test.tsx      # Unit tests
└── [ComponentName].stories.tsx   # Storybook stories
```

## Next Steps

1. **Explore all components** - Check the full documentation for each component's features
2. **Customize props** - Add your own props schema for custom behavior
3. **Run tests** - Each component comes with tests, run them with `npm test`
4. **View stories** - Use Storybook to see components in action
5. **Integrate with Docusaurus** - Follow the integration guide for Docusaurus-specific setup

## Need Help?

- Check the full [README.md](./README.md) for detailed documentation
- Browse [examples.md](./examples.md) for advanced use cases
- Each generated component includes inline documentation and examples

## Troubleshooting

### Common Issues

1. **"No generator found" error**
   - Ensure component_type is spelled correctly (case-sensitive)
   - Check that you're using one of: ChatbotWidget, PersonalizeButton, TranslateButton, AuthModal, TextSelector, ProfileBadge

2. **File permission errors**
   - Ensure your `src/components` directory is writable
   - Run with appropriate permissions

3. **TypeScript errors**
   - Make sure you have TypeScript installed: `npm install typescript @types/react @types/react-dom`
   - Check your tsconfig.json includes the src directory

### Quick Debug

```javascript
const result = await skill.execute({
  component_type: 'ChatbotWidget',
  props_schema: {
    apiEndpoint: '/api/chat'
  }
});

if (!result.success) {
  console.error('Error:', result.error);
  console.error('Component type:', result.component_type);
} else {
  console.log('Success! Files created:', result.files_created);
}
```

That's it! You're ready to create amazing interactive components with the React Components skill. 🚀