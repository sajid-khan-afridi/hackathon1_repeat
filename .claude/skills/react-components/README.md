# React Components Skill

A powerful skill for generating custom React components for Docusaurus books with built-in accessibility, TypeScript support, and comprehensive testing.

## Overview

This skill generates production-ready React components including:
- **ChatbotWidget** - Interactive chat interface with message history and context
- **PersonalizeButton** - Content personalization based on user experience level
- **TranslateButton** - Translation toggle with RTL support for Urdu
- **AuthModal** - Multi-step authentication with OAuth providers
- **TextSelector** - Smart text selection with floating action popup
- **ProfileBadge** - User profile display with dropdown menu

## Usage

### Basic Usage

```javascript
const reactComponents = require('./react-components');

// Generate a ChatbotWidget component
const result = await reactComponents.execute({
  component_type: 'ChatbotWidget',
  styling: 'css_modules',
  accessibility_level: 'AA',
  props_schema: {
    apiEndpoint: 'https://api.example.com/chat'
  }
});

console.log(result);
// Output: { success: true, component_type: 'ChatbotWidget', files_created: [...] }
```

### Advanced Usage

```javascript
// Generate a PersonalizeButton with custom props
const result = await reactComponents.execute({
  component_type: 'PersonalizeButton',
  styling: 'tailwind',
  accessibility_level: 'AAA',
  props_schema: {
    chapterId: 'chapter-1',
    userLevel: 'beginner',
    showTooltip: true
  }
});
```

## Component Types

### 1. ChatbotWidget

A collapsible chat interface with message history, typing indicators, and source citations.

**Features:**
- Message history with scroll
- Typing indicators
- Selected text context display
- Source citations with links
- Copy response button
- Feedback buttons (thumbs up/down)
- Multiple position options
- Theme support (light/dark/auto)

**Props:**
```typescript
interface ChatbotWidgetProps {
  position?: 'bottom-right' | 'bottom-left' | 'sidebar';
  defaultOpen?: boolean;
  apiEndpoint: string;
  theme?: 'light' | 'dark' | 'auto';
}
```

### 2. PersonalizeButton

Adapts content based on user's experience level.

**Features:**
- Shows user profile level badge
- Loading state during API call
- Success/error feedback
- Tooltip explaining personalization
- Disabled state for logged-out users

**Props:**
```typescript
interface PersonalizeButtonProps {
  chapterId: string;
  onPersonalize: (adaptedContent: string) => void;
  disabled?: boolean;
  userLevel?: 'beginner' | 'intermediate' | 'advanced';
}
```

### 3. TranslateButton

Provides translation toggle for content with Urdu language support.

**Features:**
- Toggle between original/translated
- RTL layout handling
- Preserves code blocks in LTR
- Loading state
- Caches translated content
- Mobile-responsive

**Props:**
```typescript
interface TranslateButtonProps {
  chapterId: string;
  originalContent: string;
  onTranslate: (translatedContent: string) => void;
  targetLanguage?: 'ur';
}
```

### 4. AuthModal

Multi-step authentication modal with OAuth provider support.

**Features:**
- Tab switch between signin/signup
- Email/password form
- OAuth provider buttons
- Profile questions (signup only)
- Form validation
- Error display
- Remember me checkbox

**Props:**
```typescript
interface AuthModalProps {
  mode?: 'signin' | 'signup';
  onSuccess: (user: User) => void;
  onClose: () => void;
  providers?: string[];
}
```

### 5. TextSelector

Smart text selection detector with floating action popup.

**Features:**
- Detect text selection in content
- Show floating "Ask about this" button
- Pass selected text to ChatbotWidget
- Highlight selected text
- Clear selection on click outside
- Keyboard accessible

**Props:**
```typescript
interface TextSelectorProps {
  onSelect: (selectedText: string, position: DOMRect) => void;
  minLength?: number;
  maxLength?: number;
}
```

### 6. ProfileBadge

User profile display with dropdown menu.

**Features:**
- Avatar display (initials or image)
- Experience level indicator
- Dropdown menu (profile, settings, logout)
- Login prompt for guests
- Mobile-responsive

**Props:**
```typescript
interface ProfileBadgeProps {
  user: User | null;
  showLevel?: boolean;
  onClick?: () => void;
}
```

## Configuration Options

### Styling Options

- **css_modules** (default) - Uses CSS modules for scoped styling
- **tailwind** - Uses Tailwind CSS utility classes
- **styled_components** - Uses styled-components

### Accessibility Levels

- **AA** (default) - WCAG 2.1 AA compliance
- **AAA** - WCAG 2.1 AAA compliance (enhanced accessibility)

### Props Schema

Define additional props for your component:

```javascript
const props_schema = {
  customProp: {
    type: 'string',
    required: true,
    description: 'Custom property description'
  },
  optionalProp: {
    type: 'number',
    default: 10,
    description: 'Optional numeric property'
  }
};
```

## Generated Files

For each component, the skill generates:

1. **Component file** (`index.tsx`) - TypeScript React component
2. **Style file** (`ComponentName.module.css`) - CSS modules styling
3. **Test file** (`ComponentName.test.tsx`) - Jest + React Testing Library tests
4. **Storybook file** (`ComponentName.stories.tsx`) - Storybook stories

## File Structure

```
src/components/
├── ChatbotWidget/
│   ├── index.tsx
│   ├── ChatbotWidget.module.css
│   ├── ChatbotWidget.test.tsx
│   └── ChatbotWidget.stories.tsx
├── PersonalizeButton/
│   └── ...
├── TranslateButton/
│   └── ...
├── AuthModal/
│   └── ...
├── TextSelector/
│   └── ...
└── ProfileBadge/
    └── ...
```

## Integration with Docusaurus

1. Register components in `src/theme/` for MDX access
2. Use `@docusaurus/theme-common` hooks
3. Components respect `colorMode` context
4. Heavy components are lazy-loaded

## Performance Requirements

- Bundle size < 50KB per component (gzipped)
- First paint < 100ms
- Interaction latency < 50ms
- Uses React.memo for expensive renders

## Testing

Each generated component includes comprehensive tests covering:
- Component rendering
- User interactions
- Accessibility features
- Error states
- Keyboard navigation

Run tests:
```bash
npm test src/components/[ComponentName]/[ComponentName].test.tsx
```

## Accessibility

All components include:
- ARIA labels on interactive elements
- Keyboard navigation support
- Focus management for modals
- Screen reader announcements
- Color contrast ratio ≥ 4.5:1

## Example Implementation

Here's a complete example of generating and using a ChatbotWidget:

```javascript
// Generate the component
const result = await reactComponents.execute({
  component_type: 'ChatbotWidget',
  apiEndpoint: 'https://api.example.com/chat',
  position: 'bottom-right',
  theme: 'auto'
});

// Use in your Docusaurus page
import ChatbotWidget from '@site/src/components/ChatbotWidget';

function MyPage() {
  return (
    <>
      <YourContent />
      <ChatbotWidget
        apiEndpoint="/api/chat"
        position="bottom-right"
        theme="auto"
      />
    </>
  );
}
```

## Error Handling

The skill provides comprehensive error handling:

```javascript
const result = await reactComponents.execute(params);

if (result.success) {
  console.log('Generated files:', result.files_created);
} else {
  console.error('Generation failed:', result.error);
  console.log('Component type:', result.component_type);
}
```

## Best Practices

1. **Always use TypeScript interfaces** for props
2. **Implement proper error boundaries** in your components
3. **Test accessibility features** with screen readers
4. **Optimize bundle size** by lazy-loading heavy components
5. **Use semantic HTML** for better accessibility
6. **Implement proper loading states** for better UX

## Contributing

To add new component types:

1. Create a new generator in `generators/[ComponentName].js`
2. Extend `BaseGenerator` class
3. Implement required methods: `generate()`, `generateComponentCode()`, etc.
4. Add to the component type mapping in `index.js`
5. Update documentation

## License

MIT License - see LICENSE file for details.