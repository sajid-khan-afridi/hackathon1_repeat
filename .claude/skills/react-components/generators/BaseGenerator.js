/**
 * Base class for component generators
 */
class BaseGenerator {
  constructor() {
    this.accessibilityFeatures = {
      AA: {
        colorContrast: 4.5,
        focusVisible: true,
        keyboardNavigation: true,
        ariaLabels: true,
        screenReaderSupport: true
      },
      AAA: {
        colorContrast: 7.0,
        focusVisible: true,
        keyboardNavigation: true,
        ariaLabels: true,
        screenReaderSupport: true,
        reducedMotion: true,
        highContrastMode: true
      }
    };
  }

  /**
   * Generate component - to be implemented by subclasses
   */
  async generate(params) {
    throw new Error('generate method must be implemented by subclass');
  }

  /**
   * Generate React imports
   */
  generateImports({ hasState = false, hasEffects = false, hasRef = false, hasCallback = false, styling = 'css_modules' }) {
    let imports = "import React";

    const hooks = [];
    if (hasState) hooks.push("useState");
    if (hasEffects) hooks.push("useEffect");
    if (hasRef) hooks.push("useRef");
    if (hasCallback) hooks.push("useCallback");

    if (hooks.length > 0) {
      imports += `, { ${hooks.join(', ')} }`;
    }
    imports += " from 'react';\n";

    if (styling === 'css_modules') {
      imports += `import styles from './{{componentName}}.module.css';\n`;
    }

    return imports + "\n";
  }

  /**
   * Generate TypeScript interfaces
   */
  generateInterfaces(componentType, props) {
    const interfaces = [];

    // Common interfaces
    if (componentType === 'ChatbotWidget') {
      interfaces.push(`
interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: Array<{
    title: string;
    url: string;
  }>;
}`);
    }

    if (componentType === 'ProfileBadge') {
      interfaces.push(`
interface User {
  id: string;
  name: string;
  email: string;
  avatar?: string;
  experienceLevel: 'beginner' | 'intermediate' | 'advanced';
}`);
    }

    // Props interface
    const propsInterface = this.generatePropsInterface(componentType, props);
    interfaces.push(propsInterface);

    return interfaces.join('\n');
  }

  /**
   * Generate props interface for component type
   */
  generatePropsInterface(componentType, customProps = {}) {
    const baseProps = this.getBaseProps(componentType);
    const allProps = { ...baseProps, ...customProps };

    const propsList = Object.entries(allProps)
      .map(([key, config]) => {
        const { type, optional = false, description } = config;
        const optionalStr = optional ? '?' : '';
        return `  /** ${description} */\n  ${key}${optionalStr}: ${type};`;
      })
      .join('\n');

    return `interface ${componentType}Props {\n${propsList}\n}`;
  }

  /**
   * Get base props for each component type
   */
  getBaseProps(componentType) {
    const propsMap = {
      ChatbotWidget: {
        position: {
          type: "'bottom-right' | 'bottom-left' | 'sidebar'",
          optional: true,
          description: "Position of the chatbot widget"
        },
        defaultOpen: {
          type: "boolean",
          optional: true,
          description: "Whether the chatbot is open by default"
        },
        apiEndpoint: {
          type: "string",
          description: "API endpoint for chatbot requests"
        },
        theme: {
          type: "'light' | 'dark' | 'auto'",
          optional: true,
          description: "Theme of the chatbot"
        }
      },
      PersonalizeButton: {
        chapterId: {
          type: "string",
          description: "ID of the chapter to personalize"
        },
        onPersonalize: {
          type: "(adaptedContent: string) => void",
          description: "Callback when personalization is complete"
        },
        disabled: {
          type: "boolean",
          optional: true,
          description: "Whether the button is disabled"
        }
      },
      TranslateButton: {
        chapterId: {
          type: "string",
          description: "ID of the chapter to translate"
        },
        originalContent: {
          type: "string",
          description: "Original content to translate"
        },
        onTranslate: {
          type: "(translatedContent: string) => void",
          description: "Callback with translated content"
        },
        targetLanguage: {
          type: "'ur'",
          optional: true,
          description: "Target language for translation"
        }
      },
      AuthModal: {
        mode: {
          type: "'signin' | 'signup'",
          optional: true,
          description: "Authentication mode"
        },
        onSuccess: {
          type: "(user: User) => void",
          description: "Callback on successful authentication"
        },
        onClose: {
          type: "() => void",
          description: "Callback when modal is closed"
        },
        providers: {
          type: "string[]",
          optional: true,
          description: "OAuth providers to show"
        }
      },
      TextSelector: {
        onSelect: {
          type: "(selectedText: string, position: DOMRect) => void",
          description: "Callback when text is selected"
        },
        minLength: {
          type: "number",
          optional: true,
          description: "Minimum selection length"
        },
        maxLength: {
          type: "number",
          optional: true,
          description: "Maximum selection length"
        }
      },
      ProfileBadge: {
        user: {
          type: "User | null",
          optional: true,
          description: "User object"
        },
        showLevel: {
          type: "boolean",
          optional: true,
          description: "Whether to show experience level"
        },
        onClick: {
          type: "() => void",
          optional: true,
          description: "Click handler"
        }
      }
    };

    return propsMap[componentType] || {};
  }

  /**
   * Generate accessibility attributes
   */
  generateAccessibilityAttributes(level = 'AA') {
    const features = this.accessibilityFeatures[level];
    const attrs = [];

    if (features.ariaLabels) {
      attrs.push('aria-label');
      attrs.push('aria-describedby');
    }

    if (features.keyboardNavigation) {
      attrs.push('tabIndex={0}');
      attrs.push('onKeyDown');
    }

    if (features.screenReaderSupport) {
      attrs.push('role');
      attrs.push('aria-live');
    }

    return attrs;
  }

  /**
   * Generate CSS styles based on styling approach
   */
  generateStyles(componentType, styling) {
    if (styling === 'css_modules') {
      return this.generateCSSModuleStyles(componentType);
    } else if (styling === 'tailwind') {
      return this.generateTailwindClasses();
    }
    return '';
  }

  /**
   * Generate CSS module styles
   */
  generateCSSModuleStyles(componentType) {
    const baseStyles = `
/* Base styles */
.container {
  display: flex;
  flex-direction: column;
}

/* Button styles */
.button {
  padding: 8px 16px;
  border: none;
  border-radius: 6px;
  cursor: pointer;
  font-size: 14px;
  font-weight: 500;
  transition: all 0.2s ease;
  min-height: 44px; /* WCAG AA touch target */
  min-width: 44px;
}

.button:focus {
  outline: 2px solid #0066cc;
  outline-offset: 2px;
}

.button:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

/* Dark mode support */
[data-theme='dark'] .button {
  background-color: #333;
  color: #fff;
}

/* Mobile responsive */
@media (max-width: 768px) {
  .container {
    width: 100%;
  }
}`;

    return baseStyles;
  }

  /**
   * Generate Tailwind CSS classes
   */
  generateTailwindClasses() {
    return `
    className="flex flex-col p-2 rounded-lg border border-gray-200 dark:border-gray-700
              hover:bg-gray-50 dark:hover:bg-gray-800 focus:outline-none focus:ring-2
              focus:ring-blue-500 min-h-[44px] min-w-[44px] transition-all duration-200"
    `;
  }
}

module.exports = BaseGenerator;