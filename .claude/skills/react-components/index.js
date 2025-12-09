/**
 * React Components Skill
 * Generates custom React components for Docusaurus book
 */

const fs = require('fs');
const path = require('path');

class ReactComponentsSkill {
  constructor() {
    this.templatesDir = path.join(__dirname, 'templates');
    this.outputDir = 'src/components';
  }

  /**
   * Main skill execution function
   */
  async execute(params) {
    try {
      const { component_type, props_schema = {}, styling = 'css_modules', accessibility_level = 'AA' } = params;

      // Validate inputs
      this.validateInputs({ component_type, styling, accessibility_level });

      // Validate props_schema
      if (props_schema && typeof props_schema !== 'object') {
        throw new Error('props_schema must be a valid object');
      }

      // Generate component based on type
      const generator = this.getComponentGenerator(component_type);
      const component = await generator.generate({
        props_schema,
        styling,
        accessibility_level
      });

      // Validate generated component
      this.validateGeneratedComponent(component, component_type);

      // Create output files
      const files = await this.createComponentFiles(component_type, component);

      return {
        success: true,
        component_type,
        files_created: files,
        message: `Successfully generated ${component_type} component`
      };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error occurred',
        component_type: params.component_type || 'unknown'
      };
    }
  }

  /**
   * Validate input parameters
   */
  validateInputs({ component_type, styling, accessibility_level }) {
    const validComponents = ['ChatbotWidget', 'PersonalizeButton', 'TranslateButton', 'AuthModal', 'TextSelector', 'ProfileBadge'];
    const validStyling = ['tailwind', 'css_modules', 'styled_components'];
    const validAccessibility = ['AA', 'AAA'];

    if (!validComponents.includes(component_type)) {
      throw new Error(`Invalid component_type. Must be one of: ${validComponents.join(', ')}`);
    }
    if (!validStyling.includes(styling)) {
      throw new Error(`Invalid styling. Must be one of: ${validStyling.join(', ')}`);
    }
    if (!validAccessibility.includes(accessibility_level)) {
      throw new Error(`Invalid accessibility_level. Must be one of: ${validAccessibility.join(', ')}`);
    }
  }

  /**
   * Get component generator class
   */
  getComponentGenerator(type) {
    const generators = {
      ChatbotWidget: require('./generators/ChatbotWidget'),
      PersonalizeButton: require('./generators/PersonalizeButton'),
      TranslateButton: require('./generators/TranslateButton'),
      AuthModal: require('./generators/AuthModal'),
      TextSelector: require('./generators/TextSelector'),
      ProfileBadge: require('./generators/ProfileBadge')
    };

    if (!generators[type]) {
      throw new Error(`No generator found for component type: ${type}`);
    }

    return new generators[type]();
  }

  /**
   * Validate generated component structure
   */
  validateGeneratedComponent(component, componentType) {
    if (!component || typeof component !== 'object') {
      throw new Error('Invalid component generated: expected object');
    }

    if (!component.code || typeof component.code !== 'string') {
      throw new Error('Invalid component generated: missing or invalid code');
    }

    // Check for required TypeScript syntax
    if (!component.code.includes('React') || !component.code.includes('interface')) {
      throw new Error('Invalid component generated: missing TypeScript React boilerplate');
    }

    // Check for accessibility features
    if (!component.code.includes('aria-') && !component.code.includes('role=')) {
      console.warn(`Warning: ${componentType} component may lack accessibility features`);
    }

    // Check component name consistency
    if (!component.code.includes(componentType)) {
      throw new Error(`Invalid component generated: component name ${componentType} not found in code`);
    }
  }

  /**
   * Create all necessary files for the component
   */
  async createComponentFiles(componentType, component) {
    const componentDir = path.join(this.outputDir, componentType);

    // Ensure directory exists
    if (!fs.existsSync(componentDir)) {
      fs.mkdirSync(componentDir, { recursive: true });
    }

    const files = [];

    try {
      // Create component file
      const componentFile = path.join(componentDir, 'index.tsx');
      fs.writeFileSync(componentFile, component.code);
      files.push(componentFile);

      // Create styles file
      if (component.styles) {
        const styleFile = path.join(componentDir, `${componentType}.module.css`);
        fs.writeFileSync(styleFile, component.styles);
        files.push(styleFile);
      }

      // Create test file
      if (component.test) {
        const testFile = path.join(componentDir, `${componentType}.test.tsx`);
        fs.writeFileSync(testFile, component.test);
        files.push(testFile);
      }

      // Create storybook file
      if (component.story) {
        const storyFile = path.join(componentDir, `${componentType}.stories.tsx`);
        fs.writeFileSync(storyFile, component.story);
        files.push(storyFile);
      }

      return files;
    } catch (error) {
      // Cleanup on failure
      if (fs.existsSync(componentDir)) {
        fs.rmSync(componentDir, { recursive: true, force: true });
      }
      throw error;
    }
  }
}

module.exports = ReactComponentsSkill;