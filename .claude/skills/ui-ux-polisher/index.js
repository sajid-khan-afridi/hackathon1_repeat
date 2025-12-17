/**
 * UI/UX Polisher Skill
 *
 * This is a prompt-based skill that guides Claude through UI/UX analysis
 * and improvement. The actual execution happens through SKILL.md instructions.
 *
 * This index.js provides the skill metadata and validation functions.
 */

const fs = require('fs');
const path = require('path');

class UIUXPolisherSkill {
  constructor() {
    this.name = 'ui-ux-polisher';
    this.version = '3.0.0';
    this.type = 'prompt';
    this.skillPath = path.join(__dirname, 'SKILL.md');
  }

  /**
   * Get the skill prompt content
   * @returns {string} The SKILL.md content
   */
  getPrompt() {
    if (fs.existsSync(this.skillPath)) {
      return fs.readFileSync(this.skillPath, 'utf-8');
    }
    throw new Error('SKILL.md not found');
  }

  /**
   * Validate that the skill is properly configured
   * @returns {object} Validation result
   */
  validate() {
    const requiredFiles = ['skill.json', 'README.md', 'SKILL.md'];
    const missingFiles = [];

    for (const file of requiredFiles) {
      const filePath = path.join(__dirname, file);
      if (!fs.existsSync(filePath)) {
        missingFiles.push(file);
      }
    }

    return {
      valid: missingFiles.length === 0,
      missingFiles,
      name: this.name,
      version: this.version
    };
  }

  /**
   * Get skill metadata
   * @returns {object} Skill metadata from skill.json
   */
  getMetadata() {
    const skillJsonPath = path.join(__dirname, 'skill.json');
    if (fs.existsSync(skillJsonPath)) {
      return JSON.parse(fs.readFileSync(skillJsonPath, 'utf-8'));
    }
    return null;
  }

  /**
   * Check if a user message should trigger this skill
   * @param {string} message - User's message
   * @returns {boolean} Whether the skill should be triggered
   */
  shouldTrigger(message) {
    const triggerPhrases = [
      'ui/ux polish',
      'polish the ui',
      'improve design',
      'professional look',
      'menu alignment',
      'navbar issues',
      'responsiveness',
      'mobile layout',
      'add animations',
      'accessibility improvements',
      'design consistency',
      'ui polish',
      'ux polish'
    ];

    const lowerMessage = message.toLowerCase();
    return triggerPhrases.some(phrase => lowerMessage.includes(phrase));
  }

  /**
   * Get the analysis checklist
   * @returns {object} Checklist items for UI/UX analysis
   */
  getChecklist() {
    return {
      alignment: [
        'Menu items vertically centered',
        'Icons aligned with text',
        'Button text centered',
        'Form labels aligned with inputs',
        'Grid/flex layouts use consistent gaps'
      ],
      spacing: [
        'Consistent spacing scale used (4, 8, 16, 24, 32px)',
        'Padding consistent across similar components',
        'No arbitrary spacing values',
        'Whitespace creates visual hierarchy'
      ],
      animations: [
        'Menu open/close smooth (150-250ms)',
        'Hover states have transitions',
        'No jarring state changes',
        'Reduced motion respected'
      ],
      accessibility: [
        'Focus indicators visible',
        'Touch targets >= 44px',
        'Color contrast >= 4.5:1',
        'Keyboard navigation works',
        'ARIA attributes present'
      ],
      responsive: [
        'Mobile (< 640px) works',
        'Tablet (640-1024px) works',
        'Desktop (>= 1024px) works',
        'No horizontal overflow'
      ]
    };
  }

  /**
   * Get CSS standards
   * @returns {object} CSS standards and tokens
   */
  getStandards() {
    return {
      animationTokens: {
        fast: '150ms',
        base: '250ms',
        slow: '350ms',
        easing: 'cubic-bezier(0.4, 0, 0.2, 1)'
      },
      spacingScale: [4, 8, 12, 16, 24, 32, 48, 64],
      breakpoints: {
        mobile: '640px',
        tablet: '1024px',
        desktop: '1440px'
      },
      accessibility: {
        minTouchTarget: '44px',
        minContrastRatio: 4.5,
        focusOutlineWidth: '2px'
      }
    };
  }
}

module.exports = UIUXPolisherSkill;
