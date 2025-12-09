import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Getting Started',
      items: [
        'getting-started/installation',
        'getting-started/configuration',
        'getting-started/quick-start',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: Core Concepts',
      items: [
        'module-1/introduction',
        'module-1/basic-usage',
        'module-1/advanced-concepts',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Advanced Features',
      items: [
        'module-2/introduction',
        'module-2/customization',
        'module-2/performance',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Integration',
      items: [
        'module-3/introduction',
        'module-3/third-party',
        'module-3/workflows',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Deployment',
      items: [
        'module-4/introduction',
        'module-4/preparation',
        'module-4/deployment',
      ],
    },
  ],

  apiSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Authentication',
      items: [
        'auth/overview',
        'auth/endpoints',
      ],
    },
    {
      type: 'category',
      label: 'Resources',
      items: [
        'resources/users',
        'resources/projects',
      ],
    },
  ],
};

export default sidebars;