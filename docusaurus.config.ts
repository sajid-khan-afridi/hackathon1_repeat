import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'Comprehensive guide to ROS 2, Isaac Sim, and advanced robotics concepts',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://sajid-khan-afridi.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/hackathon1_repeat/',

  // GitHub pages deployment config.
  organizationName: 'sajid-khan-afridi', // Usually your GitHub org/user name.
  projectName: 'hackathon1_repeat', // Usually your repo name.

  onBrokenLinks: 'warn',
  onDuplicateRoutes: 'warn',
  onBrokenAnchors: 'ignore', // TODO: Fix frontmatter in docs files (chatbot-widget-integration.md, contributing/chapter-authoring.md, etc.)
markdown: {
  anchors: {
    maintainCase: false,
  },
  hooks: {
    onBrokenMarkdownLinks: 'warn',
  },
},

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // Custom fields accessible in client-side code (T076)
  customFields: {
    // API URL for backend services
    // Defaults to Railway production URL, can be overridden with API_URL env var
    apiUrl: process.env.API_URL || 'https://hackathon1repeat-production.up.railway.app',
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/sajid-khan-afridi/hackathon1_repeat/tree/main/docs/',
        },
        blog: false, // Disable blog for Phase 1 (textbook only)
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/robotics-textbook-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
      defaultMode: 'light',
    },
    navbar: {
      title: 'Robotics Textbook',
      logo: {
        alt: 'Robotics Textbook Logo',
        src: 'img/logo.svg',
      },
      hideOnScroll: false,
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Modules',
        },
        {
          to: '/docs/intro',
          label: 'Get Started',
          position: 'left',
        },
        {
          to: '/chatbot',
          label: '🤖 AI Assistant',
          position: 'left',
        },
        {
          href: 'https://github.com/sajid-khan-afridi/hackathon1_repeat',
          position: 'right',
          className: 'header-github-link',
          'aria-label': 'GitHub repository',
        },
      ],
    },
    footer: {
      style: 'light',
      links: [
        {
          title: 'Learning',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'ROS 2 Fundamentals',
              to: '/docs/module-1-ros2-fundamentals/chapter-1-publishers',
            },
            {
              label: 'NVIDIA Isaac Sim',
              to: '/docs/module-2-isaac-sim/chapter-1-introduction',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'ROS 2 Docs',
              href: 'https://docs.ros.org/en/rolling/',
            },
            {
              label: 'NVIDIA Isaac Sim',
              href: 'https://developer.nvidia.com/isaac-sim',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/sajid-khan-afridi/hackathon1_repeat',
            },
            {
              label: 'Issues',
              href: 'https://github.com/sajid-khan-afridi/hackathon1_repeat/issues',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'cpp', 'yaml', 'bash', 'json', 'docker'],
    },
  } satisfies Preset.ThemeConfig,

  // Client modules for browser-side configuration
  clientModules: [
    require.resolve('./src/clientModules/chatbotConfig.ts'),
  ],
};

export default config;
