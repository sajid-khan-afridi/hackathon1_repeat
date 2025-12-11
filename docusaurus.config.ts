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

  onBrokenLinks: 'throw',
onDuplicateRoutes: 'warn',
markdown: {
  anchors: {
    maintainCase: false,
  },
  hooks: {
    onBrokenMarkdownLinks: 'throw',
  },
},

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
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
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Modules',
        },
        {
          href: 'https://github.com/sajid-khan-afridi/hackathon1_repeat',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learning Modules',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub Repository',
              href: 'https://github.com/sajid-khan-afridi/hackathon1_repeat',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'cpp', 'yaml', 'bash', 'json', 'docker'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
