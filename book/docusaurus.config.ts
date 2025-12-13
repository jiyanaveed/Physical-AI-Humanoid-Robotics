import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'From Theory to ROS 2: Building Intelligence That Moves.',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://your-docusaurus-site.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'jiyanaveed', // Usually your GitHub org/user name.
  projectName: 'Physical-AI-Humanoid-Robotics', // Usually your repo name.

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // File: book/docusaurus.config.js

presets: [
    [
      'classic',
      {
        docs: {
          path: 'docs', // <-- INSERTED: Explicitly points to the 'docs' subdirectory
          include: ['**/*.md', '**/*.mdx'], // <-- INSERTED: Ensures all content files are found
          sidebarPath: './sidebars.ts',
          routeBasePath: '/' ,
          lastVersion: 'current', 
          editUrl:
            'https://github.com/jiyanaveed/Physical-AI-Humanoid-Robotics/tree/main/book/', // (Assuming you updated this as well)
        },
      
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],
  plugins: [
    [
      '@docusaurus/plugin-client-redirects',
      {
        redirects: [
          {
            to: '/intro', // Redirects to your intro.md file based on its ID
            from: ['/'],
          },
        ],
      },
    ],
  ],
  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics', // Changed
      logo: {
        alt: 'Physical AI and Humanoid Robotics Logo', // Changed
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'bookSidebar', // <<< UPDATE THIS to match the new ID
          position: 'left',
          label: 'Book Modules',
        },
        // Blog item deleted
        {
          href: 'https://github.com/jiyanaveed/Physical-AI-Humanoid-Robotics', // Changed
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Book',
          items: [
            {
              label: 'Module 1: Simulation Introduction', // Changed
              to: '/module-1-simulation-sensors/intro', // <<< VERIFY THIS PATH
            },
          ],
        },
        // ... (Community links usually fine to keep)
        {
          title: 'Code',
          items: [
            // Blog link deleted
            {
              label: 'GitHub Repository',
              href: 'https://github.com/jiyanaveed/Physical-AI-Humanoid-Robotics', // Changed
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI and Humanoid Robotics. Built with Docusaurus.`, // Changed
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
