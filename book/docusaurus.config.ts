import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'From Theory to ROS 2: Building Intelligence That Moves.',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  // future: {
  //   v4: true, // Improve compatibility with the upcoming Docusaurus v4
  // },

  // Set the production url of your site here
  url: 'https://your-docusaurus-site.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'jiyanaveed', // Usually your GitHub org/user name.
  projectName: 'Physical-AI-Humanoid-Robotics', // Usually your repo name.

  onBrokenLinks:  'warn', // Change 'throw' (the default) to 'warn' or 'ignore',

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
          path: 'docs', // <-- Corrected: Explicitly points to the 'docs' subdirectory
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
  
  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      logo: {
        alt: 'Physical AI and Humanoid Robotics Logo', // Changed
        src: 'img/logo.svg',
        href: '/',
      },
      items: [
        {
          type: 'doc',
          docId: 'module-1-ros2-fundamentals/chapter-1-1-nodes-topics-services',
          position: 'left',
          label: 'Book Modules',
        },
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
              label: 'Module 1: ROS 2 Fundamentals', // Updated label for clarity
              to: '/module-1-ros2-fundamentals/chapter-1-1-nodes-topics-services',
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
      theme: prismThemes.vsDark,
      darkTheme: prismThemes.vsDark,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
