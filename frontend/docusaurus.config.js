// @ts-check
// `@ts-check` enables type checking for this file.

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'A comprehensive guide to building embodied AI systems',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
    url: 'https://seher-robotic-book.netlify.app',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',
  // For GitHub Pages deployment
  trailingSlash: true, // For GitHub Pages compatibility
  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'seher873', // Usually your GitHub org/user name.
  projectName: 'seher_ai_book', // Usually your repo name.

  onBrokenLinks: 'ignore',
  onBrokenMarkdownLinks: 'warn',
  markdown: {
    mermaid: true,
    mdx1Compat: {
      comments: true,
      admonitions: true,
      headingIds: true,
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
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/seher873/seher_ai_book/edit/main/',
        },
        blog: false, // Disable blog if not needed
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themes: [
    '@docusaurus/theme-mermaid',
  ],
  plugins: [
    [
      '@docusaurus/plugin-client-redirects',
      {
        fromExtensions: ['html', 'htm'],
        redirects: [
          {
            to: '/',
            from: ['/index'],
          },
        ],
      },
    ],
  ],
  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/robot.png',
      navbar: {
        title: 'Physical AI Textbook',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logoo.png',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            type: 'doc',
            docId: 'resume',
            position: 'left',
            label: 'Resume',
          },
          {
            to: '/dashboard',
            label: 'Dashboard',
            position: 'left',
          },
          {
            to: '/docs/assessments',
            label: 'Assessments',
            position: 'left',
          },
          {
            to: '/docs/labs',
            label: 'Labs',
            position: 'left',
          },
          {
            to: '/docs/hardware-guidelines',
            label: 'Hardware',
            position: 'left',
          },
          {
            to: '/docs/glossary',
            label: 'Glossary',
            position: 'left',
          },
          {
            to: '/docs/appendices',
            label: 'Appendices',
            position: 'left',
          },
          {
            to: '/auth/login',
            label: 'Login',
            position: 'right',
          },
          {
            to: '/auth/signup',
            label: 'Sign Up',
            position: 'right',
          },
          {
            href: 'https://github.com/seher873/seher_ai_book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Textbook',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
              {
                label: 'Chapter 1',
                to: '/docs/ch1-introduction/sub1-what-is-physical-ai',
              },
              {
                label: 'All Chapters',
                to: '/docs/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/docusaurus',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/docusaurus',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/seher873/seher_ai_book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
      },
    }),
};

module.exports = config;