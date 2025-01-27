import { defineConfig } from 'vitepress'

// https://vitepress.dev/reference/site-config
export default defineConfig({
  title: "UFR",
  description: "A VitePress Site",


  themeConfig: {
    // https://vitepress.dev/reference/default-theme-config
    nav: [
      { text: 'Home', link: '/' },
      // { text: 'Examples', link: '/markdown-examples' }
    ],

    sidebar: [
      {
        text: 'Table of Contents',
        items: [
          { text: '0. Introduction', link: '/en/0-introduction' },
          { text: '1. Getting Started', link: '/en/1-getting-started' },
          { text: '2. Examples', link: '/2-examples' }
        ]   
      }
    ],

    socialLinks: [
      { icon: 'github', link: 'https://github.com/bombark/ufr' }
    ]
  },

  // locales
  locales: {
    en: {
      label: 'English',
      lang: 'en',
      themeConfig: {
        sidebar: [
          {
            text: 'Table of Contents',
            items: [
              { link: '/en/', text: '00. Intro' },
              { link: '/en/1-getting-started', text: '01. Getting Started' },
              { link: '/en/2-assembly', text: '02. RISC-V 101' },
              { link: '/en/2-examples', text: '03. Examples' },
            ]
          },
          {
            text: 'Links',
            items: [
              { link: 'https://github.com/nuta/operating-system-in-1000-lines', text: 'GitHub repository' },
              { link: '/pt', text: 'Portugues' },
            ]
          },
        ],
        socialLinks: [
          { icon: 'github', link: 'https://github.com/bombark/ufr' }
        ]
      }
    },
    pt: {
      label: 'Português',
      lang: 'pt',
      themeConfig: {
        sidebar: [
          {
            text: 'Conteudo',
            items: [
              { link: '/pt/', text: 'Introdução' },
              { link: '/pt/1-getting-started', text: '01. Primeiros Passos' },
              { link: '/pt/2-examples', text: '02. Exemplos' },
            ]
          },
          {
            text: 'リンク',
            items: [
              { link: '/en', text: 'English version' },
              { link: 'https://github.com/bombark/ufr', text: 'GitHub' },
            ]
          },
        ],
        socialLinks: [
          { icon: 'github', link: 'https://github.com/bombark/ufr' }
        ]
      }
    }
  }



})
