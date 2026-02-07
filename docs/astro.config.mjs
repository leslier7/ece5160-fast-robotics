// @ts-check
import { defineConfig } from 'astro/config';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';

import tailwindcss from '@tailwindcss/vite';
import react from '@astrojs/react';

// https://astro.build/config
export default defineConfig({
  site: "https://leslier7.github.io",
  base: "/ece5160-fast-robotics",
  compressHTML: true,
  build: {
    inlineStylesheets: 'always',
    assetsInlineLimit: 10240,
  },
  vite: {
    plugins: [tailwindcss()],
    build: {
      assetsInlineLimit: 10240,
    }
  },
  markdown: {
    remarkPlugins: [remarkMath],
    rehypePlugins: [rehypeKatex],
  },

  integrations: [react()]
});