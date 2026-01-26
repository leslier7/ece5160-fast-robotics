import { defineCollection, z } from 'astro:content';
import { glob } from 'astro/loaders';

const labs = defineCollection({
  loader: glob({ pattern: "**/*.{md,mdx}", base: "src/content/labs" }),
  schema: ({ image }) => z.object({
    title: z.string(),
    date: z.coerce.date(),
    cover: image().optional(),
    description: z.string().optional(),
  }),
});

export const collections = {
  labs,
};