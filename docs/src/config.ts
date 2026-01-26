import heroImage from './assets/hero-real.jpg';

const BASE = '/ece5160-fast-robotics';

export const SITE = {
  website: 'https://leslier7.github.io/ece5160-fast-robotics/', // Replace with your actual deployed URL
  author: 'Robbie Leslie',
  description: 'Webpage for ECE5160.',
  title: 'ECE5160 - Fast Robotics',
  ogImage: 'astropaper-og.jpg',
  lightAndDarkMode: true,
  postPerPage: 3,
  scheduledPostMargin: 15 * 60 * 1000, // 15 minutes
  
  // Lab Info
  labName: 'ECE 5160 - Fast Robotics',
  university: 'Cornell University',
  logo: `${BASE}/assets/cornell_logo.svg`,
  avatar: `${BASE}/assets/cornell_logo.svg`,
  email: 'rwl228@cornell.edu', // Contact email for Join Us page
  github: 'https://github.com/leslier7',
  linkedin: 'https://www.linkedin.com/in/robbie-leslie/',

  // Hero Section - About Me
  hero: {
    title: 'Robbie Leslie',
    subtitle: 'Welcome to my ECE 5160 Fast Robotics portfolio. Here you\'ll find my lab reports.',
    action: 'View Labs',
    image: heroImage,
    about: `I'm a MEng student in the Electrical and Computer Engineering department. I'm interested in robotics, embedded systems, computer architecture, IoT, and digital design generally. `
  },

  // Navigation
  nav: [
    { text: 'Home', link: `${BASE}/`, key: 'home' },
    { text: 'Labs', link: `${BASE}/labs`, key: 'labs' },
  ],

  // Custom Pages (Appended after 'Join Us')
  customPages: [
    // Example: { text: 'Alumni', link: '/alumni', key: 'alumni' }
  ],
  
  // i18n Config
  i18n: {
    enabled: false,
    defaultLocale: 'zh',
  }
};

export const LOCALE = {
  lang: 'en', // html lang code. Set this empty and default will be "en"
  langTag: ['en-EN'], // BCP 47 Language Tags. Set this empty [] to use the environment default
} as const;

export const LOGO_IMAGE = {
  enable: true,
  svg: true,
  width: 216,
  height: 46,
};

export const SOCIALS = [
  {
    link: 'https://github.com/fjd2004711/scholar-lite',
    active: true,
  },
];

// Default language configuration
export const DEFAULT_LANG: 'zh' | 'en' | 'ja' | 'ko' | 'fr' | 'de' | 'es' | 'ru' = 'en'; 
