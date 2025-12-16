# Quickstart: Advanced UI/UX Redesign for Spec-Driven Docusaurus Book

## Prerequisites

- Node.js 16+ and npm/yarn
- Docusaurus 2.x project already set up
- Git
- Basic knowledge of React and TypeScript

## Setup

### 1. Clone or Navigate to Your Docusaurus Project
```bash
cd your-docusaurus-project
```

### 2. Install Required Dependencies
```bash
npm install @docusaurus/core @docusaurus/module-type-aliases @docusaurus/types
npm install --save-dev @docusaurus/preset-classic
# If using Tailwind CSS
npm install -D tailwindcss postcss autoprefixer
npx tailwindcss init -p
```

### 3. Project Structure
The UI/UX redesign will be implemented in the following structure:
```
your-docusaurus-project/
├── src/
│   ├── components/           # Custom React components
│   │   └── Chatbot/          # Chatbot UI components
│   ├── css/                  # Custom styles and Tailwind
│   │   ├── custom.css        # Main custom styles
│   │   └── tailwind.css      # Tailwind imports
│   ├── theme/                # Custom theme overrides
│   │   ├── Layout/           # Layout wrapper components
│   │   ├── MDXComponents/    # Custom MDX component overrides
│   │   └── [other theme overrides]
│   └── pages/                # Custom pages if needed
├── docusaurus.config.js      # Main Docusaurus configuration
├── sidebars.js               # Sidebar navigation configuration
├── package.json              # Project dependencies
└── static/                   # Static assets
```

## Configuration

### 1. Environment Variables
Create or update your `.env` file in the root directory:
```env
# Theme Configuration
REACT_APP_THEME_PRIMARY_COLOR=#1b6dfc
REACT_APP_THEME_SECONDARY_COLOR=#0d5ae5

# Chatbot Configuration
REACT_APP_CHATBOT_ENABLED=true
REACT_APP_CHATBOT_API_URL=https://your-rag-api.com
```

### 2. Tailwind CSS Configuration
If using Tailwind CSS, configure `tailwind.config.js`:
```javascript
/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    "./src/**/*.{js,jsx,ts,tsx}",
    "./src/pages/**/*.{js,jsx,ts,tsx}",
    "./src/components/**/*.{js,jsx,ts,tsx}",
    "./src/theme/**/*.{js,jsx,ts,tsx}",
    "./docs/**/*.{md,mdx}",
    "./blog/**/*.{md,mdx}",
    "./pages/**/*.{md,mdx}",
  ],
  theme: {
    extend: {
      colors: {
        primary: {
          50: '#eff6ff',
          100: '#dbeafe',
          200: '#bfdbfe',
          300: '#93c5fd',
          400: '#60a5fa',
          500: '#3b82f6',
          600: '#2563eb',
          700: '#1d4ed8',
          800: '#1e40af',
          900: '#1e3a8a',
        }
      }
    },
  },
  plugins: [],
}
```

## Implementation

### 1. Theme Customization
Override Docusaurus theme components by creating files in `src/theme/`:
- Create custom layout components in `src/theme/Layout/`
- Customize MDX components in `src/theme/MDXComponents/`
- Add custom CSS in `src/css/custom.css`

### 2. Typography System
Implement a consistent typography system in your custom CSS:
```css
/* Base typography */
html {
  font-size: 100%; /* 16px */
}

body {
  font-family: system-ui, -apple-system, sans-serif;
  line-height: 1.6;
  color: var(--ifm-font-color-base);
}

/* Heading scales */
h1 { font-size: 2.5rem; } /* 40px */
h2 { font-size: 2rem; }   /* 32px */
h3 { font-size: 1.75rem; } /* 28px */
h4 { font-size: 1.5rem; }  /* 24px */
h5 { font-size: 1.25rem; } /* 20px */
h6 { font-size: 1rem; }    /* 16px */

/* Reading text */
.markdown {
  font-size: 1.125rem; /* 18px */
  line-height: 1.7;
}
```

### 3. Content Container Design
Style content containers for optimal reading experience:
```css
/* Main content container */
.container {
  max-width: 75ch; /* Optimal reading width */
  margin: 0 auto;
  padding: 0 1.5rem;
}

/* Different content type styling */
.content-narrative {
  background: var(--ifm-background-color);
  padding: 1.5rem;
  border-radius: var(--ifm-global-radius);
}

.content-spec {
  background: rgba(27, 109, 252, 0.05); /* Light blue tint */
  border-left: 4px solid #1b6dfc;
  padding: 1.5rem;
  margin: 1.5rem 0;
}

.content-code {
  background: var(--ifm-code-background);
  border-radius: var(--ifm-global-radius);
  padding: 1rem;
  overflow-x: auto;
}
```

### 4. Chatbot Integration
Integrate the chatbot component following the architecture described in the data model:
1. Create the chatbot component in `src/components/Chatbot/`
2. Implement docked and expanded states
3. Add text selection capture functionality
4. Integrate with the RAG API

## Running the Application

### 1. Start Your Docusaurus Development Server
```bash
npm run start
# Or if using yarn:
yarn start
```

### 2. Verify Installation
- Open your browser to http://localhost:3000
- Check that the new UI elements are visible
- Test the chatbot functionality if implemented
- Verify responsive design on different screen sizes

## Customization Points

### Theme Configuration
The following aspects can be customized:
- Color palette (primary, secondary, accents)
- Typography scale and font families
- Spacing system and layout grids
- Component styles and visual hierarchy
- Responsive breakpoints

### Component Overrides
You can customize any Docusaurus component by creating:
- A file with the same name in `src/theme/`
- The custom component will automatically override the default

### CSS Variables
Docusaurus provides many CSS variables for theming:
- `--ifm-color-primary`: Primary brand color
- `--ifm-color-emphasis-0`: Background colors
- `--ifm-color-emphasis-100`: Card backgrounds
- `--ifm-font-size-base`: Base font size
- `--ifm-spacing-horizontal`: Horizontal spacing

## Troubleshooting

### Common Issues

1. **Custom Styles Not Applying**: Verify that your custom CSS is imported in the Docusaurus config
2. **Responsive Design Issues**: Check that your breakpoints match the design requirements
3. **Chatbot Integration Problems**: Ensure API endpoints are correctly configured
4. **Performance Issues**: Optimize images and minimize heavy JavaScript

### Verification Steps
1. Check that all custom components are properly exported
2. Verify that theme configuration is correctly set in `docusaurus.config.js`
3. Test all interactive elements across different browsers
4. Validate that accessibility requirements are met
5. Confirm that the design works across all specified form factors