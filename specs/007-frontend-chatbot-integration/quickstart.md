# Quickstart: Frontend Chatbot Integration

## Prerequisites

- Node.js 16+ and npm/yarn
- Docusaurus 2.x project already set up
- Access to the RAG backend agent API (from Spec 3)
- Git

## Setup

### 1. Clone or Navigate to Your Docusaurus Project
```bash
cd your-docusaurus-project
```

### 2. Install Required Dependencies
```bash
npm install axios @types/react
# Or if using yarn:
yarn add axios @types/react
```

### 3. Project Structure
The chatbot component will be integrated into your existing Docusaurus structure:
```
your-docusaurus-project/
├── src/
│   ├── components/
│   │   └── Chatbot/
│   │       ├── Chatbot.tsx
│   │       ├── Chatbot.module.css
│   │       ├── ChatMessage.tsx
│   │       └── ChatInput.tsx
│   └── pages/
├── docusaurus.config.js
├── package.json
└── static/
```

## Configuration

### 1. Environment Variables
Create or update your `.env` file in the root directory:
```env
# RAG Agent API Configuration
REACT_APP_RAG_API_URL=https://your-rag-agent-api.com
REACT_APP_RAG_API_TIMEOUT=30000
```

### 2. API Client Configuration
Create an API client to communicate with the RAG agent:

```typescript
// src/utils/ragApiClient.ts
import axios from 'axios';

const ragApiUrl = process.env.REACT_APP_RAG_API_URL || 'http://localhost:8000';
const timeout = parseInt(process.env.REACT_APP_RAG_API_TIMEOUT || '30000', 10);

const ragApiClient = axios.create({
  baseURL: ragApiUrl,
  timeout: timeout,
  headers: {
    'Content-Type': 'application/json',
  },
});

export default ragApiClient;
```

## Integration

### 1. Add Chatbot Component to Layout
To make the chatbot available on all pages, add it to your Docusaurus layout:

```javascript
// src/theme/Layout/index.js
import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import Chatbot from '@site/src/components/Chatbot/Chatbot';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props}>
        {props.children}
        <Chatbot />
      </OriginalLayout>
    </>
  );
}
```

### 2. Or Add to Specific Pages
To add the chatbot to specific pages only, import it directly in your MDX files:

```markdown
<!-- In any .mdx file -->
import Chatbot from '@site/src/components/Chatbot/Chatbot';

<Chatbot />
```

## Running the Application

### 1. Start Your Docusaurus Development Server
```bash
npm run start
# Or if using yarn:
yarn start
```

### 2. Verify Installation
- Open your browser to http://localhost:3000
- The chatbot UI should be accessible on all pages
- Test by submitting a question and verifying it communicates with the RAG backend

## Testing the Chatbot

### 1. Basic Query Test
- Type a question in the chatbot input field
- Submit the query
- Verify that you receive a response with source attribution

### 2. Selected Text Test
- Select some text on the current page
- Use the chatbot's "Ask about selected text" feature
- Verify that the response is contextually relevant to the selected text

### 3. Error Handling Test
- Test with the RAG API temporarily unavailable
- Verify that appropriate error messages are displayed

## Customization

### Styling
The chatbot component can be customized by modifying the CSS module file:
- `src/components/Chatbot/Chatbot.module.css`
- Adjust colors, sizes, and positioning as needed

### Positioning
The chatbot can be positioned as:
- Floating button (expands to show chat interface)
- Sidebar element
- Inline component at the bottom of pages

## Troubleshooting

### Common Issues

1. **CORS Errors**: Ensure your RAG agent has proper CORS configuration to allow requests from your Docusaurus site
2. **API Timeout**: Increase the timeout value in environment variables if queries take longer than 30 seconds
3. **Component Not Loading**: Verify that the component is properly imported and that there are no JavaScript errors in the console

### Verification Steps
1. Check that the RAG agent API is accessible: `curl -X GET [REACT_APP_RAG_API_URL]/health`
2. Verify environment variables are set correctly
3. Check browser console for any JavaScript errors
4. Test API communication with a simple query