# Frontend Setup Guide - Phase 9: Chat Interface

## Overview

The RAG chatbot frontend is fully integrated with the Docusaurus documentation site. Users can:
- Click the floating "AI Assistant" button to open the chat
- Ask questions about Physical AI & Humanoid Robotics
- Select text on any page and ask for explanations
- View conversation history (stored in localStorage for 7 days)
- See source citations for AI responses

## Components Structure

```
book-source/src/
├── types/
│   └── chat.ts                    # TypeScript types for chat
├── services/
│   └── chatApi.ts                 # Backend API client
├── hooks/
│   ├── useConversationHistory.ts  # LocalStorage management
│   ├── useChatState.ts            # Chat state & API calls
│   └── useTextSelection.ts        # Text selection detection
├── components/
│   ├── ChatInterface.tsx          # Main chat container
│   ├── ChatInterface.module.css
│   ├── ChatMessage.tsx            # Individual messages
│   ├── ChatMessage.module.css
│   ├── ChatInput.tsx              # Input field
│   └── ChatInput.module.css
└── theme/
    ├── Root.tsx                   # Docusaurus wrapper (floating button)
    └── Root.module.css
```

## Local Development

### 1. Install Dependencies

```bash
cd book-source
npm install
```

### 2. Configure Backend URL

Create `.env.local` in `book-source/`:

```bash
# For local development (backend running on port 8001)
REACT_APP_BACKEND_URL=http://localhost:8001
```

Or copy from example:

```bash
cp .env.example .env.local
```

### 3. Start Backend Server

In a separate terminal, start the FastAPI backend:

```bash
cd backend
source venv/bin/activate  # or .\venv\Scripts\activate on Windows
uvicorn app.main:app --port 8001 --reload
```

### 4. Start Docusaurus Dev Server

```bash
cd book-source
npm start
```

The site will open at http://localhost:3000

### 5. Test Chat Interface

1. Click the purple "AI Assistant" button in bottom-right corner
2. Try these test queries:
   - "What is ROS2?"
   - "Explain sensor fusion"
   - "How does NVIDIA Isaac Sim work?"
3. Test text selection:
   - Navigate to any documentation page
   - Select some text
   - Click the AI Assistant button
   - Ask "Explain this" or similar

## Production Deployment

### GitHub Pages (Frontend)

The frontend is already deployed at:
https://anusbutt.github.io/hackathon-phase-01/

To update:

```bash
cd book-source
npm run build
npm run deploy
```

### Backend URL Configuration

After deploying backend to Railway (Phase 8), update the backend URL:

**Option 1: Environment Variable (Recommended)**

Set in GitHub Pages deployment settings or CI/CD:

```bash
REACT_APP_BACKEND_URL=https://your-app.railway.app
```

**Option 2: Update docusaurus.config.ts**

Edit `book-source/docusaurus.config.ts`:

```typescript
customFields: {
  backendUrl: 'https://your-app.railway.app',
},
```

Then rebuild and redeploy.

## Features

### 1. Multi-Turn Conversations

- Last 5 messages kept as context
- Conversation ID tracked automatically
- 7-day expiry (stored in browser localStorage)

### 2. Selected Text Explanations

- Select 10-2000 characters of text on any page
- Chat input shows "Explaining selected text" banner
- AI focuses on explaining the selected content
- +3 chunks retrieved, -0.1 similarity threshold for better context

### 3. Source Citations

- Every AI response includes source links
- Shows: Module → Lesson → Section
- Displays similarity scores (0-100% match)

### 4. Error Handling

- Network errors: "Failed to connect to backend"
- Rate limits: "Too many requests, try again later"
- Out-of-scope queries: Helpful suggestions for valid topics
- Service errors: User-friendly error messages

### 5. Responsive Design

- Desktop: Full chat overlay (800px wide, 700px tall)
- Mobile: Full-screen chat interface
- Dark mode support (follows Docusaurus theme)

## Troubleshooting

### "Failed to connect to backend"

1. Check backend is running on port 8001
2. Verify REACT_APP_BACKEND_URL is correct
3. Check browser console for CORS errors
4. Ensure backend CORS_ORIGINS includes frontend URL

### Chat button not appearing

1. Verify `src/theme/Root.tsx` exists
2. Clear browser cache and reload
3. Check browser console for React errors

### Text selection not working

1. Ensure selected text is 10-2000 characters
2. Try selecting text in main content area
3. Check browser console for errors

### Conversation not persisting

1. Check browser allows localStorage
2. Verify conversation hasn't expired (7 days)
3. Clear localStorage and start new conversation:
   ```javascript
   localStorage.removeItem('hackathon_chat_conversation')
   ```

## API Endpoints Used

### Health Check
```
GET http://localhost:8001/api/health
```

### Chat Query
```
POST http://localhost:8001/api/chat/query
Content-Type: application/json

{
  "query": "What is ROS2?",
  "selected_text": null,
  "lesson_id": null,
  "conversation_history": [],
  "conversation_id": null,
  "conversation_created_at": null
}
```

## Browser Support

- Chrome/Edge: ✅ Fully supported
- Firefox: ✅ Fully supported
- Safari: ✅ Fully supported
- Mobile browsers: ✅ Responsive design

## Performance

- Initial load: ~200KB (gzipped)
- Chat interface lazy-loaded on button click
- LocalStorage used for conversation persistence (no database needed)
- API calls debounced to prevent spam

## Security

- Backend URL configurable (not hardcoded)
- No secrets in frontend code
- CORS protection on backend
- Rate limiting on backend (100 req/min)
- Input validation (1-2000 characters)

## Next Steps

1. **Complete Phase 8**: Deploy backend to Railway
2. **Update Backend URL**: Set REACT_APP_BACKEND_URL to Railway URL
3. **Test End-to-End**: Verify chat works with production backend
4. **Monitor Usage**: Check Railway logs for errors
5. **Iterate**: Gather user feedback and improve

## Related Documentation

- **Backend Setup**: `backend/README.md`
- **Railway Deployment**: `backend/RAILWAY_DEPLOYMENT.md`
- **Cross-Module Search**: `backend/CROSS_MODULE_SEARCH.md`
- **Deployment Checklist**: `backend/DEPLOYMENT_CHECKLIST.md`

---

**Phase 9 Status**: ✅ Complete

**Last Updated**: 2026-01-02
