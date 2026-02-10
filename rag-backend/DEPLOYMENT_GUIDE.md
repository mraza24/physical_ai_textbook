# RAG Chatbot - Hugging Face Spaces Deployment Guide

## Overview

This guide will help you deploy your upgraded RAG Chatbot with a professional glassmorphism UI to Hugging Face Spaces.

---

## Part 1: Backend Deployment (FastAPI on Hugging Face Spaces)

### Prerequisites

- Hugging Face account
- Backend code in this repository
- Environment variables configured

### Step 1: Create Hugging Face Space

1. Go to https://huggingface.co/new-space
2. Fill in details:
   - **Space name**: `physical-ai-backend` (or your choice)
   - **License**: Apache 2.0
   - **SDK**: Docker
   - **Hardware**: CPU Basic (free tier)

### Step 2: Prepare Repository

Your backend already has:
- ✅ `Dockerfile` (optimized for port 7860)
- ✅ `requirements.txt`
- ✅ `app/` directory with FastAPI code

### Step 3: Configure Environment Variables

In your Hugging Face Space settings, add these secrets:

```bash
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
```

### Step 4: Push to Hugging Face

```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend

# Initialize git if not already done
git init

# Add Hugging Face remote
git remote add hf https://huggingface.co/spaces/YOUR_USERNAME/physical-ai-backend

# Add all files
git add .

# Commit
git commit -m "Deploy RAG backend with optimized Dockerfile"

# Push to Hugging Face
git push hf main
```

### Step 5: Verify Deployment

1. Go to your Space URL: `https://YOUR_USERNAME-physical-ai-backend.hf.space`
2. Check the logs in the Space's "Logs" tab
3. Test the health endpoint: `https://YOUR_USERNAME-physical-ai-backend.hf.space/health`
4. Test the API: `https://YOUR_USERNAME-physical-ai-backend.hf.space/docs` (FastAPI docs)

---

## Part 2: Frontend Integration (Docusaurus)

### Step 1: Update Backend URL

In `textbook/src/components/RAGChatbot/index.tsx`, update line 105:

```typescript
const API_BASE_URL = 'https://YOUR_USERNAME-physical-ai-backend.hf.space';
```

### Step 2: Integrate Component

If not already integrated, add to your Docusaurus theme:

**Option A: Global Integration (Recommended)**

Edit `textbook/src/theme/Root.tsx`:

```tsx
import React from 'react';
import RAGChatbot from '@site/src/components/RAGChatbot';

export default function Root({ children }) {
  return (
    <>
      {children}
      <RAGChatbot />
    </>
  );
}
```

**Option B: Page-Specific Integration**

Import in any page:

```tsx
import RAGChatbot from '@site/src/components/RAGChatbot';

export default function MyPage() {
  return (
    <div>
      {/* Your page content */}
      <RAGChatbot />
    </div>
  );
}
```

### Step 3: Build and Test Locally

```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/textbook

# Install dependencies
npm install

# Run dev server
npm run start

# Test the chatbot
# Click the floating button in bottom-right corner
```

### Step 4: Deploy Frontend

**Vercel Deployment:**

```bash
# If using Vercel
npm run build
vercel --prod
```

**Netlify Deployment:**

```bash
# Build
npm run build

# Deploy
netlify deploy --prod --dir=build
```

---

## Part 3: Features Implemented

### ✅ Dockerfile

**Location**: `rag-backend/Dockerfile`

**Features**:
- Optimized Python 3.11 slim base image
- Port 7860 exposed (Hugging Face Spaces requirement)
- Multi-stage build for smaller image size
- Non-root user for security
- Health check endpoint
- Uvicorn with single worker (Hugging Face free tier)

**Build locally (optional)**:
```bash
cd rag-backend
docker build -t rag-backend .
docker run -p 7860:7860 --env-file .env rag-backend
```

### ✅ Upgraded index.tsx

**Location**: `textbook/src/components/RAGChatbot/index.tsx`

**Features**:

1. **Typewriter Effect**:
   - AI responses appear character-by-character
   - Configurable speed (20ms per character)
   - Smooth, ChatGPT-like streaming

2. **Citations Support**:
   - Displays `data.citations` from backend
   - Clickable blue badges
   - Links to `deep_link_url` sections
   - Hover effects and smooth transitions

3. **Professional Structure**:
   - TypeScript interfaces for type safety
   - Modular components (MessageBubble, Citations, TypewriterText)
   - Custom hooks (useTypewriter)
   - BrowserOnly compatibility maintained
   - ReactDOM.createPortal preserved

4. **UX Enhancements**:
   - Auto-scroll to latest message
   - Auto-focus input when chat opens
   - Loading indicator with animated dots
   - Welcome message for first-time users
   - Disabled states during typing/loading

### ✅ styles.module.css

**Location**: `textbook/src/components/RAGChatbot/styles.module.css`

**Features**:

1. **ChatGPT-Style Dark Mode**:
   - Deep slate/navy palette (`rgba(17, 25, 40, ...)`)
   - High contrast for readability
   - Subtle gradients for depth

2. **Glassmorphism**:
   - `backdrop-filter: blur(16px)` for frosted glass effect
   - Semi-transparent backgrounds
   - Layered depth with multiple blur levels
   - Border glow effects

3. **Smooth Animations**:
   - `slideUp` animation for chat window entrance
   - `messageSlideIn` for new messages
   - Pulsing ring on FAB button
   - Bouncing dots for loading indicator
   - Hover/active state transitions

4. **Modern Design Elements**:
   - Gradient floating action button (FAB)
   - Rounded corners (16-20px border-radius)
   - Custom scrollbar styling
   - Blue citation badges with hover effects
   - Responsive mobile layout

5. **Color Palette**:
   - Primary: `#667eea → #764ba2` (purple gradient)
   - Accent: `#60a5fa` (blue for citations)
   - Background: `rgba(17, 25, 40, 0.85)`
   - Text: `rgba(255, 255, 255, 0.95)`

---

## Part 4: Testing

### Test Typewriter Effect

1. Open chatbot
2. Ask: "What are humanoid robots?"
3. Observe: AI response types out character by character

### Test Citations

1. Backend must return this format:
```json
{
  "answer": "Humanoid robots are...",
  "citations": [
    {
      "section_title": "Introduction",
      "deep_link_url": "/docs/intro"
    },
    {
      "section_title": "Module 1",
      "deep_link_url": "/docs/module1/overview"
    }
  ]
}
```

2. Ask a question
3. Verify blue badges appear below AI response
4. Click badge to navigate to section

### Test Glassmorphism

1. Open chatbot on a page with varied background
2. Verify semi-transparent, blurred background effect
3. Check smooth slide-up animation

### Test Mobile Responsiveness

1. Resize browser to mobile width (< 768px)
2. Verify:
   - Chat window takes full width
   - FAB button scales appropriately
   - Messages remain readable

---

## Part 5: Troubleshooting

### Issue: Typewriter not working

**Solution**: Check browser console for errors. Ensure `isTyping` flag is set correctly.

```typescript
const aiMessage: Message = {
  role: 'ai',
  text: data.answer,
  sources: data.citations,
  isTyping: true,  // ← Must be true for typewriter
};
```

### Issue: Citations not displaying

**Check backend response**:

```bash
curl -X POST https://YOUR_BACKEND_URL/api/query \
  -H "Content-Type: application/json" \
  -d '{"query_text":"test","selected_text":null}'
```

Verify `citations` array exists in response.

### Issue: Glassmorphism not visible

**Check browser support**:

```css
/* Fallback for unsupported browsers */
@supports not (backdrop-filter: blur(16px)) {
  .chatWindow {
    background: rgba(17, 25, 40, 0.95);
  }
}
```

Most modern browsers support `backdrop-filter`. Safari requires `-webkit-` prefix (already included).

### Issue: Dockerfile build fails

**Common fixes**:

```dockerfile
# If requirements.txt has issues, install individually
RUN pip install fastapi uvicorn[standard] qdrant-client cohere python-dotenv pydantic

# If system dependencies missing
RUN apt-get update && apt-get install -y build-essential
```

### Issue: Port 7860 not accessible

**Verify Uvicorn command**:

```dockerfile
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "7860"]
```

**Check Hugging Face logs**:
- Go to Space → Logs
- Look for "Uvicorn running on..."
- Check for port binding errors

---

## Part 6: Customization

### Change Color Scheme

Edit `styles.module.css`:

```css
/* Primary gradient (FAB button, header) */
.fabButton {
  background: linear-gradient(135deg, #YOUR_COLOR1 0%, #YOUR_COLOR2 100%);
}

/* Citation badge color */
.citationBadge {
  background: rgba(YOUR_R, YOUR_G, YOUR_B, 0.2);
  border: 1px solid rgba(YOUR_R, YOUR_G, YOUR_B, 0.4);
  color: #YOUR_COLOR;
}
```

### Adjust Typewriter Speed

Edit `index.tsx`:

```typescript
// Faster typing (10ms per character)
const { displayedText, isComplete } = useTypewriter(text, 10);

// Slower typing (50ms per character)
const { displayedText, isComplete } = useTypewriter(text, 50);
```

### Change Chat Window Size

Edit `styles.module.css`:

```css
.chatWindow {
  width: 500px;  /* Default: 420px */
  height: 700px; /* Default: 600px */
}
```

### Modify Welcome Message

Edit `index.tsx`:

```tsx
<div className={styles.welcomeMessage}>
  <div className={styles.welcomeIcon}>🤖</div>
  <h3>Your Custom Title</h3>
  <p>Your custom welcome message!</p>
</div>
```

---

## Part 7: Production Checklist

### Backend

- [ ] Environment variables configured in Hugging Face Spaces
- [ ] Health endpoint responding: `/health`
- [ ] API docs accessible: `/docs`
- [ ] CORS enabled for frontend domain
- [ ] Rate limiting configured (if needed)
- [ ] Error logging enabled

### Frontend

- [ ] API_BASE_URL updated to production backend
- [ ] Citations displaying correctly
- [ ] Typewriter effect working
- [ ] Mobile responsive
- [ ] Glassmorphism rendering properly
- [ ] No console errors
- [ ] Accessibility tested (keyboard navigation, ARIA labels)

### Performance

- [ ] Backend response time < 3s
- [ ] Frontend bundle size optimized
- [ ] Images/assets compressed
- [ ] Lazy loading implemented (BrowserOnly)

### Security

- [ ] No API keys in frontend code
- [ ] HTTPS enabled
- [ ] Input sanitization (backend)
- [ ] XSS protection

---

## Part 8: Cost Optimization

### Hugging Face Free Tier

**Limits**:
- CPU Basic (free)
- 16GB RAM
- 50GB storage
- Sleep after 48h inactivity

**Recommendations**:
- Use CPU Basic for development
- Upgrade to persistent space if needed ($0/month with community subscription)

### Alternative Hosting

**Render (Free Tier)**:
- 512MB RAM
- Spins down after 15min inactivity
- Free HTTPS

**Railway (Free Tier)**:
- $5 credit/month
- Pay-as-you-go after
- Fast deployments

**Fly.io (Free Tier)**:
- 3 VMs (256MB each)
- Good for global deployment

---

## Summary

You now have:

1. ✅ **Optimized Dockerfile** - FastAPI backend ready for Hugging Face Spaces (port 7860)
2. ✅ **Upgraded index.tsx** - Professional chatbot with typewriter effect and citations
3. ✅ **Professional CSS** - ChatGPT-style dark mode with glassmorphism

**Next Steps**:

1. Deploy backend to Hugging Face Spaces
2. Update `API_BASE_URL` in frontend
3. Test locally
4. Deploy frontend to Vercel/Netlify
5. Share your beautiful RAG chatbot!

**Support**:
- Hugging Face Docs: https://huggingface.co/docs/hub/spaces-overview
- Docusaurus Docs: https://docusaurus.io/docs
- Qdrant Docs: https://qdrant.tech/documentation/
- Cohere Docs: https://docs.cohere.com/

---

**Created**: 2025-12-30
**Status**: Ready for Production Deployment
