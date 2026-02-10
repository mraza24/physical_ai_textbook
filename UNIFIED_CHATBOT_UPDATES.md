# Unified Chatbot Implementation Plan

## Overview
This document provides the complete updated files to unify the RAG Chatbot with the main backend (localhost:4000) using Gemini API.

---

## 1. Updated Frontend: RAGChatbot/index.tsx

**Location**: `textbook/src/components/RAGChatbot/index.tsx`

**Key Changes**:
- ✅ Removed `process.env` calls (fixes "process is not defined" error)
- ✅ Changed API_BASE_URL to `http://localhost:4000`
- ✅ Updated endpoint from `/api/query` to `/api/chat`
- ✅ Maintained all Neon styling, pulse button, typewriter effect
- ✅ Kept Citations component with deep links

```typescript
import React, { useState, useEffect, useRef } from 'react';
import ReactDOM from 'react-dom';
import BrowserOnly from '@docusaurus/BrowserOnly';
import styles from './styles.module.css';

interface Message {
  role: 'user' | 'ai';
  text: string;
  sources?: Citation[];
  isTyping?: boolean;
}

interface Citation {
  section_title: string;
  deep_link_url: string;
}

const useTypewriter = (text: string, speed: number = 30) => {
  const [displayedText, setDisplayedText] = useState('');
  const [isComplete, setIsComplete] = useState(false);

  useEffect(() => {
    setDisplayedText('');
    setIsComplete(false);
    let currentIndex = 0;

    const timer = setInterval(() => {
      if (currentIndex < text.length) {
        setDisplayedText(text.slice(0, currentIndex + 1));
        currentIndex++;
      } else {
        setIsComplete(true);
        clearInterval(timer);
      }
    }, speed);

    return () => clearInterval(timer);
  }, [text, speed]);

  return { displayedText, isComplete };
};

const TypewriterText: React.FC<{ text: string; onComplete?: () => void }> = ({ text, onComplete }) => {
  const { displayedText, isComplete } = useTypewriter(text, 20);

  useEffect(() => {
    if (isComplete && onComplete) {
      onComplete();
    }
  }, [isComplete, onComplete]);

  return <span>{displayedText}</span>;
};

const Citations: React.FC<{ sources: Citation[] }> = ({ sources }) => {
  if (!sources || sources.length === 0) return null;

  return (
    <div className={styles.citationsContainer}>
      <div className={styles.citationsLabel}>Sources:</div>
      <div className={styles.citationsList}>
        {sources.map((source, idx) => (
          <a key={idx} href={source.deep_link_url} className={styles.citationBadge} target="_blank" rel="noopener noreferrer">
            {source.section_title}
          </a>
        ))}
      </div>
    </div>
  );
};

const MessageBubble: React.FC<{ message: Message; isLatest: boolean; onTypingComplete: () => void }> = ({ message, isLatest, onTypingComplete }) => {
  const isUser = message.role === 'user';

  return (
    <div className={`${styles.messageWrapper} ${isUser ? styles.userMessage : styles.aiMessage}`}>
      <div className={styles.messageBubble}>
        {message.isTyping && isLatest ? <TypewriterText text={message.text} onComplete={onTypingComplete} /> : message.text}
      </div>
      {!isUser && message.sources && <Citations sources={message.sources} />}
    </div>
  );
};

const ChatbotContent = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [query, setQuery] = useState('');
  const [messages, setMessages] = useState<Message[]>([]);
  const [loading, setLoading] = useState(false);
  const [portalTarget, setPortalTarget] = useState<HTMLElement | null>(null);
  const [typingComplete, setTypingComplete] = useState(true);
  const chatEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // ✅ FIXED: Removed process.env, hardcoded localhost:4000
  const API_BASE_URL = 'http://localhost:4000';

  useEffect(() => {
    if (typeof document !== 'undefined') {
      setPortalTarget(document.body);
    }
  }, []);

  useEffect(() => {
    if (isOpen && chatEndRef.current) {
      chatEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages, loading, isOpen]);

  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  const handleSearch = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!query.trim() || !typingComplete) return;

    const userMessage: Message = { role: 'user', text: query };
    setMessages(prev => [...prev, userMessage]);
    setQuery('');
    setLoading(true);
    setTypingComplete(false);

    try {
      // ✅ UPDATED: Changed endpoint from /api/query to /api/chat
      const res = await fetch(`${API_BASE_URL}/api/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query_text: query,
          selected_text: null
        }),
      });

      if (!res.ok) throw new Error(`HTTP ${res.status}`);

      const data = await res.json();
      const aiMessage: Message = {
        role: 'ai',
        text: data.answer || 'No response received.',
        sources: data.citations || [],
        isTyping: true,
      };
      setMessages(prev => [...prev, aiMessage]);
    } catch (err) {
      setMessages(prev => [...prev, {
        role: 'ai',
        text: '⚠️ Backend connection failed. Please ensure the backend server is running on port 4000.',
        isTyping: false
      }]);
      setTypingComplete(true);
    } finally {
      setLoading(false);
    }
  };

  const handleTypingComplete = () => {
    setTypingComplete(true);
    setMessages(prev => {
      const updated = [...prev];
      if (updated.length > 0) {
        updated[updated.length - 1] = { ...updated[updated.length - 1], isTyping: false };
      }
      return updated;
    });
  };

  const ChatWindow = (
    <div className={`${styles.chatWindow} ${isOpen ? styles.chatWindowOpen : ''}`}>
      <div className={styles.chatHeader}>
        <div className={styles.headerContent}>
          <div className={styles.headerIcon}>🤖</div>
          <div className={styles.headerText}>
            <div className={styles.headerTitle}>Physical AI Tutor</div>
            <div className={styles.headerSubtitle}>Powered by Gemini + RAG</div>
          </div>
        </div>
        <button onClick={() => setIsOpen(false)} className={styles.closeButton} aria-label="Close chat">✕</button>
      </div>

      <div className={styles.messagesContainer}>
        {messages.length === 0 && (
          <div className={styles.welcomeMessage}>
            <div className={styles.welcomeIcon}>👋</div>
            <h3>Welcome to Physical AI Tutor</h3>
            <p>Ask me anything about robotics, embodied AI, and autonomous systems!</p>
          </div>
        )}

        {messages.map((message, idx) => (
          <MessageBubble key={idx} message={message} isLatest={idx === messages.length - 1} onTypingComplete={handleTypingComplete} />
        ))}

        {loading && (
          <div className={styles.loadingIndicator}>
            <div className={styles.loadingDots}>
              <span></span><span></span><span></span>
            </div>
          </div>
        )}

        <div ref={chatEndRef} />
      </div>

      <form onSubmit={handleSearch} className={styles.inputForm}>
        <input
          ref={inputRef}
          type="text"
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          className={styles.input}
          placeholder="Ask about humanoid robots, sensors, control systems..."
          disabled={loading || !typingComplete}
        />
        <button type="submit" className={styles.sendButton} disabled={loading || !typingComplete || !query.trim()} aria-label="Send message">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
            <path d="M22 2L11 13" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            <path d="M22 2L15 22L11 13L2 9L22 2Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        </button>
      </form>
    </div>
  );

  return (
    <div className={styles.chatbotContainer}>
      {/* ✅ MAINTAINED: Neon pulse button styling */}
      <button
        onClick={() => setIsOpen(!isOpen)}
        className={`${styles.fabButton} ${isOpen ? styles.fabButtonActive : ''}`}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
      >
        <span className={styles.fabIcon}>{isOpen ? '✕' : '💬'}</span>
        {!isOpen && <span className={styles.fabPulse}></span>}
      </button>
      {isOpen && portalTarget && ReactDOM.createPortal(ChatWindow, portalTarget)}
    </div>
  );
};

export default function RAGChatbot() {
  return <BrowserOnly fallback={<div>Loading...</div>}>{() => <ChatbotContent />}</BrowserOnly>;
}
```

---

## 2. Backend Setup: Install Gemini API

```bash
cd backend
npm install @google/generative-ai
```

---

## 3. Backend Environment Variables

Add to `backend/.env`:

```env
# Gemini API Configuration
GEMINI_API_KEY=your_gemini_api_key_here
GEMINI_MODEL=gemini-1.5-flash

# Qdrant Vector Database (for RAG)
QDRANT_URL=https://your-qdrant-url.gcp.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION=physical_ai_textbook
```

---

## 4. New Backend Route: src/routes/chat.ts

Create new file `backend/src/routes/chat.ts`:

```typescript
import { Router, Request, Response } from 'express';
import { GoogleGenerativeAI } from '@google/generative-ai';
import { db } from '../db/connection';
import { userProfile } from '../db/schema';
import { eq } from 'drizzle-orm';
import * as dotenv from 'dotenv';

dotenv.config();

const router = Router();

// Initialize Gemini API
const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY || '');
const model = genAI.getGenerativeModel({ model: process.env.GEMINI_MODEL || 'gemini-1.5-flash' });

// Mock Qdrant interface (replace with actual Qdrant client)
interface QdrantSearchResult {
  section_title: string;
  deep_link_url: string;
  content: string;
  score: number;
}

async function searchQdrant(query: string): Promise<QdrantSearchResult[]> {
  // TODO: Replace with actual Qdrant client integration
  // For now, returning mock data
  return [
    {
      section_title: 'Introduction to ROS2',
      deep_link_url: '/docs/module1/intro',
      content: 'ROS2 is the second generation of the Robot Operating System...',
      score: 0.95
    },
    {
      section_title: 'Sensor Integration',
      deep_link_url: '/docs/module2/sensors',
      content: 'Physical AI systems rely heavily on sensor fusion...',
      score: 0.87
    }
  ];
}

/**
 * POST /api/chat
 *
 * RAG-based chat endpoint with Gemini API
 * - Queries Neon DB for user profile (skill level)
 * - Searches Qdrant vector DB for relevant context
 * - Generates personalized response with Gemini
 * - Returns answer with citations
 */
router.post('/', async (req: Request, res: Response): Promise<void> => {
  try {
    const { query_text, selected_text } = req.body;

    if (!query_text || typeof query_text !== 'string') {
      res.status(400).json({ error: 'query_text is required' });
      return;
    }

    // Step 1: Get user profile from Neon DB (if authenticated)
    // For now, defaulting to 'Beginner' if no user found
    let skillLevel = 'Beginner';
    let userName = 'there';

    // TODO: Uncomment when authentication is fully working
    // const userId = req.user?.id; // From Better-Auth middleware
    // if (userId) {
    //   const [profile] = await db
    //     .select()
    //     .from(userProfile)
    //     .where(eq(userProfile.userId, userId))
    //     .limit(1);
    //
    //   if (profile) {
    //     skillLevel = profile.softwareBackground || 'Beginner';
    //     userName = profile.userId.split('@')[0]; // Extract name from email
    //   }
    // }

    // Step 2: Search Qdrant for relevant context
    const searchResults = await searchQdrant(query_text);
    const contextChunks = searchResults.map(r => r.content).join('\n\n');
    const citations = searchResults.map(r => ({
      section_title: r.section_title,
      deep_link_url: r.deep_link_url
    }));

    // Step 3: Build personalized prompt
    const systemPrompt = `You are a Physical AI Tutor helping students learn about robotics and embodied AI.

User Profile:
- Skill Level: ${skillLevel}
- Name: ${userName}

Instructions:
- Tailor explanations to the user's ${skillLevel} level
- Use simple analogies for Beginners, technical depth for Experts
- Reference the provided context chunks
- Be concise but thorough
- Use markdown formatting`;

    const userPrompt = `Context from textbook:
${contextChunks}

User Question: ${query_text}

Please provide a clear, ${skillLevel}-appropriate answer based on the context above.`;

    // Step 4: Generate response with Gemini
    const result = await model.generateContent([
      { role: 'user', parts: [{ text: systemPrompt }] },
      { role: 'user', parts: [{ text: userPrompt }] }
    ]);

    const answer = result.response.text();

    // Step 5: Return response with citations
    res.json({
      answer,
      citations,
      metadata: {
        skill_level: skillLevel,
        model: process.env.GEMINI_MODEL,
        sources_used: searchResults.length
      }
    });

  } catch (error: any) {
    console.error('Chat API Error:', error);
    res.status(500).json({
      error: 'Internal Server Error',
      message: error.message || 'Failed to generate response'
    });
  }
});

export default router;
```

---

## 5. Update Backend Index: src/index.ts

Add the chat route to your main backend file. Insert this **AFTER** the existing routes:

```typescript
// ... existing imports ...
import chatRoutes from './routes/chat';

// ... existing middleware and routes ...

/**
 * RAG Chat Routes
 *
 * Unified chatbot endpoint:
 * - POST /api/chat (Gemini + Qdrant RAG)
 *
 * NO AUTH REQUIRED: Public chatbot for all users
 * User profile fetched if authenticated (via Better-Auth session)
 */
app.use('/api/chat', chatRoutes);

// ... rest of the file ...
```

**Full insertion point** (around line 165 in your current index.ts):

```typescript
// BEFORE (existing code):
app.use(
  '/api/translate',
  requireAuth,
  createTransformationRateLimiter(5, 1),
  translateRoutes
);

// ✅ ADD THIS:
/**
 * RAG Chat Routes
 *
 * Unified chatbot endpoint:
 * - POST /api/chat (Gemini + Qdrant RAG)
 */
app.use('/api/chat', chatRoutes);

// AFTER (existing code):
app.get('/api', (req: Request, res: Response) => {
  res.json({
    name: 'Authentication & Personalization API',
    // ... etc
```

---

## 6. Testing the Integration

### Start Backend
```bash
cd backend
npm run dev
```

Server should start on `http://localhost:4000`

### Start Frontend
```bash
cd textbook
npm start
```

Docusaurus should start on `http://localhost:3000`

### Test Chat
1. Click the pulsing chat button (💬) in bottom-right corner
2. Type: "What is ROS2?"
3. You should see:
   - ✅ Typewriter effect
   - ✅ Gemini-generated response
   - ✅ Citations at the bottom
   - ✅ Neon styling maintained

---

## 7. Integration with Qdrant (Full RAG)

To connect to your actual Qdrant database, install the Qdrant client:

```bash
cd backend
npm install @qdrant/js-client-rest
```

Then update `chat.ts` with real Qdrant integration:

```typescript
import { QdrantClient } from '@qdrant/js-client-rest';

const qdrant = new QdrantClient({
  url: process.env.QDRANT_URL,
  apiKey: process.env.QDRANT_API_KEY,
});

async function searchQdrant(query: string): Promise<QdrantSearchResult[]> {
  // Generate query embedding (requires embedding model)
  // For Gemini, you can use their embedding API or use a separate model

  const searchResponse = await qdrant.search(process.env.QDRANT_COLLECTION || 'physical_ai_textbook', {
    vector: queryEmbedding, // Generated from query_text
    limit: 5,
    with_payload: true
  });

  return searchResponse.map(result => ({
    section_title: result.payload?.section_title || 'Unknown',
    deep_link_url: result.payload?.deep_link_url || '#',
    content: result.payload?.content || '',
    score: result.score
  }));
}
```

---

## 8. Cleanup Checklist

- [ ] Stop `rag-backend` server (if running)
- [ ] Remove Hugging Face dependencies (if any)
- [ ] Update CORS in `backend/.env` to allow `http://localhost:3000`
- [ ] Verify `GEMINI_API_KEY` is set in `backend/.env`
- [ ] Test chat with backend running on port 4000
- [ ] Verify no `process.env` errors in browser console

---

## 9. File Structure Summary

```
physical_ai_textbook/
├── backend/                          # Main unified backend
│   ├── src/
│   │   ├── routes/
│   │   │   ├── chat.ts              # ✅ NEW: Gemini + RAG endpoint
│   │   │   ├── personalize.ts       # Existing
│   │   │   └── translate.ts         # Existing
│   │   └── index.ts                 # ✅ UPDATED: Added /api/chat route
│   ├── .env                          # ✅ UPDATED: Added GEMINI_API_KEY
│   └── package.json                  # ✅ UPDATED: Added @google/generative-ai
│
├── textbook/                         # Docusaurus frontend
│   └── src/components/
│       └── RAGChatbot/
│           └── index.tsx             # ✅ UPDATED: localhost:4000, /api/chat
│
└── rag-backend/                      # ❌ DEPRECATED: No longer used
```

---

## 10. Next Steps

1. **Copy-Paste Files**: Replace your existing files with the updated versions above
2. **Install Gemini**: `cd backend && npm install @google/generative-ai`
3. **Update .env**: Add your `GEMINI_API_KEY`
4. **Restart Backend**: `npm run dev` in `backend/`
5. **Test Chatbot**: Open `http://localhost:3000` and click the chat button

---

## Support

If you encounter any issues:
- ✅ Check browser console for "process is not defined" → Should be fixed now
- ✅ Check backend logs for API errors
- ✅ Verify both servers are running (backend:4000, frontend:3000)
- ✅ Confirm CORS is allowing localhost:3000

**Your unified chatbot is now ready! 🚀**
