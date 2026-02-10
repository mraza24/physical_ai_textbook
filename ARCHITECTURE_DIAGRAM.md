# 🏗️ Unified Backend Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        DOCUSAURUS FRONTEND (localhost:3000)                 │
│                                                                             │
│  ┌──────────────────────┐  ┌───────────────────────┐  ┌─────────────────┐ │
│  │  RAGChatbot UI       │  │  ChapterActions UI    │  │  Auth Forms     │ │
│  │  (Neon Pulse Button) │  │  (Personalize/Translate│  │  (Signup/Login) │ │
│  └──────────┬───────────┘  └───────────┬───────────┘  └────────┬────────┘ │
│             │                           │                        │          │
│             │  POST /api/chat           │  POST /api/personalize │          │
│             │  (Gemini + RAG)           │  POST /api/translate   │          │
│             │                           │                        │          │
└─────────────┼───────────────────────────┼────────────────────────┼──────────┘
              │                           │                        │
              │                           │                        │
              ▼                           ▼                        ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                   UNIFIED BACKEND (localhost:4000)                          │
│                                                                             │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │                         EXPRESS.JS ROUTES                             │ │
│  │                                                                       │ │
│  │  /api/chat              /api/personalize         /api/auth           │ │
│  │  (chat.ts)              (personalize.ts)         (auth routes)       │ │
│  │  ↓                      ↓                        ↓                   │ │
│  │  Gemini API             Claude API               Better-Auth         │ │
│  │  + RAG Search           + User Profile           + JWT              │ │
│  └─────┬───────────────────┬────────────────────────┬───────────────────┘ │
│        │                   │                        │                     │
│        ▼                   ▼                        ▼                     │
│  ┌─────────────────────────────────────────────────────────────────────┐ │
│  │                     INTEGRATIONS & DATA                             │ │
│  │                                                                     │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐             │ │
│  │  │ Gemini API   │  │ Claude API   │  │ Neon DB      │             │ │
│  │  │ (RAG Chat)   │  │ (Transform)  │  │ (Postgres)   │             │ │
│  │  └──────────────┘  └──────────────┘  └──────┬───────┘             │ │
│  │                                              │                      │ │
│  │  ┌──────────────┐                           │                      │ │
│  │  │ Qdrant DB    │←─────────────────────────┘                       │ │
│  │  │ (Vector Search)                                                 │ │
│  │  └──────────────┘                                                  │ │
│  └─────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## API Endpoints Overview

### 🤖 Chat Endpoint (NEW - Unified Chatbot)
```
POST /api/chat
Body: { "query_text": "What is ROS2?", "selected_text": null }

Response: {
  "answer": "ROS2 is the second generation...",
  "citations": [
    { "section_title": "Intro to ROS2", "deep_link_url": "/docs/module1/intro" }
  ],
  "metadata": {
    "skill_level": "Beginner",
    "model": "gemini-1.5-flash"
  }
}
```

### ✨ Personalization Endpoint (Existing - Chapter Actions)
```
POST /api/personalize
Headers: { "Authorization": "Bearer <JWT>" }
Body: { "chapterPath": "/docs/module1/intro", "content": "..." }

Response: {
  "transformed_content": "Simplified for beginners...",
  "metadata": { "complexity_level": "Beginner", "cached": false }
}
```

### 🌍 Translation Endpoint (Existing - Chapter Actions)
```
POST /api/translate/urdu
Headers: { "Authorization": "Bearer <JWT>" }
Body: { "chapterPath": "/docs/module1/intro", "content": "..." }

Response: {
  "translated_content": "اردو ترجمہ...",
  "metadata": { "preserved_terms": ["ROS2", "LIDAR", "PID"] }
}
```

### 🔐 Authentication Endpoints (Existing - Better-Auth)
```
POST /api/auth/signup
POST /api/auth/signin
GET  /api/auth/session
POST /api/auth/signout
```

---

## Data Flow Example: User Asks "What is ROS2?"

1. **User clicks chat button** → Opens RAGChatbot UI
2. **User types query** → "What is ROS2?"
3. **Frontend sends request** → `POST localhost:4000/api/chat`
4. **Backend chat.ts route**:
   - Checks if user authenticated (optional)
   - Queries Neon DB for user profile (skill level)
   - Searches Qdrant for relevant textbook sections
   - Sends context + query to Gemini API
   - Receives personalized response
5. **Backend returns**:
   ```json
   {
     "answer": "ROS2 (Robot Operating System 2) is...",
     "citations": [
       { "section_title": "Introduction to ROS2", "deep_link_url": "/docs/module1/intro" }
     ]
   }
   ```
6. **Frontend displays**:
   - Typewriter effect for answer
   - Clickable citation badges
   - Neon styling maintained

---

## Component Responsibilities

| Component | Responsibility | Tech Stack |
|-----------|---------------|------------|
| **RAGChatbot** | User interface for chat | React + TypeScript |
| **ChapterActions** | Personalize/Translate buttons | React + TypeScript |
| **Auth Forms** | Signup/Login | React + Better-Auth |
| **chat.ts** | Gemini + RAG integration | Express + Gemini API |
| **personalize.ts** | Content adaptation | Express + Claude API |
| **translate.ts** | Urdu translation | Express + Claude API |
| **auth routes** | JWT authentication | Better-Auth + Neon |
| **Neon DB** | User profiles + cache | PostgreSQL |
| **Qdrant** | Vector search | Vector Database |

---

## Environment Variables Required

### Backend (.env)
```env
# Database
DATABASE_URL=postgresql://...@neon.tech/neondb

# Authentication
JWT_SECRET=your_secret_key_here

# AI APIs
GEMINI_API_KEY=your_gemini_key_here
GEMINI_MODEL=gemini-1.5-flash
ANTHROPIC_API_KEY=your_claude_key_here

# Vector Search
QDRANT_URL=https://...gcp.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_key_here
QDRANT_COLLECTION=physical_ai_textbook

# Server Config
PORT=4000
CORS_ORIGINS=http://localhost:3000
NODE_ENV=development
```

### Frontend (No .env needed!)
- ✅ Hardcoded API_BASE_URL = 'http://localhost:4000'
- ✅ No process.env calls
- ✅ Works in browser without webpack config

---

## Security Flow

```
User Request
    ↓
╔═══════════════════════════════════════════╗
║  CORS Middleware                          ║
║  ✓ Check origin (localhost:3000)         ║
╚═══════════════════════════════════════════╝
    ↓
╔═══════════════════════════════════════════╗
║  Optional: JWT Middleware                 ║
║  ✓ Verify token (if authenticated)       ║
║  ✓ Attach user profile                   ║
╚═══════════════════════════════════════════╝
    ↓
╔═══════════════════════════════════════════╗
║  Rate Limiter (for /personalize, /translate)
║  ✓ 5 requests per minute per user        ║
╚═══════════════════════════════════════════╝
    ↓
╔═══════════════════════════════════════════╗
║  Route Handler (chat, personalize, etc)   ║
║  ✓ Process request                        ║
║  ✓ Return response                        ║
╚═══════════════════════════════════════════╝
```

---

## Caching Strategy

### Transformation Cache (5-min TTL)
```
User requests personalization for Chapter 1
    ↓
Backend generates SHA-256 cache key:
    hash(chapterPath + userId + contentHash)
    ↓
Check if cached in Neon DB transformation_cache table
    ↓
    ├─ CACHE HIT: Return cached result (< 100ms)
    └─ CACHE MISS: Call Claude API, store result, return (~ 3-5s)
```

### Frontend Persistence (24-hour expiry)
```
User personalizes Chapter 1
    ↓
Store in localStorage:
    key: "chapter_content_/docs/module1/intro"
    value: { content, type, timestamp }
    ↓
User navigates away and returns
    ↓
Auto-restore from localStorage (instant)
```

---

**This unified architecture provides:**
- ✅ Single backend (localhost:4000)
- ✅ Multiple AI providers (Gemini, Claude)
- ✅ User authentication (Better-Auth + JWT)
- ✅ Vector search (Qdrant RAG)
- ✅ Response caching (Neon DB)
- ✅ Frontend persistence (localStorage)
- ✅ Beautiful UI (Neon aesthetics)
