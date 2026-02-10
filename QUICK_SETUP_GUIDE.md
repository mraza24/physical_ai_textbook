# 🚀 Quick Setup Guide: Unified Chatbot

## Step 1: Install Gemini API

```bash
cd backend
npm install @google/generative-ai
```

## Step 2: Update Environment Variables

Add to `backend/.env`:

```env
# Gemini API Configuration
GEMINI_API_KEY=your_gemini_api_key_here
GEMINI_MODEL=gemini-1.5-flash

# CORS Configuration
CORS_ORIGINS=http://localhost:3000,http://127.0.0.1:3000
```

## Step 3: Copy Updated Files

### Frontend (Chatbot):
```bash
# Backup original
cp textbook/src/components/RAGChatbot/index.tsx textbook/src/components/RAGChatbot/index.tsx.backup

# Copy new version
cp UPDATED_RAGCHATBOT_INDEX.tsx textbook/src/components/RAGChatbot/index.tsx
```

### Backend (Chat Route):
```bash
# Create new route
cp UPDATED_BACKEND_CHAT_ROUTE.ts backend/src/routes/chat.ts
```

### Backend (Index.ts):

Add this import at the top of `backend/src/index.ts` (around line 10):

```typescript
import chatRoutes from './routes/chat';
```

Add this route AFTER the translate routes (around line 165):

```typescript
/**
 * RAG Chat Routes
 *
 * Unified chatbot endpoint:
 * - POST /api/chat (Gemini + Qdrant RAG)
 *
 * NO AUTH REQUIRED: Public chatbot for all users
 */
app.use('/api/chat', chatRoutes);
```

## Step 4: Start Both Servers

### Terminal 1 - Backend:
```bash
cd backend
npm run dev
```

Expected output:
```
🚀 Authentication & Personalization Backend
📍 Server running on: http://localhost:4000
```

### Terminal 2 - Frontend:
```bash
cd textbook
npm start
```

Expected output:
```
Docusaurus website is running at: http://localhost:3000/
```

## Step 5: Test the Chatbot

1. Open browser: http://localhost:3000
2. Click the pulsing chat button (💬) in bottom-right
3. Type: "What is ROS2?"
4. Verify:
   - ✅ Typewriter effect works
   - ✅ Gemini response appears
   - ✅ Citations show at bottom
   - ✅ No "process is not defined" errors

## Step 6: Test Backend Health

```bash
curl http://localhost:4000/api/chat/health
```

Expected response:
```json
{
  "status": "OK",
  "gemini_connected": true,
  "model": "gemini-1.5-flash",
  "qdrant_mock": true
}
```

## Troubleshooting

### Error: "process is not defined"
- ✅ FIXED in new version (no process.env calls)

### Error: "Backend connection failed"
- Check if backend is running on port 4000
- Verify CORS allows localhost:3000

### Error: "Gemini API key not found"
- Add GEMINI_API_KEY to backend/.env
- Restart backend server

### Error: "Module not found: chat.ts"
- Ensure you copied UPDATED_BACKEND_CHAT_ROUTE.ts to backend/src/routes/chat.ts
- Check import in index.ts

## File Locations Summary

```
Updated Files:
✅ textbook/src/components/RAGChatbot/index.tsx  (from UPDATED_RAGCHATBOT_INDEX.tsx)
✅ backend/src/routes/chat.ts                    (from UPDATED_BACKEND_CHAT_ROUTE.ts)
✅ backend/src/index.ts                          (add import + route)
✅ backend/.env                                   (add GEMINI_API_KEY)
```

## Next: Add Real Qdrant Integration

Once the basic setup works, you can replace the mock Qdrant function in `chat.ts`:

```bash
cd backend
npm install @qdrant/js-client-rest
```

Update `chat.ts` with real vector search (instructions in UNIFIED_CHATBOT_UPDATES.md).

---

**That's it! Your unified chatbot should now be working! 🎉**
