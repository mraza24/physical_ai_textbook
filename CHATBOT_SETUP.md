# 🤖 RAG Chatbot Integration Guide

## ✅ What Was Created

I've added a professional chatbot UI to your Docusaurus site that connects to your RAG backend. Here's what was created:

### Files Created:
1. **`src/components/Chatbot/index.tsx`** - Main chatbot component
2. **`src/components/Chatbot/styles.module.css`** - Chatbot styling
3. **`src/theme/Root.tsx`** - Global wrapper to show chatbot on every page
4. **`.env.local`** - Environment configuration for API URL

---

## 🚀 Quick Start

### Step 1: Install Dependencies (Already Done!)

Your existing `package.json` has everything needed. No new dependencies required!

### Step 2: Configure Backend URL

#### For Local Development:

`.env.local` is already configured:
```bash
REACT_APP_API_URL=http://localhost:8000
```

#### For Production (Vercel):

1. Go to **Vercel Dashboard** → Your Project → **Settings** → **Environment Variables**

2. Add this environment variable:
   ```
   Name:  REACT_APP_API_URL
   Value: https://your-backend.onrender.com
   ```

   Replace `your-backend.onrender.com` with your actual backend URL.

3. Select environments: **Production**, **Preview**, **Development**

4. Click **Save**

### Step 3: Update Backend URL

Find your deployed backend URL (from Render or Railway) and update:

**Option A: Update `.env.local` for local testing:**
```bash
REACT_APP_API_URL=https://your-actual-backend.onrender.com
```

**Option B: Set in Vercel (recommended for production):**
- See Step 2 above

---

## 🧪 Testing

### Test Locally:

1. **Start your backend** (if not running):
   ```bash
   cd rag-backend
   source venv/bin/activate  # or venv\Scripts\activate on Windows
   uvicorn app.main:app --reload
   ```

2. **Start Docusaurus**:
   ```bash
   cd /path/to/physical_ai_textbook
   npm start
   ```

3. **Open browser**: `http://localhost:3000`

4. **Look for the chatbot button** (purple circle in bottom-right corner)

5. **Click it and ask a question!**

### Test in Production:

1. **Deploy backend** to Render/Railway (if not done):
   ```bash
   cd rag-backend
   ./deploy-render.sh  # or ./deploy-railway.sh
   ```

2. **Get backend URL** (e.g., `https://rag-backend.onrender.com`)

3. **Set Vercel environment variable** (see Step 2 above)

4. **Redeploy frontend** to Vercel:
   ```bash
   git add .
   git commit -m "Add RAG chatbot UI"
   git push origin main
   ```

5. **Wait for deployment** (~2-3 minutes)

6. **Visit your site**: `https://your-site.vercel.app`

7. **Test the chatbot!**

---

## 🎨 Chatbot Features

### ✨ What It Does:

- **Floating button** in bottom-right corner
- **Smooth animations** when opening/closing
- **Real-time Q&A** with your textbook content
- **Citations** with clickable links to textbook sections
- **Confidence scores** for each answer
- **Loading indicators** while processing
- **Error handling** with user-friendly messages
- **Dark mode support** (automatic with Docusaurus theme)
- **Mobile responsive** design
- **Character counter** (max 500 chars)
- **Example questions** to get started
- **Clear chat** button to start fresh

### 📱 User Experience:

1. User clicks purple chatbot button
2. Chat window slides up
3. User sees welcome message and example questions
4. User types or clicks example question
5. Message appears on right (user side)
6. Loading animation shows
7. AI response appears on left with citations
8. User clicks citation links to read more
9. User continues conversation or closes chat

---

## 🔧 Customization

### Change Chatbot Colors:

Edit `src/components/Chatbot/styles.module.css`:

```css
/* Line 9: Change gradient colors */
.floatingButton {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  /* Change to your brand colors, e.g.: */
  /* background: linear-gradient(135deg, #FF6B6B 0%, #4ECDC4 100%); */
}
```

### Change Button Position:

Edit `src/components/Chatbot/styles.module.css`:

```css
/* Line 6-7: Change position */
.floatingButton {
  bottom: 24px;  /* Distance from bottom */
  right: 24px;   /* Distance from right */
  /* For left side: */
  /* left: 24px; */
  /* right: auto; */
}
```

### Modify Example Questions:

Edit `src/components/Chatbot/index.tsx` around line 150:

```tsx
<button onClick={() => setInputValue('Your custom question?')}>
  Your custom question?
</button>
```

### Change Chatbot Title:

Edit `src/components/Chatbot/index.tsx` line 106:

```tsx
<h3>🤖 Ask the Textbook</h3>
<!-- Change to: -->
<h3>💡 Your Custom Title</h3>
```

---

## 🐛 Troubleshooting

### Issue: "Failed to get response" error

**Cause:** Backend not accessible

**Solutions:**
1. Check backend is running: `curl http://localhost:8000/api/health`
2. Verify `REACT_APP_API_URL` is correct in `.env.local`
3. Check CORS is enabled in backend (should be by default)
4. Open browser DevTools → Network tab to see actual error

### Issue: Chatbot button doesn't appear

**Cause:** Root.tsx not loaded

**Solutions:**
1. Restart Docusaurus dev server: `npm start`
2. Clear cache: `npm run clear && npm start`
3. Check browser console for errors

### Issue: "CORS error" in browser console

**Cause:** Backend CORS not configured for your frontend URL

**Solutions:**
1. Update backend `.env` file:
   ```bash
   ALLOWED_ORIGINS=https://your-site.vercel.app,http://localhost:3000
   ```
2. Restart backend
3. Redeploy backend if in production

### Issue: Responses are slow or timeout

**Cause:** Backend processing or cold start (Render free tier)

**Solutions:**
1. First request after 15min inactivity takes ~30s (Render cold start)
2. Use paid Render tier for always-on backend
3. Or use health check pinger to keep backend warm:
   ```bash
   # Run this every 10 minutes (cron job or UptimeRobot)
   curl https://your-backend.onrender.com/api/health
   ```

### Issue: Citations links don't work

**Cause:** Deep link URLs don't match Docusaurus structure

**Solutions:**
1. Check your textbook content ingestion
2. Ensure `deep_link_url` in Qdrant matches Docusaurus routes
3. Update ingestion script to generate correct URLs

---

## 📊 API Integration Details

### Backend Endpoint:
```
POST https://your-backend.com/api/query
```

### Request Format:
```json
{
  "query_text": "What is forward kinematics?",
  "selected_text": null
}
```

### Response Format:
```json
{
  "answer": "Forward kinematics is the process...",
  "citations": [
    {
      "section_title": "Module 1: Robot Kinematics",
      "deep_link_url": "/docs/module1/kinematics",
      "chunk_count": 3
    }
  ],
  "confidence": 0.87,
  "retrieved_chunk_count": 5,
  "processing_time_ms": 342
}
```

### Error Handling:
- **400**: Invalid query (empty, too long)
- **500**: Backend error (Qdrant/Cohere failure)
- **Network error**: Backend unreachable

All errors display user-friendly messages in the chat.

---

## 🚀 Deployment Checklist

### Backend:
- [ ] Backend deployed to Render/Railway
- [ ] Environment variables configured
- [ ] `ALLOWED_ORIGINS` includes Vercel URL
- [ ] Health endpoint working: `/api/health`
- [ ] Query endpoint working: `/api/query`
- [ ] Book content ingested into Qdrant

### Frontend:
- [ ] Chatbot files created (✅ Done!)
- [ ] `.env.local` configured for local dev
- [ ] `REACT_APP_API_URL` set in Vercel dashboard
- [ ] Code committed and pushed to GitHub
- [ ] Vercel deployment successful
- [ ] Chatbot button visible on site
- [ ] Test query returns valid response
- [ ] Citations link to correct pages
- [ ] Dark mode works correctly
- [ ] Mobile view works correctly

---

## 🎯 Next Steps

1. **Test locally** with backend running
2. **Set Vercel environment variable** with your backend URL
3. **Commit and push** your changes
4. **Wait for Vercel deployment**
5. **Test on production** site
6. **Share with users!**

---

## 💡 Advanced Features (Future Enhancements)

Consider adding these later:

- **Conversation history** (save to localStorage)
- **User authentication** (track user queries)
- **Feedback buttons** (thumbs up/down)
- **Copy answer** button
- **Voice input** (Web Speech API)
- **Keyboard shortcuts** (Ctrl+K to open)
- **Typing indicators** (more realistic)
- **Multi-language support**
- **Analytics** (track popular questions)

---

## 📞 Need Help?

If you encounter issues:

1. Check browser console (F12) for errors
2. Check Network tab for failed API calls
3. Verify backend logs (Render/Railway dashboard)
4. Test backend directly: `curl -X POST ... /api/query`
5. Review this guide's Troubleshooting section

---

**Your chatbot is ready to go! 🎉**

Just set your backend URL and deploy to Vercel!
