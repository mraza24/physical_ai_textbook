# Cohere API Migration Guide - January 2026

## Summary of Changes

**Issue**: Backend was returning 404 errors because the `/v1/generate` endpoint was deprecated in September 2025, and models `command-r-plus` and `command-r` were also removed.

**Resolution**: Migrated from deprecated `cohere.generate()` to current `cohere.chat()` API with the `command-r-08-2024` model.

---

## What Changed

### 1. API Endpoint Migration

**Before (DEPRECATED - Sept 2025)**:
```typescript
const response = await cohere.generate({
  model: 'command-r-plus',
  prompt: `${systemPrompt}\n\nUser Question: ${query_text}\n\nAnswer:`,
  maxTokens: 500,
  temperature: 0.7,
});

const answer = response.generations[0].text;
```

**After (CURRENT - Jan 2026)**:
```typescript
const response = await cohere.chat({
  model: 'command-r-08-2024',
  message: query_text,
  preamble: systemPrompt,
  maxTokens: 500,
  temperature: 0.7,
});

const answer = response.text;
```

### 2. Key Differences

| Aspect | Old API (generate) | New API (chat) |
|--------|-------------------|----------------|
| **Method** | `cohere.generate()` | `cohere.chat()` |
| **Model** | `command-r-plus` | `command-r-08-2024` |
| **User Input** | `prompt` (string with system + user) | `message` (user message only) |
| **System Context** | Concatenated into `prompt` | Separate `preamble` parameter |
| **Response Path** | `response.generations[0].text` | `response.text` |
| **Status** | ❌ Deprecated Sept 2025 | ✅ Current (Jan 2026) |

---

## Files Updated

### Production Code

**`backend/src/routes/chat.ts`** (lines 104-114):
- Changed `cohere.generate()` → `cohere.chat()`
- Changed model: `command-r-plus` → `command-r-08-2024`
- Split prompt into `message` and `preamble` parameters
- Changed response extraction: `response.generations[0].text` → `response.text`
- Updated metadata model name

### Test Scripts

**`backend/test-cohere.ts`**:
- Updated both test cases to use `cohere.chat()`
- Changed model to `command-r-08-2024`
- Updated response extraction logic
- Added note about deprecated `/v1/generate` endpoint

---

## Current Cohere Models (Jan 2026)

**Recommended Models**:
- `command-r-08-2024` - General-purpose chat (USED IN THIS PROJECT)
- `command-r-plus-08-2024` - Enhanced version with better performance
- `command-a-03-2025` - Most performant (256K context, improved efficiency)

**Deprecated Models** (removed Sept 15, 2025):
- ❌ `command-r-plus` - Use `command-r-plus-08-2024` instead
- ❌ `command-r` - Use `command-r-08-2024` instead
- ❌ All models accessed via `/v1/generate` endpoint

---

## Testing Results

### Before Migration
```
❌ Error: NotFoundError
Status code: 404
Body: {
  "message": "model 'command-r-plus' was removed on September 15, 2025.
              See https://docs.cohere.com/docs/models#command for a list
              of models you can use instead."
}
```

### After Migration
```
✅ Chat endpoint (primary): WORKING
✅ Chat endpoint (production): WORKING
✅ Total response time: 20700ms
✅ API Key Status: ACTIVE (2026)
ℹ️  Note: /v1/generate deprecated Sept 2025, using /v1/chat
```

---

## How to Verify the Fix

### Step 1: Test Cohere API Connection
```bash
cd backend
npx tsx test-cohere.ts
```

**Expected Output**:
```
✅ API Key: VALID
✅ Chat endpoint (primary): WORKING
✅ Chat endpoint (production): WORKING
✅ API Key Status: ACTIVE (2026)
```

### Step 2: Test Backend Chat Endpoint
Start the backend:
```bash
npm run dev
```

Test the chat endpoint (in another terminal):
```bash
curl -X POST http://localhost:4000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "What is ROS2?"
  }'
```

**Expected Response**:
```json
{
  "answer": "ROS2, or Robot Operating System 2, is...",
  "citations": [...],
  "metadata": {
    "model": "cohere-command-r-08-2024",
    "timestamp": "2026-01-03T..."
  }
}
```

---

## Troubleshooting

### Issue: 404 "model not found"
**Cause**: Using deprecated model name

**Fix**: Ensure you're using `command-r-08-2024`, not `command-r` or `command-r-plus`

### Issue: 401 Unauthorized
**Cause**: Invalid or expired COHERE_API_KEY

**Fix**:
1. Go to https://dashboard.cohere.com/api-keys
2. Generate new API key
3. Update `COHERE_API_KEY` in `.env`

### Issue: TypeError accessing response.generations
**Cause**: Using old response extraction logic

**Fix**: Change `response.generations[0].text` to `response.text`

---

## Migration Checklist

- [x] Update `cohere.generate()` → `cohere.chat()`
- [x] Change model to `command-r-08-2024`
- [x] Split `prompt` into `message` and `preamble`
- [x] Update response extraction: `response.text` (not `response.generations[0].text`)
- [x] Update test scripts
- [x] Verify Cohere API test passes
- [x] Test backend chat endpoint works

---

## References

- [Cohere Models Overview](https://docs.cohere.com/docs/models)
- [Cohere Chat API Reference](https://docs.cohere.com/reference/chat)
- [Cohere Release Notes](https://docs.cohere.com/changelog)

**Date**: January 3, 2026
**Migration Status**: ✅ Complete
