# API Documentation: Physical AI Textbook Bonus Features

**Base URL**: `http://localhost:4000` (development)

**Version**: 1.0.0

**Authentication**: JWT Bearer token in `Authorization` header

**Rate Limiting**: Redis-backed rate limiting per user/IP

---

## Table of Contents

1. [Authentication API](#authentication-api)
2. [Personalization API](#personalization-api)
3. [Translation API](#translation-api)
4. [Error Handling](#error-handling)
5. [Rate Limiting](#rate-limiting)

---

## Authentication API

### POST /api/auth/signup

Create a new user account with background profile.

**Request**:
```json
{
  "email": "student@example.com",
  "password": "SecurePass123!",
  "profile": {
    "python_level": "Intermediate",
    "ros2_level": "Beginner",
    "gpu_available": "Yes",
    "hardware_tier": "Tier2",
    "primary_goal": "Learning"
  }
}
```

**Response** (201 Created):
```json
{
  "user": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "student@example.com",
    "created_at": "2025-12-12T10:00:00.000Z",
    "last_login": "2025-12-12T10:00:00.000Z"
  },
  "profile": {
    "python_level": "Intermediate",
    "ros2_level": "Beginner",
    "gpu_available": "Yes",
    "hardware_tier": "Tier2",
    "primary_goal": "Learning",
    "language_preference": "English"
  },
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
}
```

**Validation Rules**:
- Email: Valid RFC 5322 format, max 255 chars
- Password: Min 8 chars, at least 1 number, 1 special char
- Profile fields: Must match enum values

**Enums**:
```typescript
PythonLevel: "Beginner" | "Intermediate" | "Expert"
ROS2Level: "None" | "Beginner" | "Intermediate" | "Expert"
GPUAvailability: "Yes" | "No" | "Cloud"
HardwareTier: "Tier1" | "Tier2" | "Tier3"
PrimaryGoal: "Learning" | "Research" | "Teaching"
LanguagePreference: "English" | "Urdu"
```

**Errors**:
- `400`: Invalid email, weak password, missing required fields
- `409`: Email already registered

---

### POST /api/auth/signin

Sign in with email and password.

**Request**:
```json
{
  "email": "student@example.com",
  "password": "SecurePass123!",
  "rememberMe": true
}
```

**Response** (200 OK):
```json
{
  "user": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "student@example.com",
    "created_at": "2025-12-12T10:00:00.000Z",
    "last_login": "2025-12-12T10:30:00.000Z"
  },
  "profile": {
    "python_level": "Intermediate",
    "ros2_level": "Beginner",
    "gpu_available": "Yes",
    "hardware_tier": "Tier2",
    "primary_goal": "Learning",
    "language_preference": "English"
  },
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
}
```

**JWT Token Expiration**:
- Without `rememberMe`: 1 day
- With `rememberMe`: 7 days

**Account Lockout**:
- After 3 failed attempts: Account locked for 15 minutes
- Failed attempts tracked in Redis cache (15-minute TTL)

**Errors**:
- `401`: Invalid credentials
- `423`: Account locked (too many failed attempts)

---

### POST /api/auth/logout

Logout current user (invalidate session).

**Headers**:
```
Authorization: Bearer <jwt-token>
```

**Response** (200 OK):
```json
{
  "message": "Logout successful"
}
```

**Errors**:
- `401`: Not authenticated

**Note**: JWT-based auth primarily client-side. Server logs logout action but does not blacklist tokens. Client must delete token from localStorage.

---

### GET /api/auth/profile

Get current user's profile.

**Headers**:
```
Authorization: Bearer <jwt-token>
```

**Response** (200 OK):
```json
{
  "profile": {
    "python_level": "Intermediate",
    "ros2_level": "Beginner",
    "gpu_available": "Yes",
    "hardware_tier": "Tier2",
    "primary_goal": "Learning",
    "language_preference": "English"
  }
}
```

**Errors**:
- `401`: Not authenticated
- `404`: Profile not found

---

### PUT /api/auth/profile

Update current user's profile.

**Headers**:
```
Authorization: Bearer <jwt-token>
```

**Request** (partial update allowed):
```json
{
  "python_level": "Expert",
  "gpu_available": "Cloud",
  "language_preference": "Urdu"
}
```

**Response** (200 OK):
```json
{
  "profile": {
    "python_level": "Expert",
    "ros2_level": "Beginner",
    "gpu_available": "Cloud",
    "hardware_tier": "Tier2",
    "primary_goal": "Learning",
    "language_preference": "Urdu"
  }
}
```

**Side Effects**:
- Invalidates all personalization caches (pattern: `personalize:*`)
- Updates `updated_at` timestamp

**Errors**:
- `400`: Invalid enum value
- `401`: Not authenticated

---

## Personalization API

### POST /api/personalize

Personalize chapter content based on user profile.

**Headers**:
```
Authorization: Bearer <jwt-token>
Content-Type: application/json
```

**Request**:
```json
{
  "chapterId": "ch1.1",
  "originalContent": "# ROS 2 Fundamentals\n\nROS 2 is a middleware..."
}
```

**Response** (200 OK):
```json
{
  "personalizedContent": "# ROS 2 Fundamentals\n\n[Adapted for Expert Python, Beginner ROS 2]...",
  "cacheHit": false,
  "profile": {
    "python_level": "Expert",
    "ros2_level": "Beginner",
    "gpu_available": "Cloud",
    "hardware_tier": "Tier2",
    "primary_goal": "Learning"
  }
}
```

**Personalization Strategy**:

Claude API receives prompt with:
- User profile (5 background fields)
- Original chapter content
- Adaptive instructions based on profile

**Adaptations by Profile**:

| Field | Value | Adaptation |
|-------|-------|------------|
| `python_level` | Beginner | Add Python syntax explanations, inline comments |
| | Expert | Minimize Python basics, focus on ROS 2 patterns |
| `ros2_level` | None | Expand ROS 2 concepts, add motivation/context |
| | Expert | Reduce basics, focus on advanced patterns/edge cases |
| `gpu_available` | No | Emphasize CPU alternatives, mention cloud options |
| | Yes | Include GPU workflows, performance comparisons |
| | Cloud | Mention AWS/GCP setup, cost estimates |
| `hardware_tier` | Tier1 | Edge device workflows, optimization techniques |
| | Tier3 | Advanced features, multi-GPU setups |
| `primary_goal` | Learning | Learning tips, common mistakes, exercises |
| | Research | Recent papers, open problems, research opportunities |
| | Teaching | Pedagogical notes, assessment questions, rubrics |

**Caching**:
- Key: `personalize:{chapterId}:{profileHash}`
- TTL: 24 hours
- Profile hash: MD5 of `python_level|ros2_level|gpu_available|hardware_tier|primary_goal`
- Cache hit rate typically 60-80% for common profiles

**Rate Limiting**:
- 10 requests per minute per user
- Redis-backed distributed rate limiting

**Validation**:
- `chapterId`: Format `ch{module}.{chapter}` (e.g., `ch1.1`, `ch3.4`)
- `originalContent`: Max 100,000 characters

**Errors**:
- `400`: Invalid chapterId format, content too large
- `401`: Not authenticated
- `404`: Profile not found
- `429`: Rate limit exceeded (retry after 60 seconds)
- `500`: Claude API error

**Performance**:
- Cache hit: ~50ms (Redis lookup)
- Cache miss: ~5-10 seconds (Claude API call)

---

## Translation API

### POST /api/translate

Translate chapter content to target language (currently Urdu only).

**Headers**:
```
Content-Type: application/json
```

**Request**:
```json
{
  "chapterId": "ch1.1",
  "originalContent": "# ROS 2 Fundamentals\n\nROS 2 is a middleware...",
  "targetLanguage": "urdu"
}
```

**Response** (200 OK):
```json
{
  "translatedContent": "# آر او ایس 2 (ROS 2) بنیادی باتیں\n\nآر او ایس 2 ایک مڈل ویئر ہے...",
  "cacheHit": false,
  "targetLanguage": "urdu",
  "technicalTerms": {
    "ROS 2": "آر او ایس 2",
    "Node": "نوڈ",
    "Publisher": "پبلشر",
    "Subscriber": "سبسکرائبر"
  }
}
```

**Translation Rules**:

1. **Preserve Code**: All code blocks (` ``` `) and inline code (` ` `) remain in English
2. **Transliterate Terms**: Technical terms transliterated to Urdu script + original in parentheses first time
3. **Maintain Structure**: All Markdown syntax (#, -, *, >) preserved
4. **Mermaid Diagrams**: Labels translated, syntax preserved
5. **RTL Text**: Urdu text flows right-to-left, code blocks left-to-right

**Supported Languages**:
- `urdu` (اردو) - Right-to-left, Nastaliq script

**Caching**:
- Key: `translate:{chapterId}:{targetLanguage}`
- TTL: 7 days (translations stable)
- No automatic invalidation

**Rate Limiting**:
- 5 requests per minute per IP
- No authentication required

**Validation**:
- `chapterId`: Format `ch{module}.{chapter}`
- `originalContent`: Max 100,000 characters
- `targetLanguage`: Must be in supported list

**Errors**:
- `400`: Invalid chapterId, unsupported language, content too large
- `429`: Rate limit exceeded (retry after 60 seconds)
- `500`: Claude API error

**Performance**:
- Cache hit: ~50ms
- Cache miss: ~10-15 seconds (translation slower than personalization)

---

## Error Handling

All errors follow consistent JSON format:

```json
{
  "error": {
    "message": "Human-readable error message",
    "code": 400,
    "details": {} // Optional, context-specific details
  },
  "timestamp": "2025-12-12T10:00:00.000Z",
  "path": "/api/personalize"
}
```

**HTTP Status Codes**:

| Code | Meaning | Common Causes |
|------|---------|---------------|
| 400 | Bad Request | Invalid input, validation failure |
| 401 | Unauthorized | Missing/invalid JWT token |
| 404 | Not Found | Resource not found (user, profile) |
| 409 | Conflict | Email already registered |
| 423 | Locked | Account locked (too many failed logins) |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Database error, Claude API error |
| 503 | Service Unavailable | Database/Redis/Claude service down |

**Error Logging**:

Errors logged to console with context:
```json
{
  "message": "Error message",
  "stack": "Error stack trace (development only)",
  "path": "/api/personalize",
  "method": "POST",
  "timestamp": "2025-12-12T10:00:00.000Z"
}
```

---

## Rate Limiting

**Implementation**:
- Library: `express-rate-limit` v7+
- Store: Redis-backed (distributed rate limiting)
- Fallback: Memory store if Redis unavailable

**Rate Limits by Endpoint**:

| Endpoint | Limit | Window | Store |
|----------|-------|--------|-------|
| `/api/auth/signup` | Unlimited | - | - |
| `/api/auth/signin` | Unlimited | - | Account lockout after 3 failures |
| `/api/personalize` | 10 requests | 60 seconds | Redis per user |
| `/api/translate` | 5 requests | 60 seconds | Redis per IP |

**Rate Limit Headers**:

Response includes standard rate limit headers:
```
RateLimit-Limit: 10
RateLimit-Remaining: 7
RateLimit-Reset: 1639324800
```

**429 Response**:
```json
{
  "error": {
    "message": "Too many personalization requests. Please wait before trying again.",
    "code": 429,
    "retryAfter": 60
  }
}
```

**Redis Keys**:
- Format: `rate-limit:{endpoint}:{userId or IP}`
- TTL: 60 seconds (window duration)
- Atomic increment + expire on first hit

---

## Health Check

### GET /health

Check service status (database, Redis, Claude API).

**Response** (200 OK - All Healthy):
```json
{
  "status": "healthy",
  "timestamp": "2025-12-12T10:00:00.000Z",
  "service": "physical-ai-textbook-api",
  "version": "1.0.0",
  "services": {
    "database": true,
    "redis": true,
    "claude": true
  }
}
```

**Response** (503 Service Unavailable - Degraded):
```json
{
  "status": "degraded",
  "timestamp": "2025-12-12T10:00:00.000Z",
  "service": "physical-ai-textbook-api",
  "version": "1.0.0",
  "services": {
    "database": true,
    "redis": false, // Redis connection failed
    "claude": true
  }
}
```

**Service Checks**:
- **Database**: `await prisma.$connect()` + `$disconnect()`
- **Redis**: `await redis.connect()` + `ping()`
- **Claude**: Test API call with minimal token usage (10 tokens)

---

## Authentication Flow

### Signup Flow

```
1. Client → POST /api/auth/signup (email + password + profile)
2. Server validates input (email format, password strength)
3. Server checks email uniqueness
4. Server hashes password (bcrypt, 10 rounds)
5. Server creates User + UserProfile in transaction
6. Server generates JWT token (7-day expiration)
7. Server returns user + profile + token
8. Client stores token in localStorage
```

### Signin Flow

```
1. Client → POST /api/auth/signin (email + password + rememberMe)
2. Server finds user by email
3. Server checks account lock status
4. Server verifies password with bcrypt.compare()
5. On success: Generate JWT (1-day or 7-day), update last_login
6. On failure: Increment failed attempts in Redis, lock after 3 failures
7. Server returns user + profile + token
8. Client stores token in localStorage
```

### Authenticated Request Flow

```
1. Client → Request with Authorization: Bearer <token>
2. Server extracts token from header
3. Server verifies token with JWT secret
4. Server attaches userId + email to req.user
5. Server processes request with user context
6. Server returns response
```

---

## Examples

### Complete Signup + Personalize + Translate Workflow

```bash
# 1. Signup
curl -X POST http://localhost:4000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "Test123!@#",
    "profile": {
      "python_level": "Intermediate",
      "ros2_level": "Beginner",
      "gpu_available": "Yes",
      "hardware_tier": "Tier2",
      "primary_goal": "Learning"
    }
  }'

# Response includes token:
# { "user": {...}, "profile": {...}, "token": "eyJ..." }

# 2. Personalize Chapter (with token)
curl -X POST http://localhost:4000/api/personalize \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer eyJ..." \
  -d '{
    "chapterId": "ch1.1",
    "originalContent": "# ROS 2 Fundamentals..."
  }'

# 3. Translate Chapter (no auth required)
curl -X POST http://localhost:4000/api/translate \
  -H "Content-Type: application/json" \
  -d '{
    "chapterId": "ch1.1",
    "originalContent": "# ROS 2 Fundamentals...",
    "targetLanguage": "urdu"
  }'
```

---

## Security Considerations

1. **Password Security**:
   - Bcrypt with 10 rounds (balance security/performance)
   - Min 8 chars, requires number + special char
   - Passwords never logged or returned in responses

2. **JWT Security**:
   - Signed with HMAC SHA256
   - Secret must be ≥32 characters in production
   - Tokens include userId + email (no sensitive data)
   - No token blacklist (stateless auth)

3. **Account Lockout**:
   - 3 failed attempts → 15 minute lock
   - Failed attempts tracked in Redis (15-min TTL)
   - Lock expires automatically after 15 minutes

4. **CORS**:
   - Origin whitelist: `DOCUSAURUS_ORIGIN` env var
   - Credentials enabled for cookie-based auth (future)
   - Preflight requests cached

5. **Input Validation**:
   - Email: RFC 5322 regex, max 255 chars
   - Password: Strength rules enforced
   - ChapterId: Strict format validation
   - Content: Max 100KB to prevent DoS

6. **Rate Limiting**:
   - Prevents brute force attacks
   - Distributed (Redis-backed)
   - Per-user for auth, per-IP for public endpoints

---

## Monitoring & Observability

**Logs**:
- Request/response logged to console
- Error stack traces (development only)
- Health check status changes

**Metrics** (Future):
- Request latency (p50, p95, p99)
- Cache hit rates (personalization, translation)
- Claude API usage (tokens, cost)
- Rate limit violations

**Alerts** (Future):
- Health check failures
- High error rates (>5%)
- Claude API quota warnings

---

## Changelog

### v1.0.0 (2025-12-12)

**Added**:
- Authentication API (signup, signin, logout, profile management)
- Personalization API with Claude 3.5 Sonnet
- Translation API (English → Urdu)
- Redis caching (24hr personalization, 7-day translation)
- Rate limiting (10 req/min personalize, 5 req/min translate)
- Account lockout (3 failures, 15-min lock)
- Health check endpoint

**Security**:
- JWT authentication with 1-day/7-day expiration
- Bcrypt password hashing (10 rounds)
- Input validation and sanitization
- CORS protection

---

## Support

For issues or questions:
- **GitHub Issues**: https://github.com/your-org/physical-ai-textbook/issues
- **Email**: support@example.com
- **Documentation**: See `specs/008-bonus-features/` for spec, plan, tasks

---

**Last Updated**: 2025-12-12
**API Version**: 1.0.0
