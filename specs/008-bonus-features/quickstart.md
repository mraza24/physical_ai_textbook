# Quickstart Guide: Physical AI Textbook with Bonus Features

This guide will help you set up and run the Physical AI Textbook with authentication, personalization, and Urdu translation features.

## Prerequisites

- **Node.js**: 20 LTS or higher
- **Docker & Docker Compose**: For PostgreSQL and Redis
- **Git**: For cloning repository
- **Claude API Key**: From https://console.anthropic.com/

## Setup Steps

### 1. Clone Repository

```bash
git clone <repository-url>
cd physical_ai_textbook
```

### 2. Install Dependencies

```bash
# Install API dependencies
cd api
npm install

# Install Docusaurus dependencies
cd ../textbook
npm install
```

### 3. Start Docker Services

Start PostgreSQL and Redis containers:

```bash
cd ../api
docker-compose up -d
```

Verify services are running:

```bash
docker ps
# Should show postgres:15-alpine and redis:7-alpine containers
```

### 4. Configure Environment Variables

Create `.env` file in `api/` directory:

```bash
cd api
cp .env.example .env
```

Edit `.env` with your credentials:

```env
# Database
DATABASE_URL="postgresql://textbook_user:textbook_pass@localhost:5432/textbook_db"

# Redis
REDIS_URL="redis://localhost:6379"

# Claude API
CLAUDE_API_KEY="sk-ant-api03-..." # Your Claude API key from Anthropic Console

# JWT Secret (generate a strong random string)
JWT_SECRET="your-32-character-secret-key-change-in-production"

# Server
PORT=4000
NODE_ENV=development

# CORS
DOCUSAURUS_ORIGIN="http://localhost:3000"
```

### 5. Run Database Migrations

Initialize database schema with Prisma:

```bash
cd api
npx prisma generate
npx prisma migrate dev --name init
```

This creates:
- `users` table (id, email, password_hash, timestamps, account lock fields)
- `user_profiles` table (user_id FK, 5 background question enums, language preference)

### 6. Start Servers

**Terminal 1 - API Server**:

```bash
cd api
npm run dev
```

API should start at http://localhost:4000

Verify health check:
```bash
curl http://localhost:4000/health
```

Expected response:
```json
{
  "status": "healthy",
  "services": {
    "database": true,
    "redis": true,
    "claude": true
  }
}
```

**Terminal 2 - Docusaurus**:

```bash
cd textbook
npm start
```

Docusaurus should start at http://localhost:3000

### 7. Verify Features

#### Authentication (US1)

1. Navigate to http://localhost:3000/auth/signup
2. Create account:
   - Email: `test@example.com`
   - Password: `Test123!@#` (min 8 chars, 1 number, 1 special char)
   - Python Level: Select your experience
   - ROS 2 Level: Select your experience
   - GPU Available: Select your hardware
   - Hardware Tier: Select your tier
   - Primary Goal: Select your goal
3. Click "Sign Up" → should auto-login and redirect to home
4. Check profile at http://localhost:3000/auth/profile

#### Personalization (US2)

1. Ensure you're logged in
2. Open any chapter (e.g., Chapter 1.1)
3. Click "Personalize This Chapter" button
4. Wait 5-10 seconds for Claude API response
5. Verify content adapted to your profile:
   - Python explanations match your level
   - ROS 2 concepts expanded/minimized based on experience
   - Hardware recommendations match your tier
6. Click "Reset to Default" to restore original

#### Translation (US3)

1. Open any chapter
2. Click "Translate to Urdu" button
3. Wait 10-15 seconds for translation
4. Verify:
   - Text in Urdu (RTL layout)
   - Code blocks still in English
   - Technical terms transliterated (e.g., "ROS 2" → "آر او ایس 2")
   - Headings translated
5. Click "Show Original" to return to English

## Troubleshooting

### Database Connection Failed

**Error**: `P1001: Can't reach database server`

**Solution**:
```bash
# Check Docker containers
docker ps

# Restart PostgreSQL
docker-compose restart postgres

# Check logs
docker-compose logs postgres
```

### Redis Connection Failed

**Error**: `Redis connection refused`

**Solution**:
```bash
# Check Redis container
docker ps

# Restart Redis
docker-compose restart redis

# Test connection
redis-cli -h localhost -p 6379 ping
# Should return: PONG
```

### Claude API Error

**Error**: `401 Unauthorized` or `Invalid API key`

**Solution**:
1. Verify API key in `.env` starts with `sk-ant-api03-`
2. Check API key has not expired at https://console.anthropic.com/
3. Ensure `CLAUDE_API_KEY` has no quotes or extra spaces

### Prisma Migration Error

**Error**: `Environment variable not found: DATABASE_URL`

**Solution**:
```bash
# Verify .env file exists
cat api/.env | grep DATABASE_URL

# Re-run migration
cd api
npx prisma migrate dev
```

### Port Already in Use

**Error**: `EADDRINUSE: address already in use :::4000`

**Solution**:
```bash
# Find process using port 4000
lsof -i :4000

# Kill process (replace PID)
kill -9 <PID>

# Or use different port in .env
PORT=4001
```

### CORS Error in Browser

**Error**: `Access to fetch blocked by CORS policy`

**Solution**:
1. Verify `DOCUSAURUS_ORIGIN` in `.env` matches Docusaurus URL
2. Restart API server after changing `.env`
3. Check browser console for exact origin mismatch

## API Endpoints Reference

### Authentication Endpoints

| Method | Endpoint | Description | Auth Required |
|--------|----------|-------------|---------------|
| POST | `/api/auth/signup` | Create new account | No |
| POST | `/api/auth/signin` | Sign in with email/password | No |
| POST | `/api/auth/logout` | Logout current user | Yes |
| GET | `/api/auth/profile` | Get user profile | Yes |
| PUT | `/api/auth/profile` | Update user profile | Yes |

### Personalization Endpoints

| Method | Endpoint | Description | Auth Required | Rate Limit |
|--------|----------|-------------|---------------|------------|
| POST | `/api/personalize` | Personalize chapter content | Yes | 10 req/min |

### Translation Endpoints

| Method | Endpoint | Description | Auth Required | Rate Limit |
|--------|----------|-------------|---------------|------------|
| POST | `/api/translate` | Translate chapter to Urdu | No | 5 req/min |

## Development Notes

### Docker Compose Services

**PostgreSQL**:
- Image: `postgres:15-alpine`
- Port: `5432`
- Database: `textbook_db`
- User: `textbook_user`
- Password: `textbook_pass`
- Health check: `pg_isready`
- Volume: `postgres_data`

**Redis**:
- Image: `redis:7-alpine`
- Port: `6379`
- Health check: `redis-cli ping`
- Volume: `redis_data`

### Caching Strategy

**Personalization Cache**:
- Key format: `personalize:{chapterId}:{profileHash}`
- TTL: 24 hours
- Invalidation: On profile update

**Translation Cache**:
- Key format: `translate:{chapterId}:urdu`
- TTL: 7 days
- No automatic invalidation (translations stable)

### Database Schema

**User Model**:
```prisma
model User {
  id               String    @id @default(uuid())
  email            String    @unique
  password_hash    String
  created_at       DateTime  @default(now())
  last_login       DateTime?
  is_locked        Boolean   @default(false)
  lock_expires_at  DateTime?
  profile          UserProfile?
}
```

**UserProfile Model**:
```prisma
model UserProfile {
  id                   String              @id @default(uuid())
  user_id              String              @unique
  python_level         PythonLevel
  ros2_level           ROS2Level
  gpu_available        GPUAvailability
  hardware_tier        HardwareTier
  primary_goal         PrimaryGoal
  language_preference  LanguagePreference  @default(English)
  updated_at           DateTime            @default(now())
  user                 User                @relation(fields: [user_id], references: [id], onDelete: Cascade)
}
```

## Next Steps

1. **Theme Integration** (Optional): Swizzle Docusaurus theme to inject Personalize/Translate buttons into chapter pages
2. **Production Deployment**: Configure environment variables for production (strong JWT secret, production database, secure CORS)
3. **Performance Testing**: Test personalization/translation with concurrent users
4. **Content Migration**: Add 16 existing textbook chapters to test personalization at scale

## Support

For issues or questions:
- Check logs: `docker-compose logs` (API), browser console (frontend)
- Review error middleware output in API terminal
- Verify all services healthy at http://localhost:4000/health
- Consult plan.md and spec.md in `specs/008-bonus-features/`
