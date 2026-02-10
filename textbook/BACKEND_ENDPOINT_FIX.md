# Backend Endpoint Fix for 404 Error

## Problem
Frontend is calling `http://localhost:4000/api/auth/signup` but backend returns 404.

## Solution: Update Backend index.ts

### Option 1: Better Auth (Recommended if using better-auth library)

```typescript
// backend/src/index.ts
import express from 'express';
import cors from 'cors';
import { betterAuth } from 'better-auth';

const app = express();

// CORS configuration
app.use(cors({
  origin: 'http://localhost:3000',
  credentials: true
}));

app.use(express.json());

// Better Auth configuration
const auth = betterAuth({
  database: {
    // Your database config
  },
  emailAndPassword: {
    enabled: true,
  },
});

// Mount Better Auth routes at /api/auth
app.use('/api/auth', auth.handler);

// OR if Better Auth uses a different pattern:
app.all('/api/auth/*', auth.handler);

const PORT = 4000;
app.listen(PORT, () => {
  console.log(`Server running on http://localhost:${PORT}`);
});
```

### Option 2: Manual Route (If NOT using better-auth)

```typescript
// backend/src/index.ts
import express from 'express';
import cors from 'cors';
import bcrypt from 'bcrypt';
import jwt from 'jsonwebtoken';
import { pool } from './db'; // Your database connection

const app = express();

app.use(cors({
  origin: 'http://localhost:3000',
  credentials: true
}));

app.use(express.json());

// Signup endpoint
app.post('/api/auth/signup', async (req, res) => {
  try {
    const {
      email,
      password,
      software_background,
      hardware_experience,
      language_preference
    } = req.body;

    // Validate input
    if (!email || !password) {
      return res.status(400).json({ message: 'Email and password required' });
    }

    // Check if user exists
    const existingUser = await pool.query(
      'SELECT * FROM users WHERE email = $1',
      [email]
    );

    if (existingUser.rows.length > 0) {
      return res.status(409).json({ message: 'Email already registered' });
    }

    // Hash password
    const password_hash = await bcrypt.hash(password, 10);

    // Create user
    const userResult = await pool.query(
      'INSERT INTO users (email, password_hash, created_at) VALUES ($1, $2, NOW()) RETURNING id, email',
      [email, password_hash]
    );

    const userId = userResult.rows[0].id;

    // Create user profile
    await pool.query(
      `INSERT INTO user_profiles
       (user_id, software_background, hardware_experience, language_preference, created_at)
       VALUES ($1, $2, $3, $4, NOW())`,
      [userId, software_background, hardware_experience, language_preference]
    );

    // Generate JWT token (optional)
    const token = jwt.sign(
      { userId, email },
      process.env.JWT_SECRET || 'your-secret-key',
      { expiresIn: '7d' }
    );

    // Return success
    return res.status(201).json({
      message: 'User created successfully',
      session: { token },
      user: {
        id: userId,
        email,
        software_background,
        hardware_experience,
      }
    });

  } catch (error) {
    console.error('Signup error:', error);
    return res.status(500).json({
      message: 'Internal server error',
      error: error.message
    });
  }
});

const PORT = 4000;
app.listen(PORT, () => {
  console.log(`Server running on http://localhost:${PORT}`);
});
```

### Option 3: Express Router Pattern

```typescript
// backend/src/routes/auth.ts
import { Router } from 'express';

const router = Router();

router.post('/signup', async (req, res) => {
  // ... signup logic from Option 2 ...
});

router.post('/login', async (req, res) => {
  // ... login logic ...
});

export default router;

// backend/src/index.ts
import authRoutes from './routes/auth';

app.use('/api/auth', authRoutes);
```

## Test Endpoints

### Test if endpoint exists:
```bash
curl -X POST http://localhost:4000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "password123",
    "software_background": "Intermediate",
    "hardware_experience": "Basic",
    "language_preference": "English"
  }'
```

### Expected response:
```json
{
  "message": "User created successfully",
  "session": { "token": "jwt_token_here" },
  "user": {
    "id": 1,
    "email": "test@example.com",
    "software_background": "Intermediate",
    "hardware_experience": "Basic"
  }
}
```

## Debugging

### 1. Check all registered routes:
```typescript
// Add this to your backend
app._router.stack.forEach((middleware) => {
  if (middleware.route) {
    console.log('Route:', middleware.route.path);
  } else if (middleware.name === 'router') {
    middleware.handle.stack.forEach((handler) => {
      if (handler.route) {
        console.log('Route:', handler.route.path);
      }
    });
  }
});
```

### 2. Check backend logs:
When you start the backend, you should see:
```
Server running on http://localhost:4000
Route: /api/auth/signup
Route: /api/auth/login
```

### 3. Enable request logging:
```typescript
app.use((req, res, next) => {
  console.log(`${req.method} ${req.path}`);
  next();
});
```

This will log every request to the console.
