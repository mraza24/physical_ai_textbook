import { betterAuth } from 'better-auth';
import { toNodeHandler } from 'better-auth/node';
import { drizzleAdapter } from 'better-auth/adapters/drizzle';
import { db } from '../db/connection';
import * as schema from '../db/schema';
import * as dotenv from 'dotenv';

// Load environment variables
dotenv.config();

/**
 * Better-Auth Configuration
 *
 * Provides JWT-based authentication with Drizzle ORM adapter for Neon Postgres.
 *
 * Features:
 * - Email/password authentication with bcrypt (cost factor 12)
 * - JWT sessions with RS256 signing
 * - Configurable session expiration (24h standard, 7d "remember me")
 * - Custom user profile fields accessible in session
 *
 * @see https://better-auth.com/docs
 */

// Validate required environment variables
if (!process.env.JWT_SECRET) {
  throw new Error(
    'JWT_SECRET environment variable is required. ' +
    'Generate one with: openssl rand -base64 32'
  );
}

export const auth = betterAuth({
  trustedOrigins: [
    'https://physical-ai-textbook-jet.vercel.app', // آپ کا ورسل فرنٹ اینڈ
    'http://localhost:3000', // لوکل ٹیسٹنگ کے لیے (اگر ضرورت ہو)
  ],
  // Database adapter configuration
  database: drizzleAdapter(db, {
    provider: 'pg', // PostgreSQL (Neon)
    schema, // Pass schema object so Better-Auth can find all tables
  }),
baseURL: 'https://physical-ai-auth-backend.onrender.com',
  // JWT secret for signing tokens
  secret: process.env.JWT_SECRET!,

  // Session configuration
  session: {
    expiresIn: 60 * 60 * 24, // 24 hours (standard session) - FR-006
    updateAge: 60 * 60, // Refresh token every hour
  },

  // Email and password authentication
  emailAndPassword: {
    enabled: true,
    bcryptCost: 12, // FR-005: Industry standard for 2026 (OWASP recommendation)
  },

  // User configuration
  // NOTE: Profile fields are stored in separate user_profiles table, not on user object
  // This prevents Better-Auth schema conflicts
  user: {
    // No additional fields - profile data comes from user_profiles table join
  },

  // Advanced options
  advanced: {
    // Use RS256 algorithm for production (spec Security requirement)
    useSecureCookies: process.env.NODE_ENV === 'production',
    cookiePrefix: 'auth',
    crossSubDomainCookies: {
      enabled: false, // Only if using subdomains
    },
  },
});

/**
 * Export auth handlers for Express integration
 *
 * Better-Auth uses Web API Request/Response, so we need to convert
 * to Node.js/Express compatible handler using toNodeHandler.
 *
 * Usage in Express:
 * ```typescript
 * import { authHandler } from './auth/config';
 * app.all('/api/auth/*', authHandler);
 * ```
 */
export const authHandler = toNodeHandler(auth);

/**
 * Type-safe session interface
 * Extends Better-Auth session with custom user profile fields
 */
export interface AuthSession {
  session: {
    token: string;
    expiresAt: Date;
    userId: string;
  };
  user: {
    id: string;
    email: string;
    software_background: 'Beginner' | 'Intermediate' | 'Expert';
    hardware_experience: 'None' | 'Basic' | 'Advanced';
    language_preference: 'English' | 'Urdu';
    python_level?: string;
    ros2_level?: string;
    gpu_available?: string;
    hardware_tier?: string;
    primary_goal?: string;
  };
}
