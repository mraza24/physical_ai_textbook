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
 */

// Validate required environment variables
if (!process.env.JWT_SECRET) {
  throw new Error(
    'JWT_SECRET environment variable is required. ' +
    'Generate one with: openssl rand -base64 32'
  );
}

export const auth = betterAuth({
  // ✅ Trusted Origins Configuration (Fixed: Only one property)
  trustedOrigins: [
    'https://physical-ai-textbook-jet.vercel.app', // Production frontend (Vercel)
    'http://localhost:3000', // Local development
    'http://localhost:3001', // Alternative local port
    ...(process.env.TRUSTED_ORIGINS?.split(',').map(origin => origin.trim()) || []),
  ],

  // ✅ Backend Base URL (CRITICAL for Render)
  baseURL: 'https://physical-ai-auth-backend.onrender.com',

  // Database adapter configuration
  database: drizzleAdapter(db, {
    provider: 'pg', // PostgreSQL (Neon)
    schema, // Pass schema object so Better-Auth can find all tables
  }),

  // JWT secret for signing tokens
  secret: process.env.JWT_SECRET!,

  // Session configuration
  session: {
    expiresIn: 60 * 60 * 24, // 24 hours
    updateAge: 60 * 60, // Refresh token every hour
    cookieCache: {
        enabled: true,
        maxAge: 5 * 60 // Cache for 5 minutes
    }
  },

  // Email and password authentication
  emailAndPassword: {
    enabled: true,
    bcryptCost: 12,
  },

  // Advanced options for Production
  advanced: {
    useSecureCookies: process.env.NODE_ENV === 'production',
    cookiePrefix: 'auth',
    crossSubDomainCookies: {
      enabled: false, 
    },
  },
});

/**
 * Export auth handlers for Express integration
 */
export const authHandler = toNodeHandler(auth);

/**
 * Type-safe session interface
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