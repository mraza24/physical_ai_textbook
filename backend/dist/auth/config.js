"use strict";
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || (function () {
    var ownKeys = function(o) {
        ownKeys = Object.getOwnPropertyNames || function (o) {
            var ar = [];
            for (var k in o) if (Object.prototype.hasOwnProperty.call(o, k)) ar[ar.length] = k;
            return ar;
        };
        return ownKeys(o);
    };
    return function (mod) {
        if (mod && mod.__esModule) return mod;
        var result = {};
        if (mod != null) for (var k = ownKeys(mod), i = 0; i < k.length; i++) if (k[i] !== "default") __createBinding(result, mod, k[i]);
        __setModuleDefault(result, mod);
        return result;
    };
})();
Object.defineProperty(exports, "__esModule", { value: true });
exports.authHandler = exports.auth = void 0;
const better_auth_1 = require("better-auth");
const node_1 = require("better-auth/node");
const drizzle_1 = require("better-auth/adapters/drizzle");
const connection_1 = require("../db/connection");
const schema = __importStar(require("../db/schema"));
const dotenv = __importStar(require("dotenv"));
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
    throw new Error('JWT_SECRET environment variable is required. ' +
        'Generate one with: openssl rand -base64 32');
}
exports.auth = (0, better_auth_1.betterAuth)({
    trustedOrigins: [
        'https://physical-ai-textbook-jet.vercel.app', // آپ کا ورسل فرنٹ اینڈ
        'http://localhost:3000', // لوکل ٹیسٹنگ کے لیے (اگر ضرورت ہو)
    ],
    // Database adapter configuration
    database: (0, drizzle_1.drizzleAdapter)(connection_1.db, {
        provider: 'pg', // PostgreSQL (Neon)
        schema, // Pass schema object so Better-Auth can find all tables
    }),
    baseURL: 'https://physical-ai-auth-backend.onrender.com',
    // JWT secret for signing tokens
    secret: process.env.JWT_SECRET,
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
exports.authHandler = (0, node_1.toNodeHandler)(exports.auth);
