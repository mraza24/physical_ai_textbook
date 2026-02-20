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
exports.db = exports.sql = void 0;
exports.healthCheck = healthCheck;
exports.cleanupExpiredCache = cleanupExpiredCache;
exports.cleanupExpiredSessions = cleanupExpiredSessions;
const serverless_1 = require("@neondatabase/serverless");
const neon_http_1 = require("drizzle-orm/neon-http");
const schema = __importStar(require("./schema"));
const dotenv = __importStar(require("dotenv"));
const path = __importStar(require("path"));
// Load environment variables BEFORE any usage
dotenv.config({ path: path.resolve(__dirname, '../../.env') });
/**
 * Database Connection Module
 *
 * Uses Neon Serverless Postgres with HTTP driver for optimal serverless performance:
 * - No connection pooling overhead
 * - Auto-scaling with Neon's serverless architecture
 * - HTTP-based queries (no WebSocket/TCP connection management)
 *
 * @see https://neon.tech/docs/serverless/serverless-driver
 */
// Validate DATABASE_URL environment variable
if (!process.env.DATABASE_URL) {
    throw new Error('DATABASE_URL environment variable is not set. ' +
        'Please configure it in backend/.env file. ' +
        'Get your connection string from: https://console.neon.tech');
}
/**
 * Neon SQL client (HTTP driver)
 * Executes raw SQL queries via Neon's HTTP API
 */
exports.sql = (0, serverless_1.neon)(process.env.DATABASE_URL);
/**
 * Drizzle ORM instance
 * Provides type-safe database operations with Neon serverless backend
 *
 * Schema includes:
 * - user: Better-Auth managed authentication
 * - session: JWT-based session management
 * - user_profiles: Custom user personalization data
 * - transformation_cache: AI content transformation cache (5-min TTL)
 */
exports.db = (0, neon_http_1.drizzle)(exports.sql, { schema });
/**
 * Health check function with timeout and retry logic
 * Verifies database connectivity and returns server time
 *
 * @param timeoutMs - Maximum time to wait for database response (default: 5000ms)
 * @param retries - Number of retry attempts (default: 2)
 * @returns Promise<Date> Current database server timestamp
 * @throws Error if database connection fails after all retries
 */
async function healthCheck(timeoutMs = 5000, retries = 2) {
    let lastError = null;
    for (let attempt = 0; attempt <= retries; attempt++) {
        try {
            // Create timeout promise
            const timeoutPromise = new Promise((_, reject) => {
                setTimeout(() => reject(new Error('Database health check timeout')), timeoutMs);
            });
            // Race between database query and timeout
            const queryPromise = (0, exports.sql) `SELECT NOW() as server_time`;
            const result = await Promise.race([queryPromise, timeoutPromise]);
            if (Array.isArray(result) && result.length > 0 && result[0].server_time) {
                return new Date(result[0].server_time);
            }
            throw new Error('Invalid database response format');
        }
        catch (error) {
            lastError = error instanceof Error ? error : new Error(String(error));
            if (attempt < retries) {
                console.warn(`⚠️  Database health check attempt ${attempt + 1} failed, retrying...`);
                // Exponential backoff: wait 1s, then 2s
                await new Promise(resolve => setTimeout(resolve, 1000 * (attempt + 1)));
            }
        }
    }
    // All retries exhausted
    const errorMessage = lastError?.message || 'Unknown error';
    // Provide helpful error messages based on error type
    if (errorMessage.includes('fetch failed') || errorMessage.includes('ENOTFOUND')) {
        throw new Error(`Database health check failed: Cannot connect to Neon database.\n` +
            `  Possible causes:\n` +
            `  1. DATABASE_URL is incorrect or malformed\n` +
            `  2. Neon database is paused (wake it up at https://console.neon.tech)\n` +
            `  3. Network connectivity issues\n` +
            `  4. Database does not exist\n` +
            `  Original error: ${errorMessage}`);
    }
    else if (errorMessage.includes('timeout')) {
        throw new Error(`Database health check failed: Connection timeout after ${timeoutMs}ms.\n` +
            `  The database might be slow to respond or unavailable.\n` +
            `  Original error: ${errorMessage}`);
    }
    else {
        throw new Error(`Database health check failed: ${errorMessage}`);
    }
}
/**
 * Cleanup expired cache entries
 * Should be run hourly via cron job or serverless function
 *
 * Deletes transformation_cache entries where expires_at < NOW()
 *
 * @returns Promise<number> Number of deleted rows
 */
async function cleanupExpiredCache() {
    const result = await (0, exports.sql) `
    DELETE FROM transformation_cache
    WHERE expires_at < NOW()
    RETURNING id
  `;
    return result.length;
}
/**
 * Cleanup expired sessions
 * Should be run hourly via cron job or serverless function
 *
 * Deletes session entries where expires_at < NOW()
 *
 * @returns Promise<number> Number of deleted rows
 */
async function cleanupExpiredSessions() {
    const result = await (0, exports.sql) `
    DELETE FROM session
    WHERE expires_at < NOW()
    RETURNING session_token
  `;
    return result.length;
}
