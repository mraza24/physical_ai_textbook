import { neon } from '@neondatabase/serverless';
import { drizzle } from 'drizzle-orm/neon-http';
import * as schema from './schema';
import * as dotenv from 'dotenv';
import * as path from 'path';

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
  throw new Error(
    'DATABASE_URL environment variable is not set. ' +
    'Please configure it in backend/.env file. ' +
    'Get your connection string from: https://console.neon.tech'
  );
}

/**
 * Neon SQL client (HTTP driver)
 * Executes raw SQL queries via Neon's HTTP API
 */
export const sql = neon(process.env.DATABASE_URL);

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
export const db = drizzle(sql, { schema });

/**
 * Health check function with timeout and retry logic
 * Verifies database connectivity and returns server time
 *
 * @param timeoutMs - Maximum time to wait for database response (default: 5000ms)
 * @param retries - Number of retry attempts (default: 2)
 * @returns Promise<Date> Current database server timestamp
 * @throws Error if database connection fails after all retries
 */
export async function healthCheck(timeoutMs: number = 5000, retries: number = 2): Promise<Date> {
  let lastError: Error | null = null;

  for (let attempt = 0; attempt <= retries; attempt++) {
    try {
      // Create timeout promise
      const timeoutPromise = new Promise<never>((_, reject) => {
        setTimeout(() => reject(new Error('Database health check timeout')), timeoutMs);
      });

      // Race between database query and timeout
      const queryPromise = sql`SELECT NOW() as server_time`;
      const result = await Promise.race([queryPromise, timeoutPromise]);

      if (Array.isArray(result) && result.length > 0 && result[0].server_time) {
        return new Date(result[0].server_time);
      }

      throw new Error('Invalid database response format');

    } catch (error) {
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
    throw new Error(
      `Database health check failed: Cannot connect to Neon database.\n` +
      `  Possible causes:\n` +
      `  1. DATABASE_URL is incorrect or malformed\n` +
      `  2. Neon database is paused (wake it up at https://console.neon.tech)\n` +
      `  3. Network connectivity issues\n` +
      `  4. Database does not exist\n` +
      `  Original error: ${errorMessage}`
    );
  } else if (errorMessage.includes('timeout')) {
    throw new Error(
      `Database health check failed: Connection timeout after ${timeoutMs}ms.\n` +
      `  The database might be slow to respond or unavailable.\n` +
      `  Original error: ${errorMessage}`
    );
  } else {
    throw new Error(
      `Database health check failed: ${errorMessage}`
    );
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
export async function cleanupExpiredCache(): Promise<number> {
  const result = await sql`
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
export async function cleanupExpiredSessions(): Promise<number> {
  const result = await sql`
    DELETE FROM session
    WHERE expires_at < NOW()
    RETURNING session_token
  `;
  return result.length;
}
