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
 * Health check function
 * Verifies database connectivity and returns server time
 *
 * @returns Promise<Date> Current database server timestamp
 * @throws Error if database connection fails
 */
export async function healthCheck(): Promise<Date> {
  try {
    const result = await sql`SELECT NOW() as server_time`;
    return new Date(result[0].server_time);
  } catch (error) {
    throw new Error(
      `Database health check failed: ${error instanceof Error ? error.message : 'Unknown error'}`
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
