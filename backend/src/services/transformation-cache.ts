import crypto from 'crypto';
import { db } from '../db/connection';
import { transformationCache, type NewTransformationCache, type TransformationCache } from '../db/schema';
import { eq, lt } from 'drizzle-orm';

/**
 * Transformation Cache Service
 *
 * Manages caching of AI-transformed content (personalized/translated chapters)
 * with 5-minute TTL to reduce LLM API costs.
 *
 * Features:
 * - SHA-256 cache key generation from chapter path + user profile + transformation type
 * - Automatic expiration (5-minute TTL per FR-036)
 * - JSONB metadata storage for tracking transformations
 * - Hit/miss metrics for monitoring
 *
 * Cache Key Format:
 * SHA-256(chapterPath | softwareBackground | hardwareExperience | transformationType)
 */

export interface CacheMetadata {
  changes_made?: number;
  complexity_level?: string;
  model?: string;
  cached?: boolean;
  preserved_terms?: string[];
  [key: string]: any;
}

export interface UserProfileCacheInput {
  software_background: string;
  hardware_experience: string;
}

export type TransformationType = 'personalize' | 'translate';

/**
 * Generate SHA-256 cache key from input parameters
 *
 * Ensures same chapter + profile + type = same cache key
 *
 * @param chapterPath - Docusaurus path (e.g., "/docs/module1/sensors")
 * @param userProfile - Software/hardware background
 * @param transformationType - "personalize" or "translate"
 * @returns 64-character hex string (SHA-256 hash)
 */
export function generateCacheKey(
  chapterPath: string,
  userProfile: UserProfileCacheInput,
  transformationType: TransformationType
): string {
  const data = `${chapterPath}|${userProfile.software_background}|${userProfile.hardware_experience}|${transformationType}`;
  return crypto.createHash('sha256').update(data).digest('hex');
}

/**
 * Store transformed content in cache
 *
 * @param cacheKey - SHA-256 hash from generateCacheKey()
 * @param transformedContent - Markdown output from LLM
 * @param metadata - Transformation details (model, changes, etc.)
 * @param ttlMinutes - Time-to-live in minutes (default: 5 from FR-036)
 * @returns Created cache entry
 */
export async function setCacheEntry(
  cacheKey: string,
  transformedContent: string,
  metadata: CacheMetadata = {},
  ttlMinutes: number = 5
): Promise<TransformationCache> {
  const now = new Date();
  const expiresAt = new Date(now.getTime() + ttlMinutes * 60 * 1000);

  const entry: NewTransformationCache = {
    cache_key: cacheKey,
    transformed_content: transformedContent,
    transformation_metadata: metadata,
    created_at: now,
    expires_at: expiresAt,
  };

  const result = await db.insert(transformationCache).values(entry).returning();
  return result[0];
}

/**
 * Retrieve cached content if still valid
 *
 * Returns null if:
 * - Cache key not found
 * - Entry has expired
 *
 * @param cacheKey - SHA-256 hash from generateCacheKey()
 * @returns Cached entry or null
 */
export async function getCacheEntry(
  cacheKey: string
): Promise<TransformationCache | null> {
  const now = new Date();

  const results = await db
    .select()
    .from(transformationCache)
    .where(eq(transformationCache.cache_key, cacheKey))
    .limit(1);

  if (results.length === 0) {
    return null; // Cache miss
  }

  const entry = results[0];

  // Check expiration
  if (entry.expires_at <= now) {
    // Expired - delete and return null
    await db
      .delete(transformationCache)
      .where(eq(transformationCache.cache_key, cacheKey));
    return null;
  }

  return entry; // Cache hit
}

/**
 * Delete all expired cache entries
 *
 * Should be called hourly via cron job or serverless function
 *
 * @returns Number of deleted entries
 */
export async function cleanupExpiredEntries(): Promise<number> {
  const now = new Date();

  const deleted = await db
    .delete(transformationCache)
    .where(lt(transformationCache.expires_at, now))
    .returning({ id: transformationCache.id });

  return deleted.length;
}

/**
 * Get cache statistics
 *
 * Useful for monitoring and optimizing cache TTL
 *
 * @returns Cache stats object
 */
export async function getCacheStats() {
  const now = new Date();

  const allEntries = await db.select().from(transformationCache);

  const total = allEntries.length;
  const active = allEntries.filter(e => e.expires_at > now).length;
  const expired = total - active;

  // Calculate average TTL remaining for active entries
  const activeEntries = allEntries.filter(e => e.expires_at > now);
  const avgTtlMs = activeEntries.length > 0
    ? activeEntries.reduce((sum, e) => sum + (e.expires_at.getTime() - now.getTime()), 0) / activeEntries.length
    : 0;

  return {
    total,
    active,
    expired,
    avgTtlRemainingSeconds: Math.round(avgTtlMs / 1000),
  };
}

/**
 * Delete specific cache entry
 *
 * Useful for cache invalidation when content is updated
 *
 * @param cacheKey - SHA-256 hash to delete
 * @returns True if entry was deleted, false if not found
 */
export async function deleteCacheEntry(cacheKey: string): Promise<boolean> {
  const deleted = await db
    .delete(transformationCache)
    .where(eq(transformationCache.cache_key, cacheKey))
    .returning({ id: transformationCache.id });

  return deleted.length > 0;
}

/**
 * High-level cache wrapper for transformations
 *
 * Combines cache key generation, get/set logic
 *
 * @param chapterPath - Docusaurus chapter path
 * @param userProfile - User software/hardware background
 * @param transformationType - "personalize" or "translate"
 * @param transformFn - Async function that performs transformation (calls LLM)
 * @returns Transformed content (from cache or fresh)
 */
export async function getOrTransform(
  chapterPath: string,
  userProfile: UserProfileCacheInput,
  transformationType: TransformationType,
  transformFn: () => Promise<{ content: string; metadata: CacheMetadata }>
): Promise<{ content: string; metadata: CacheMetadata; fromCache: boolean }> {
  const cacheKey = generateCacheKey(chapterPath, userProfile, transformationType);

  // Try cache first
  const cached = await getCacheEntry(cacheKey);
  if (cached) {
    const metadata = (cached.transformation_metadata as CacheMetadata) || {};
    return {
      content: cached.transformed_content,
      metadata: { ...metadata, cached: true },
      fromCache: true,
    };
  }

  // Cache miss - perform transformation
  const { content, metadata } = await transformFn();

  // Store in cache
  await setCacheEntry(cacheKey, content, { ...metadata, cached: false });

  return {
    content,
    metadata: { ...metadata, cached: false },
    fromCache: false,
  };
}
