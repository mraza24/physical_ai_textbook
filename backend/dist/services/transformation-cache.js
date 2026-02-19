"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.generateCacheKey = generateCacheKey;
exports.setCacheEntry = setCacheEntry;
exports.getCacheEntry = getCacheEntry;
exports.cleanupExpiredEntries = cleanupExpiredEntries;
exports.getCacheStats = getCacheStats;
exports.deleteCacheEntry = deleteCacheEntry;
exports.getOrTransform = getOrTransform;
const crypto_1 = __importDefault(require("crypto"));
const connection_1 = require("../db/connection");
const schema_1 = require("../db/schema");
const drizzle_orm_1 = require("drizzle-orm");
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
function generateCacheKey(chapterPath, userProfile, transformationType) {
    const data = `${chapterPath}|${userProfile.software_background}|${userProfile.hardware_experience}|${transformationType}`;
    return crypto_1.default.createHash('sha256').update(data).digest('hex');
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
async function setCacheEntry(cacheKey, transformedContent, metadata = {}, ttlMinutes = 5) {
    const now = new Date();
    const expiresAt = new Date(now.getTime() + ttlMinutes * 60 * 1000);
    const entry = {
        cache_key: cacheKey,
        transformed_content: transformedContent,
        transformation_metadata: metadata,
        created_at: now,
        expires_at: expiresAt,
    };
    const result = await connection_1.db.insert(schema_1.transformationCache).values(entry).returning();
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
async function getCacheEntry(cacheKey) {
    const now = new Date();
    const results = await connection_1.db
        .select()
        .from(schema_1.transformationCache)
        .where((0, drizzle_orm_1.eq)(schema_1.transformationCache.cache_key, cacheKey))
        .limit(1);
    if (results.length === 0) {
        return null; // Cache miss
    }
    const entry = results[0];
    // Check expiration
    if (entry.expires_at <= now) {
        // Expired - delete and return null
        await connection_1.db
            .delete(schema_1.transformationCache)
            .where((0, drizzle_orm_1.eq)(schema_1.transformationCache.cache_key, cacheKey));
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
async function cleanupExpiredEntries() {
    const now = new Date();
    const deleted = await connection_1.db
        .delete(schema_1.transformationCache)
        .where((0, drizzle_orm_1.lt)(schema_1.transformationCache.expires_at, now))
        .returning({ id: schema_1.transformationCache.id });
    return deleted.length;
}
/**
 * Get cache statistics
 *
 * Useful for monitoring and optimizing cache TTL
 *
 * @returns Cache stats object
 */
async function getCacheStats() {
    const now = new Date();
    const allEntries = await connection_1.db.select().from(schema_1.transformationCache);
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
async function deleteCacheEntry(cacheKey) {
    const deleted = await connection_1.db
        .delete(schema_1.transformationCache)
        .where((0, drizzle_orm_1.eq)(schema_1.transformationCache.cache_key, cacheKey))
        .returning({ id: schema_1.transformationCache.id });
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
async function getOrTransform(chapterPath, userProfile, transformationType, transformFn) {
    const cacheKey = generateCacheKey(chapterPath, userProfile, transformationType);
    // Try cache first
    const cached = await getCacheEntry(cacheKey);
    if (cached) {
        const metadata = cached.transformation_metadata || {};
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
