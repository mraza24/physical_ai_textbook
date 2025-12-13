import rateLimit from 'express-rate-limit';
import redis from '../config/redis';

/**
 * Rate limiting middleware for personalization endpoint
 * 10 requests per 60 seconds per user
 */
export const personalizeRateLimiter = rateLimit({
  windowMs: 60 * 1000, // 1 minute
  max: 10, // 10 requests per window
  message: {
    error: {
      message: 'Too many personalization requests. Please wait before trying again.',
      code: 429,
      retryAfter: 60,
    },
  },
  standardHeaders: true, // Return rate limit info in `RateLimit-*` headers
  legacyHeaders: false, // Disable `X-RateLimit-*` headers
  // Use Redis for distributed rate limiting (optional, falls back to memory)
  store: createRedisStore('personalize'),
});

/**
 * Rate limiting middleware for translation endpoint
 * 5 requests per 60 seconds per user
 */
export const translateRateLimiter = rateLimit({
  windowMs: 60 * 1000, // 1 minute
  max: 5, // 5 requests per window
  message: {
    error: {
      message: 'Too many translation requests. Please wait before trying again.',
      code: 429,
      retryAfter: 60,
    },
  },
  standardHeaders: true,
  legacyHeaders: false,
  store: createRedisStore('translate'),
});

/**
 * Create Redis-backed rate limit store
 * Falls back to memory store if Redis unavailable
 */
function createRedisStore(prefix: string) {
  try {
    // Simple Redis-backed store using increment + expire
    return {
      async increment(key: string): Promise<{ totalHits: number; resetTime: Date }> {
        const redisKey = `rate-limit:${prefix}:${key}`;
        const ttl = 60; // 1 minute TTL

        try {
          // Increment counter
          const hits = await redis.incr(redisKey);

          // Set expiry on first hit
          if (hits === 1) {
            await redis.expire(redisKey, ttl);
          }

          // Get TTL to calculate reset time
          const remaining = await redis.ttl(redisKey);
          const resetTime = new Date(Date.now() + remaining * 1000);

          return { totalHits: hits, resetTime };
        } catch (error) {
          console.error('Redis rate limit error:', error);
          // Fallback: allow request if Redis fails
          return { totalHits: 0, resetTime: new Date(Date.now() + ttl * 1000) };
        }
      },

      async decrement(key: string): Promise<void> {
        const redisKey = `rate-limit:${prefix}:${key}`;
        try {
          await redis.decr(redisKey);
        } catch (error) {
          console.error('Redis rate limit decrement error:', error);
        }
      },

      async resetKey(key: string): Promise<void> {
        const redisKey = `rate-limit:${prefix}:${key}`;
        try {
          await redis.del(redisKey);
        } catch (error) {
          console.error('Redis rate limit reset error:', error);
        }
      },
    };
  } catch (error) {
    console.error('Failed to create Redis rate limit store:', error);
    // Return undefined to fall back to express-rate-limit's default memory store
    return undefined;
  }
}
