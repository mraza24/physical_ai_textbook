import redis from '../config/redis';

export class CacheService {
  /**
   * Get value from cache
   */
  async get(key: string): Promise<string | null> {
    try {
      const value = await redis.get(key);
      return value;
    } catch (error) {
      console.error(`Cache GET error for key ${key}:`, error);
      return null;
    }
  }

  /**
   * Set value in cache with TTL (in seconds)
   */
  async set(key: string, value: string, ttlSeconds: number): Promise<boolean> {
    try {
      await redis.setex(key, ttlSeconds, value);
      return true;
    } catch (error) {
      console.error(`Cache SET error for key ${key}:`, error);
      return false;
    }
  }

  /**
   * Delete a specific key
   */
  async delete(key: string): Promise<boolean> {
    try {
      await redis.del(key);
      return true;
    } catch (error) {
      console.error(`Cache DELETE error for key ${key}:`, error);
      return false;
    }
  }

  /**
   * Invalidate all keys matching a pattern
   * Example: invalidatePattern('personalize:ch1.1:*')
   */
  async invalidatePattern(pattern: string): Promise<number> {
    try {
      const keys = await redis.keys(pattern);
      if (keys.length === 0) return 0;

      const result = await redis.del(...keys);
      console.log(`Invalidated ${result} keys matching pattern: ${pattern}`);
      return result;
    } catch (error) {
      console.error(`Cache INVALIDATE error for pattern ${pattern}:`, error);
      return 0;
    }
  }

  /**
   * Check if key exists in cache
   */
  async exists(key: string): Promise<boolean> {
    try {
      const result = await redis.exists(key);
      return result === 1;
    } catch (error) {
      console.error(`Cache EXISTS error for key ${key}:`, error);
      return false;
    }
  }
}

// Export singleton instance
export default new CacheService();
