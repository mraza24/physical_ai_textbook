import Redis from 'ioredis';

// Initialize Redis client with retry strategy
const redis = new Redis(process.env.REDIS_URL || 'redis://localhost:6379', {
  retryStrategy: (times: number) => {
    const delay = Math.min(times * 50, 2000);
    return delay;
  },
  maxRetriesPerRequest: 3,
  lazyConnect: true,
});

// Connection event handlers
redis.on('connect', () => {
  console.log('✅ Redis connection established');
});

redis.on('error', (error) => {
  console.error('❌ Redis connection error:', error.message);
});

redis.on('ready', () => {
  console.log('Redis client ready');
});

// Connection health check
export async function checkRedisConnection(): Promise<boolean> {
  try {
    await redis.connect();
    await redis.ping();
    console.log('Redis ping successful');
    return true;
  } catch (error) {
    console.error('Redis health check failed:', error);
    return false;
  }
}

// Graceful shutdown
export async function disconnectRedis(): Promise<void> {
  await redis.quit();
  console.log('Redis connection closed');
}

// Export Redis client instance
export default redis;
