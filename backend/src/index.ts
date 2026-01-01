import express, { Request, Response, NextFunction } from 'express';
import cors from 'cors';
import * as dotenv from 'dotenv';
import { authHandler } from './auth/config';
import { healthCheck } from './db/connection';
import { requireAuth } from './auth/middleware';
import authRoutes from './auth/routes';
import personalizeRoutes from './routes/personalize';
import translateRoutes from './routes/translate';
import { createTransformationRateLimiter } from './services/rate-limiter';

// Load environment variables
dotenv.config();

/**
 * Authentication & Personalization Backend Server
 *
 * Provides:
 * - Better-Auth integration for user authentication
 * - User profile management with personalization fields
 * - AI content transformation endpoints (personalize, translate)
 * - Transformation caching with 5-min TTL
 * - Health check endpoint for monitoring
 *
 * Port: 4000 (configurable via PORT env var)
 * Frontend: Docusaurus on port 3000
 */

const app = express();
const PORT = process.env.PORT || 4000;

// ============================================================================
// Middleware Configuration
// ============================================================================

/**
 * CORS Configuration
 * Allows requests from Docusaurus frontend (localhost:3000)
 */
const corsOrigins = process.env.CORS_ORIGINS?.split(',') || ['http://localhost:3000'];
app.use(cors({
  origin: corsOrigins,
  credentials: true, // Allow cookies for session management
  methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
  allowedHeaders: ['Content-Type', 'Authorization'],
}));

/**
 * JSON body parser
 */
app.use(express.json({ limit: '10mb' })); // Support large markdown content

/**
 * URL-encoded body parser
 */
app.use(express.urlencoded({ extended: true }));

/**
 * Request logging middleware (development only)
 */
if (process.env.NODE_ENV === 'development') {
  app.use((req: Request, res: Response, next: NextFunction) => {
    const timestamp = new Date().toISOString();
    console.log(`[${timestamp}] ${req.method} ${req.path}`);
    next();
  });
}

// ============================================================================
// Routes
// ============================================================================

/**
 * Health Check Endpoint
 *
 * Tests database connectivity and returns server status
 *
 * Response:
 * - 200 OK: Database connected, server healthy
 * - 503 Service Unavailable: Database connection failed
 */
app.get('/health', async (req: Request, res: Response) => {
  try {
    const dbTime = await healthCheck();
    res.status(200).json({
      status: 'OK',
      timestamp: new Date().toISOString(),
      database: {
        connected: true,
        serverTime: dbTime.toISOString(),
      },
      environment: process.env.NODE_ENV || 'development',
    });
  } catch (error) {
    console.error('Health check failed:', error);
    res.status(503).json({
      status: 'ERROR',
      timestamp: new Date().toISOString(),
      database: {
        connected: false,
        error: error instanceof Error ? error.message : 'Unknown error',
      },
      environment: process.env.NODE_ENV || 'development',
    });
  }
});

/**
 * Better-Auth Base Handler
 *
 * Handles core Better-Auth endpoints:
 * - POST /api/auth/signin
 * - POST /api/auth/signout
 * - GET /api/auth/session
 * - POST /api/auth/refresh
 *
 * Note: toNodeHandler wraps Better-Auth for Node.js/Express compatibility
 */
app.use('/api/auth', authHandler);

/**
 * Custom Auth Routes
 *
 * Custom signup and profile management:
 * - POST /api/auth/signup (custom with profile creation)
 * - GET /api/auth/profile (requires authentication)
 * - PUT /api/auth/profile (requires authentication)
 */
app.use('/api/auth', authRoutes);

/**
 * Personalization Routes (T050-T056)
 *
 * AI-powered content personalization:
 * - POST /api/personalize (requires authentication + rate limiting)
 * - GET /api/personalize/status/:chapterPath (requires authentication)
 *
 * PROTECTED: Requires JWT authentication via requireAuth middleware
 * RATE LIMITED: 5 requests per minute per user (transformation rate limiter)
 */
app.use(
  '/api/personalize',
  requireAuth,
  createTransformationRateLimiter(5, 1), // 5 req/min for AI transformations
  personalizeRoutes
);

/**
 * Translation Routes (T036-T040 equivalent)
 *
 * Technical content translation with term preservation:
 * - POST /api/translate/urdu (requires authentication + rate limiting)
 * - GET /api/translate/urdu/status/:chapterPath (requires authentication)
 * - GET /api/translate/terms (requires authentication)
 *
 * PROTECTED: Requires JWT authentication via requireAuth middleware
 * RATE LIMITED: 5 requests per minute per user (transformation rate limiter)
 */
app.use(
  '/api/translate',
  requireAuth,
  createTransformationRateLimiter(5, 1), // 5 req/min for AI transformations
  translateRoutes
);

/**
 * API Status Endpoint
 *
 * Returns API version and available endpoints
 */
app.get('/api', (req: Request, res: Response) => {
  res.json({
    name: 'Authentication & Personalization API',
    version: '1.0.0',
    endpoints: {
      health: '/health',
      auth: '/api/auth/*',
      personalize: '/api/personalize (POST - requires auth)',
      translate: '/api/translate/urdu (POST - requires auth)',
      terms: '/api/translate/terms (GET - requires auth)',
    },
  });
});

/**
 * Root Endpoint
 *
 * Simple welcome message
 */
app.get('/', (req: Request, res: Response) => {
  res.json({
    message: 'Authentication & Personalization Backend',
    documentation: '/api',
    health: '/health',
  });
});

/**
 * 404 Handler
 *
 * Returns error for undefined routes
 */
app.use((req: Request, res: Response) => {
  res.status(404).json({
    error: 'Not Found',
    message: `Route ${req.method} ${req.path} not found`,
    availableRoutes: ['/', '/health', '/api', '/api/auth/*'],
  });
});

/**
 * Global Error Handler
 *
 * Catches and formats all unhandled errors
 */
app.use((err: Error, req: Request, res: Response, next: NextFunction) => {
  console.error('Unhandled error:', err);
  res.status(500).json({
    error: 'Internal Server Error',
    message: process.env.NODE_ENV === 'development' ? err.message : 'An unexpected error occurred',
    ...(process.env.NODE_ENV === 'development' && { stack: err.stack }),
  });
});

// ============================================================================
// Server Startup
// ============================================================================

/**
 * Start the Express server
 */
const server = app.listen(PORT, () => {
  console.log('='.repeat(70));
  console.log('🚀 Authentication & Personalization Backend');
  console.log('='.repeat(70));
  console.log(`📍 Server running on: http://localhost:${PORT}`);
  console.log(`🌐 CORS enabled for: ${corsOrigins.join(', ')}`);
  console.log(`🔐 Auth endpoints: http://localhost:${PORT}/api/auth/*`);
  console.log(`❤️  Health check: http://localhost:${PORT}/health`);
  console.log(`📊 Environment: ${process.env.NODE_ENV || 'development'}`);
  console.log('='.repeat(70));
});

/**
 * Graceful Shutdown Handler
 *
 * Closes server connections cleanly on SIGTERM/SIGINT
 */
const gracefulShutdown = (signal: string) => {
  console.log(`\n${signal} received. Shutting down gracefully...`);
  server.close(() => {
    console.log('✅ Server closed');
    process.exit(0);
  });

  // Force shutdown after 10 seconds
  setTimeout(() => {
    console.error('⚠️  Forced shutdown after timeout');
    process.exit(1);
  }, 10000);
};

process.on('SIGTERM', () => gracefulShutdown('SIGTERM'));
process.on('SIGINT', () => gracefulShutdown('SIGINT'));

export default app;
