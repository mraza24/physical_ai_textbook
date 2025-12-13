import express, { Request, Response } from 'express';
import cors from 'cors';
import { errorHandler, notFoundHandler } from './middleware/error.middleware';
import { checkDatabaseConnection } from './config/database';
import { checkRedisConnection } from './config/redis';
import { checkClaudeConnection } from './config/claude';
import authRoutes from './routes/auth.routes';
import personalizeRoutes from './routes/personalize.routes';
import translateRoutes from './routes/translate.routes';

const app = express();
const PORT = process.env.PORT || 4000;

// Middleware
app.use(cors({
  origin: process.env.DOCUSAURUS_ORIGIN || 'http://localhost:3000',
  credentials: true
}));
app.use(express.json({ limit: '10mb' })); // Increased limit for chapter content
app.use(express.urlencoded({ extended: true, limit: '10mb' }));

// Health check endpoint with service status
app.get('/health', async (req: Request, res: Response) => {
  const services = {
    database: await checkDatabaseConnection().catch(() => false),
    redis: await checkRedisConnection().catch(() => false),
    claude: await checkClaudeConnection().catch(() => false),
  };

  const allHealthy = Object.values(services).every(status => status);

  res.status(allHealthy ? 200 : 503).json({
    status: allHealthy ? 'healthy' : 'degraded',
    timestamp: new Date().toISOString(),
    service: 'physical-ai-textbook-api',
    version: '1.0.0',
    services,
  });
});

// Root endpoint
app.get('/', (req: Request, res: Response) => {
  res.status(200).json({
    message: 'Physical AI Textbook API',
    endpoints: {
      health: '/health',
      auth: '/api/auth/*',
      personalize: '/api/personalize',
      translate: '/api/translate'
    }
  });
});

// Register route handlers
app.use('/api/auth', authRoutes);
app.use('/api/personalize', personalizeRoutes);
app.use('/api/translate', translateRoutes);

// 404 handler (must be after all routes)
app.use(notFoundHandler);

// Error handler (must be last)
app.use(errorHandler);

// Start server
app.listen(PORT, () => {
  console.log(`🚀 Server running on http://localhost:${PORT}`);
  console.log(`📊 Health check: http://localhost:${PORT}/health`);
  console.log(`🌍 Environment: ${process.env.NODE_ENV || 'development'}`);
});

export default app;
