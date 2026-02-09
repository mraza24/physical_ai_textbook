import express, { Request, Response, NextFunction } from 'express';
import cors from 'cors';
import * as dotenv from 'dotenv';
import { authHandler } from './auth/config';
import { healthCheck } from './db/connection';
import { requireAuth } from './auth/middleware';
import authRoutes from './auth/routes';
import chatRoutes from './routes/chat';
import personalizeRoutes from './routes/personalize';
import translateRoutes from './routes/translate'; 

// Load environment variables
dotenv.config();

const app = express();
const PORT = process.env.PORT || 4000;

// CORS Configuration
const corsOrigins = process.env.CORS_ORIGINS?.split(',') || ['http://localhost:3000'];
app.use(cors({
  origin: corsOrigins,
  credentials: true, 
  methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
  allowedHeaders: ['Content-Type', 'Authorization'],
}));

app.use(express.json({ limit: '10mb' }));

// Health Check Route
app.get('/health', async (req: Request, res: Response) => {
  try {
    const dbTime = await healthCheck();
    res.status(200).json({
      status: 'OK',
      database: { connected: true, serverTime: dbTime.toISOString() },
    });
  } catch (error) {
    res.status(503).json({ status: 'ERROR', database: { connected: false } });
  }
});

// Routes Registration
// IMPORTANT: Order matters! Custom routes MUST come before Better Auth handler
// Custom auth routes (signup with profile creation)
app.use('/api/auth', authRoutes);
// Better Auth handler - Handles built-in routes like /sign-in, /session, etc.
// This MUST come after custom routes so our /signup isn't shadowed
app.use('/api/auth', (req: Request, res: Response) => authHandler(req, res));
app.use('/api/chat', chatRoutes);
app.use('/api/personalize', personalizeRoutes); // POST /api/personalize
app.use('/api/translate', translateRoutes);     // POST /api/translate/urdu 

app.get('/', (req: Request, res: Response) => {
  res.json({ message: 'Physical AI Backend Running' });
});

// Server Startup Function
async function startServer() {
  // Track database connection status
  let dbConnected = false;

  // Try to connect to database, but don't crash if it fails
  try {
    console.log('🔍 Testing database connection...');
    await healthCheck(5000, 2); // 5s timeout, 2 retries
    console.log('✅ Database connected successfully\n');
    dbConnected = true;
  } catch (error) {
    console.error('⚠️  Database connection failed during startup:');
    console.error(error instanceof Error ? error.message : String(error));
    console.warn('\n⚠️  WARNING: Server starting WITHOUT database connection');
    console.warn('   - Authentication endpoints will not work');
    console.warn('   - Chat endpoints will not work');
    console.warn('   - Health check endpoint will report database as unavailable');
    console.warn('   - Fix DATABASE_URL in .env and restart server\n');
  }

  // Start Express server regardless of database status
  const server = app.listen(PORT, () => {
    console.log('='.repeat(70));
    console.log(`🚀 UNIFIED BACKEND LIVE ON PORT: ${PORT}`);
    console.log(`🌐 CORS ENABLED FOR: ${corsOrigins.join(', ')}`);
    console.log(`📊 Health Check: http://localhost:${PORT}/health`);
    console.log(`💾 Database Status: ${dbConnected ? '✅ Connected' : '❌ Not Connected'}`);
    console.log('='.repeat(70));

    if (!dbConnected) {
      console.log('\n📝 Database Troubleshooting:');
      console.log('   1. Check your .env file has DATABASE_URL set');
      console.log('   2. Verify your Neon database is not paused at https://console.neon.tech');
      console.log('   3. Test connection: curl http://localhost:4000/health');
      console.log('   4. Check DATABASE_URL format: postgresql://user:pass@host/dbname');
      console.log('');
    }
  });

  // Stability Fix: Keep the process alive
  setInterval(() => {}, 1000 * 60 * 60);

  // Graceful Shutdown
  const gracefulShutdown = (signal: string) => {
    console.log(`\n${signal} received. Shutting down gracefully...`);
    server.close(() => {
      console.log('Server closed');
      process.exit(0);
    });
  };

  process.on('SIGTERM', () => gracefulShutdown('SIGTERM'));
  process.on('SIGINT', () => gracefulShutdown('SIGINT'));
}

startServer();

export default app;