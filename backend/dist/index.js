"use strict";
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || (function () {
    var ownKeys = function(o) {
        ownKeys = Object.getOwnPropertyNames || function (o) {
            var ar = [];
            for (var k in o) if (Object.prototype.hasOwnProperty.call(o, k)) ar[ar.length] = k;
            return ar;
        };
        return ownKeys(o);
    };
    return function (mod) {
        if (mod && mod.__esModule) return mod;
        var result = {};
        if (mod != null) for (var k = ownKeys(mod), i = 0; i < k.length; i++) if (k[i] !== "default") __createBinding(result, mod, k[i]);
        __setModuleDefault(result, mod);
        return result;
    };
})();
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
const express_1 = __importDefault(require("express"));
const cors_1 = __importDefault(require("cors"));
const dotenv = __importStar(require("dotenv"));
const config_1 = require("./auth/config");
const connection_1 = require("./db/connection");
const routes_1 = __importDefault(require("./auth/routes"));
const chat_1 = __importDefault(require("./routes/chat"));
const personalize_1 = __importDefault(require("./routes/personalize"));
const translate_1 = __importDefault(require("./routes/translate"));
// Load environment variables
dotenv.config();
const app = (0, express_1.default)();
const PORT = process.env.PORT || 4000;
// CORS Configuration
const corsOrigins = process.env.CORS_ORIGINS?.split(',') || ['http://localhost:3000'];
app.use((0, cors_1.default)({
    origin: corsOrigins,
    credentials: true,
    methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
    allowedHeaders: ['Content-Type', 'Authorization'],
}));
app.use(express_1.default.json({ limit: '10mb' }));
// Health Check Route
app.get('/health', async (req, res) => {
    try {
        const dbTime = await (0, connection_1.healthCheck)();
        res.status(200).json({
            status: 'OK',
            database: { connected: true, serverTime: dbTime.toISOString() },
        });
    }
    catch (error) {
        res.status(503).json({ status: 'ERROR', database: { connected: false } });
    }
});
// Routes Registration
// IMPORTANT: Order matters! Custom routes MUST come before Better Auth handler
// Custom auth routes (signup with profile creation)
app.use('/api/auth', routes_1.default);
// Better Auth handler - Handles built-in routes like /sign-in, /session, etc.
// This MUST come after custom routes so our /signup isn't shadowed
app.use('/api/auth', (req, res) => (0, config_1.authHandler)(req, res));
app.use('/api/chat', chat_1.default);
app.use('/api/personalize', personalize_1.default); // POST /api/personalize
app.use('/api/translate', translate_1.default); // POST /api/translate/urdu 
app.get('/', (req, res) => {
    res.json({ message: 'Physical AI Backend Running' });
});
// Server Startup Function
async function startServer() {
    // Track database connection status
    let dbConnected = false;
    // Try to connect to database, but don't crash if it fails
    try {
        console.log('🔍 Testing database connection...');
        await (0, connection_1.healthCheck)(5000, 2); // 5s timeout, 2 retries
        console.log('✅ Database connected successfully\n');
        dbConnected = true;
    }
    catch (error) {
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
    setInterval(() => { }, 1000 * 60 * 60);
    // Graceful Shutdown
    const gracefulShutdown = (signal) => {
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
exports.default = app;
