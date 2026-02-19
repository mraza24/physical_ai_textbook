"use strict";
/**
 * RAG Chat Route with COHERE API and JWT-based Personalization
 * * File: backend/src/routes/chat.ts
 */
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
Object.defineProperty(exports, "__esModule", { value: true });
const express_1 = require("express");
const cohere_ai_1 = require("cohere-ai"); // ✅ Cohere Integrated
const config_1 = require("../auth/config");
const connection_1 = require("../db/connection");
const schema_1 = require("../db/schema");
const drizzle_orm_1 = require("drizzle-orm");
const dotenv = __importStar(require("dotenv"));
dotenv.config();
const router = (0, express_1.Router)();
// ✅ Initialize Cohere Client
const cohere = new cohere_ai_1.CohereClient({
    token: process.env.COHERE_API_KEY || '',
});
async function searchQdrant(query) {
    console.log('[Qdrant Mock] Searching for:', query);
    return [
        {
            section_title: 'Introduction to ROS2',
            deep_link_url: '/docs/module1/intro',
            content: 'ROS2 is the second generation of the Robot Operating System. It features middleware abstraction and hardware control.',
            score: 0.95
        },
        {
            section_title: 'Sensor Integration',
            deep_link_url: '/docs/module2/sensors',
            content: 'Physical AI relies on sensor fusion combining LIDAR, cameras, and IMUs.',
            score: 0.87
        }
    ];
}
router.post('/', async (req, res) => {
    try {
        const { query_text } = req.body;
        if (!query_text) {
            res.status(400).json({ error: 'query_text is required' });
            return;
        }
        // Step 1: User Profile & Personalization logic
        let skillLevel = 'Beginner';
        let userName = 'User';
        let authenticated = false;
        const authHeader = req.headers.authorization;
        if (authHeader && authHeader.startsWith('Bearer ')) {
            try {
                const session = await config_1.auth.api.getSession({
                    headers: req.headers,
                });
                if (session?.user) {
                    authenticated = true;
                    // ✅ FIX: Using user_id (Snake Case) to match Drizzle/Postgres schema
                    const [profile] = await connection_1.db
                        .select()
                        .from(schema_1.userProfile)
                        .where((0, drizzle_orm_1.eq)(schema_1.userProfile.user_id, session.user.id))
                        .limit(1);
                    if (profile) {
                        // ✅ FIX: Using software_background to match DB schema
                        skillLevel = profile.software_background || 'Beginner';
                        userName = session.user.email.split('@')[0];
                    }
                }
            }
            catch (authError) {
                console.log('[Chat API] Auth failed, using defaults');
            }
        }
        // Step 2: Search Context
        const searchResults = await searchQdrant(query_text);
        const contextChunks = searchResults.map(r => r.content).join('\n\n');
        const citations = searchResults.map(r => ({
            section_title: r.section_title,
            deep_link_url: r.deep_link_url
        }));
        // Step 3: Build Personalized System Prompt for Cohere
        const systemPrompt = `You are a Physical AI Tutor.
User Skill Level: ${skillLevel}.
Tone: ${skillLevel === 'Expert' ? 'Technical and detailed' : 'Simple and using analogies'}.
Context from Textbook: ${contextChunks}`;
        // Step 4: Generate response with Cohere Chat API (Updated Jan 2026)
        console.log('[Chat API] Generating response with Cohere...');
        const response = await cohere.chat({
            model: 'command-r-08-2024', // Current model (command-r/command-r-plus removed Sept 2025)
            message: query_text,
            preamble: systemPrompt,
            maxTokens: 500,
            temperature: 0.7,
        });
        const answer = response.text;
        // Step 5: Return result
        res.json({
            answer,
            citations,
            metadata: {
                skill_level: skillLevel,
                authenticated,
                user_name: userName,
                model: 'cohere-command-r-08-2024',
                timestamp: new Date().toISOString()
            }
        });
    }
    catch (error) {
        console.error('[Chat API] Error:', error);
        res.status(500).json({ error: 'Internal Server Error', message: error.message });
    }
});
// Health check for Cohere
router.get('/health', async (req, res) => {
    res.json({ status: 'OK', model: 'Cohere Production', timestamp: new Date().toISOString() });
});
exports.default = router;
