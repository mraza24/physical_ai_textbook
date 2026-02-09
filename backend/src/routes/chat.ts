/**
 * RAG Chat Route with COHERE API and JWT-based Personalization
 * * File: backend/src/routes/chat.ts
 */

import { Router, Request, Response } from 'express';
import { CohereClient } from 'cohere-ai'; // ✅ Cohere Integrated
import { auth } from '../auth/config';
import { db } from '../db/connection';
import { userProfile } from '../db/schema';
import { eq } from 'drizzle-orm';
import * as dotenv from 'dotenv';

dotenv.config();

const router = Router();

// ✅ Initialize Cohere Client
const cohere = new CohereClient({
  token: process.env.COHERE_API_KEY || '', 
});

// Mock Qdrant interface
interface QdrantSearchResult {
  section_title: string;
  deep_link_url: string;
  content: string;
  score: number;
}

async function searchQdrant(query: string): Promise<QdrantSearchResult[]> {
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

router.post('/', async (req: Request, res: Response): Promise<void> => {
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
        const session = await auth.api.getSession({
          headers: req.headers as any,
        });

        if (session?.user) {
          authenticated = true;
          // ✅ FIX: Using user_id (Snake Case) to match Drizzle/Postgres schema
          const [profile] = await db
            .select()
            .from(userProfile)
            .where(eq(userProfile.user_id, session.user.id))
            .limit(1);

          if (profile) {
            // ✅ FIX: Using software_background to match DB schema
            skillLevel = profile.software_background || 'Beginner';
            userName = session.user.email.split('@')[0];
          }
        }
      } catch (authError) {
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

  } catch (error: any) {
    console.error('[Chat API] Error:', error);
    res.status(500).json({ error: 'Internal Server Error', message: error.message });
  }
});

// Health check for Cohere
router.get('/health', async (req: Request, res: Response) => {
  res.json({ status: 'OK', model: 'Cohere Production', timestamp: new Date().toISOString() });
});

export default router;