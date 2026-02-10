import { Router, Request, Response } from 'express';
import { GoogleGenerativeAI } from '@google/generative-ai';
import { db } from '../db/connection';
import { userProfile } from '../db/schema'; 
import { eq } from 'drizzle-orm';
import * as dotenv from 'dotenv';

// ✅ Custom Type Extension for Request
interface AuthenticatedRequest extends Request {
  user?: {
    id: string;
    email: string;
  };
}

dotenv.config();

const router = Router();

// Initialize Gemini
const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY || '');
const model = genAI.getGenerativeModel({ model: process.env.GEMINI_MODEL || 'gemini-1.5-flash' });

// Mock Qdrant
async function searchQdrant(query: string) {
  return [
    {
      section_title: 'ROS2 Basics',
      deep_link_url: '/docs/intro',
      content: 'ROS2 provides middleware for robot communication.',
      score: 0.9
    }
  ];
}

// ✅ Use 'any' or 'AuthenticatedRequest' to stop req.user errors
router.post('/', async (req: any, res: Response): Promise<void> => {
  try {
    const { query_text } = req.body;
    
    // Auth token check (from middleware)
    const userId = req.user?.id;

    if (!query_text) {
      res.status(400).json({ error: 'query_text is required' });
      return;
    }

    let skillLevel = 'Beginner';

    // ✅ DB Query Fix
    if (userId) {
      try {
        const [profile] = await db
          .select()
          .from(userProfile)
          // Ensure your schema.ts uses 'userId' or 'user_id' exactly like this:
          .where(eq(userProfile.user_id, userId)) 
          .limit(1);

        if (profile) {
          skillLevel = profile.software_background || 'Beginner';
        }
      } catch (dbError) {
        console.error('[DB Error] Could not fetch profile:', dbError);
        // Don't crash, just fallback to Beginner
      }
    }

    const searchResults = await searchQdrant(query_text);
    const context = searchResults.map(r => r.content).join('\n');

    const prompt = `You are a Physical AI Tutor. User Skill: ${skillLevel}. Context: ${context}. Question: ${query_text}`;

    const result = await model.generateContent(prompt);
    const answer = result.response.text();

    res.json({
      answer,
      citations: searchResults.map(r => ({ title: r.section_title, url: r.deep_link_url })),
      metadata: { skillLevel }
    });

  } catch (error: any) {
    console.error('[Chat API Error]:', error);
    res.status(500).json({ error: 'Server Error' });
  }
});

export default router;