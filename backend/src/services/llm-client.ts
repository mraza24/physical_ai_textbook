import Anthropic from '@anthropic-ai/sdk';
import * as dotenv from 'dotenv';

dotenv.config();

/**
 * Anthropic Claude LLM Client
 *
 * Wrapper around Anthropic SDK with:
 * - Automatic retry logic for transient errors
 * - Environment-based configuration
 * - Integration with content-personalizer and urdu-translator skills
 * - Error handling and logging
 *
 * Model: claude-sonnet-4-5-20250929 (configured via env)
 */

// Validate API key
if (!process.env.ANTHROPIC_API_KEY) {
  throw new Error(
    'ANTHROPIC_API_KEY environment variable is required. ' +
    'Get your API key from: https://console.anthropic.com'
  );
}

// Initialize Anthropic client
const anthropic = new Anthropic({
  apiKey: process.env.ANTHROPIC_API_KEY,
});

const DEFAULT_MODEL = process.env.CLAUDE_MODEL || 'claude-sonnet-4-5-20250929';
const MAX_RETRIES = 3;
const RETRY_DELAY_MS = 1000; // Start with 1 second, exponential backoff

/**
 * Sleep utility for retry delays
 */
function sleep(ms: number): Promise<void> {
  return new Promise(resolve => setTimeout(resolve, ms));
}

/**
 * Call Claude API with retry logic
 *
 * Handles transient errors (rate limits, network issues) with exponential backoff
 *
 * @param messages - Conversation messages
 * @param systemPrompt - System instructions (optional)
 * @param maxTokens - Maximum response tokens (default: 4096)
 * @param temperature - Sampling temperature (default: 0.7)
 * @returns Claude's response text
 */
export async function callClaude(
  messages: Array<{ role: 'user' | 'assistant'; content: string }>,
  systemPrompt?: string,
  maxTokens: number = 4096,
  temperature: number = 0.7
): Promise<string> {
  let lastError: Error | null = null;

  for (let attempt = 0; attempt < MAX_RETRIES; attempt++) {
    try {
      const response = await anthropic.messages.create({
        model: DEFAULT_MODEL,
        max_tokens: maxTokens,
        temperature,
        system: systemPrompt,
        messages,
      });

      // Extract text from response
      const textContent = response.content.find(block => block.type === 'text');
      if (!textContent || textContent.type !== 'text') {
        throw new Error('No text content in Claude response');
      }

      return textContent.text;

    } catch (error) {
      lastError = error as Error;

      // Check if error is retryable
      const isRetryable =
        error instanceof Anthropic.APIError &&
        (error.status === 429 || // Rate limit
         error.status === 500 || // Server error
         error.status === 503);  // Service unavailable

      if (!isRetryable || attempt === MAX_RETRIES - 1) {
        // Non-retryable error or final attempt
        throw error;
      }

      // Exponential backoff
      const delayMs = RETRY_DELAY_MS * Math.pow(2, attempt);
      console.warn(
        `Claude API error (attempt ${attempt + 1}/${MAX_RETRIES}): ${error.message}. ` +
        `Retrying in ${delayMs}ms...`
      );
      await sleep(delayMs);
    }
  }

  throw lastError || new Error('Failed to call Claude API after retries');
}

/**
 * Personalize chapter content using content-personalizer skill
 *
 * Adapts chapter complexity based on user's software background
 *
 * @param chapterContent - Original markdown content
 * @param softwareBackground - Beginner | Intermediate | Expert
 * @param hardwareExperience - None | Basic | Advanced
 * @returns Personalized markdown content
 */
export async function personalizeContent(
  chapterContent: string,
  softwareBackground: string,
  hardwareExperience: string
): Promise<string> {
  const systemPrompt = `You are a technical content personalizer for a Physical AI textbook.

Your task is to adapt chapter content for the reader's background:
- Software Background: ${softwareBackground}
- Hardware Experience: ${hardwareExperience}

PERSONALIZATION RULES:
1. For "Beginner" software background:
   - Add more explanations for programming concepts
   - Include code comments explaining syntax
   - Add "Prerequisites" sections with links to fundamentals

2. For "Expert" software background:
   - Remove basic explanations
   - Add advanced optimization notes
   - Include performance considerations

3. For "None" hardware experience:
   - Explain physical concepts thoroughly
   - Add diagrams and visual aids
   - Include safety warnings

4. For "Advanced" hardware experience:
   - Focus on technical specifications
   - Add circuit diagrams and datasheets
   - Include troubleshooting tips

PRESERVATION REQUIREMENTS:
- Keep all Docusaurus-specific syntax (admonitions, imports, tabs)
- Preserve code blocks exactly as-is (only add comments if needed)
- Maintain all links and references
- Keep section structure intact`;

  const userMessage = `Personalize this chapter content:\n\n${chapterContent}`;

  return await callClaude(
    [{ role: 'user', content: userMessage }],
    systemPrompt,
    8192, // Longer responses for full chapters
    0.5   // Lower temperature for consistency
  );
}

/**
 * Translate chapter content to Urdu using urdu-translator skill
 *
 * Preserves technical terms and Docusaurus syntax
 *
 * @param chapterContent - Original English markdown content
 * @param technicalTerms - List of terms to preserve (optional)
 * @returns Urdu translation with preserved technical terms
 */
export async function translateToUrdu(
  chapterContent: string,
  technicalTerms: string[] = []
): Promise<string> {
  const termsList = technicalTerms.length > 0
    ? technicalTerms.join(', ')
    : 'ROS2, sensor, actuator, SLAM, Gazebo, Docker, Python, neural network, IoT, etc.';

  const systemPrompt = `You are a technical translator specializing in Physical AI and robotics content.

Your task is to translate English technical content to Urdu while preserving:
1. ALL technical terms in English (${termsList})
2. ALL code blocks exactly as-is
3. ALL Docusaurus syntax (:::tip, :::warning, import statements, etc.)
4. ALL links and file paths

TRANSLATION GUIDELINES:
- Use proper Urdu typography and right-to-left formatting
- Maintain technical accuracy
- Keep sentences clear and concise
- Use standard Urdu terminology for common concepts
- Preserve markdown formatting (headings, lists, tables)

CRITICAL PRESERVATION:
- Code blocks: Keep 100% unchanged (including comments in English)
- Technical terms: Keep in English, may add Urdu explanation in parentheses
- Docusaurus admonitions: Keep syntax, translate content only
- Import statements: Keep completely unchanged`;

  const userMessage = `Translate this chapter to Urdu:\n\n${chapterContent}`;

  return await callClaude(
    [{ role: 'user', content: userMessage }],
    systemPrompt,
    8192, // Longer responses for full chapters
    0.3   // Lower temperature for accurate translation
  );
}

/**
 * Validate chapter technical accuracy (research-validator skill)
 *
 * Returns structured feedback on technical content
 *
 * @param chapterContent - Chapter markdown content
 * @returns Validation feedback as JSON
 */
export async function validateChapter(
  chapterContent: string
): Promise<{
  overall_score: number;
  issues: Array<{ severity: string; issue: string; suggestion: string }>;
  strengths: string[];
}> {
  const systemPrompt = `You are a technical reviewer for a Physical AI textbook.

Your task is to validate technical accuracy and identify:
1. Factual errors or outdated information
2. Missing prerequisites or dependencies
3. Unclear explanations
4. Code issues (syntax errors, deprecated APIs)
5. Safety concerns (hardware/software)

Return feedback as JSON with this structure:
{
  "overall_score": 85,
  "issues": [
    {
      "severity": "high | medium | low",
      "issue": "Description of the problem",
      "suggestion": "How to fix it"
    }
  ],
  "strengths": ["What the chapter does well"]
}`;

  const userMessage = `Validate this chapter content:\n\n${chapterContent}`;

  const response = await callClaude(
    [{ role: 'user', content: userMessage }],
    systemPrompt,
    4096,
    0.2 // Very low temperature for consistent JSON
  );

  // Parse JSON response
  try {
    return JSON.parse(response);
  } catch (error) {
    console.error('Failed to parse validation response:', error);
    return {
      overall_score: 0,
      issues: [{ severity: 'high', issue: 'Validation failed', suggestion: 'Retry validation' }],
      strengths: [],
    };
  }
}

/**
 * Test LLM connectivity
 *
 * Simple health check to verify API key and model access
 *
 * @returns True if successful, throws error otherwise
 */
export async function testLLMConnection(): Promise<boolean> {
  try {
    const response = await callClaude(
      [{ role: 'user', content: 'Respond with "OK"' }],
      'You are a helpful assistant. Respond concisely.',
      50,
      0
    );

    return response.toLowerCase().includes('ok');
  } catch (error) {
    console.error('LLM connection test failed:', error);
    throw error;
  }
}
