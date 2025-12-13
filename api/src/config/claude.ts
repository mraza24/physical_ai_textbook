import Anthropic from '@anthropic-ai/sdk';

// Initialize Claude API client
const claude = new Anthropic({
  apiKey: process.env.CLAUDE_API_KEY || 'placeholder',
});

// Default model configuration
export const CLAUDE_MODEL = 'claude-3-5-sonnet-20241022';

// Default API parameters
export const DEFAULT_MAX_TOKENS = 4096;
export const DEFAULT_TEMPERATURE = 1.0;

// Health check function
export async function checkClaudeConnection(): Promise<boolean> {
  try {
    if (!process.env.CLAUDE_API_KEY || process.env.CLAUDE_API_KEY === 'placeholder') {
      console.warn('⚠️  Claude API key not configured');
      return false;
    }

    // Test with a minimal request
    const response = await claude.messages.create({
      model: CLAUDE_MODEL,
      max_tokens: 10,
      messages: [{ role: 'user', content: 'Hi' }],
    });

    console.log('✅ Claude API connection successful');
    return response.content.length > 0;
  } catch (error) {
    console.error('❌ Claude API connection failed:', error);
    return false;
  }
}

// Export Claude client instance
export default claude;
