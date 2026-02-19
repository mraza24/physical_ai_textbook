"use strict";
/**
 * HACKATHON DEMO MODE - Static Mock Responses
 *
 * NO ANTHROPIC API KEY REQUIRED
 *
 * This module returns pre-written responses for:
 * - Urdu translations (from static-responses.ts)
 * - Content personalization (Bulldog AI tips)
 * - Chapter validation (mock responses)
 *
 * Original LLM integration code preserved below for reference
 */
Object.defineProperty(exports, "__esModule", { value: true });
exports.personalizeContent = personalizeContent;
exports.translateToUrdu = translateToUrdu;
exports.validateChapter = validateChapter;
exports.testLLMConnection = testLLMConnection;
const static_responses_1 = require("./static-responses");
/**
 * Personalize chapter content using static Bulldog AI tips
 *
 * NO API CALLS - Returns pre-written personalization based on user profile
 *
 * @param chapterContent - Original markdown content
 * @param softwareBackground - Beginner | Intermediate | Expert
 * @param hardwareExperience - None | Basic | Advanced
 * @returns Personalized markdown content with Bulldog AI tips
 */
async function personalizeContent(chapterContent, softwareBackground, hardwareExperience) {
    console.log(`[Static LLM] Personalizing for: ${softwareBackground} software, ${hardwareExperience} hardware`);
    // Simulate API delay (50-150ms for realism)
    await sleep(50 + Math.random() * 100);
    // Get static personalized content
    const personalized = (0, static_responses_1.getPersonalizedContent)(chapterContent, softwareBackground, hardwareExperience);
    console.log(`[Static LLM] Personalization complete (${personalized.length} characters)`);
    return personalized;
}
/**
 * Translate chapter content to Urdu using pre-written translations
 *
 * NO API CALLS - Returns complete Urdu translations from static-responses.ts
 *
 * @param chapterContent - Original English markdown content (not used in static mode)
 * @param technicalTerms - List of terms to preserve (not used in static mode)
 * @param chapterPath - Path to chapter (e.g., "/docs/module1/intro")
 * @returns Complete Urdu translation or original content if no translation exists
 */
async function translateToUrdu(chapterContent, technicalTerms = [], chapterPath) {
    console.log(`[Static LLM] Translating to Urdu: ${chapterPath || 'unknown chapter'}`);
    // Simulate API delay (100-200ms for realism)
    await sleep(100 + Math.random() * 100);
    // Get static Urdu translation if available
    if (chapterPath) {
        const urduContent = (0, static_responses_1.getUrduTranslation)(chapterPath);
        if (urduContent) {
            console.log(`[Static LLM] Urdu translation found (${urduContent.length} characters)`);
            return urduContent;
        }
    }
    // Fallback: No translation available
    console.log(`[Static LLM] No Urdu translation for ${chapterPath}, returning original`);
    return chapterContent;
}
/**
 * Validate chapter technical accuracy (mock response)
 *
 * Returns static validation feedback
 *
 * @param chapterContent - Chapter markdown content
 * @returns Validation feedback as JSON
 */
async function validateChapter(chapterContent) {
    console.log(`[Static LLM] Validating chapter (${chapterContent.length} characters)`);
    // Simulate API delay
    await sleep(200 + Math.random() * 300);
    // Return static validation feedback
    return {
        overall_score: 92,
        issues: [
            {
                severity: 'low',
                issue: 'Consider adding more real-world examples',
                suggestion: 'Include industry use cases or research projects'
            }
        ],
        strengths: [
            'Clear structure with progressive difficulty',
            'Comprehensive code examples',
            'Good balance of theory and practice'
        ]
    };
}
/**
 * Test LLM connectivity (always returns true in static mode)
 *
 * @returns Always true
 */
async function testLLMConnection() {
    console.log('[Static LLM] Connection test: STATIC MODE (no API calls)');
    return true;
}
/**
 * Sleep utility for simulating API delays
 */
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}
// ========================================
// ORIGINAL API CODE (PRESERVED FOR REFERENCE)
// ========================================
/*
import Anthropic from '@anthropic-ai/sdk';
import * as dotenv from 'dotenv';

dotenv.config();

if (!process.env.ANTHROPIC_API_KEY) {
  console.warn(
    '⚠️  WARNING: ANTHROPIC_API_KEY not set. LLM features will return mock data.' +
    '\n   Get your API key from: https://console.anthropic.com'
  );
}

const anthropic = process.env.ANTHROPIC_API_KEY
  ? new Anthropic({ apiKey: process.env.ANTHROPIC_API_KEY })
  : null;

const DEFAULT_MODEL = process.env.CLAUDE_MODEL || 'claude-sonnet-4-5-20250929';
const MAX_RETRIES = 3;
const RETRY_DELAY_MS = 1000;

function sleep(ms: number): Promise<void> {
  return new Promise(resolve => setTimeout(resolve, ms));
}

export async function callClaude(
  messages: Array<{ role: 'user' | 'assistant'; content: string }>,
  systemPrompt?: string,
  maxTokens: number = 4096,
  temperature: number = 0.7
): Promise<string> {
  if (!anthropic) {
    console.warn('[LLM Client] No API key - returning mock response');
    return `[MOCK RESPONSE] This content would be transformed based on: ${systemPrompt?.substring(0, 50)}...`;
  }

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

      const textContent = response.content.find(block => block.type === 'text');
      if (!textContent || textContent.type !== 'text') {
        throw new Error('No text content in Claude response');
      }

      return textContent.text;

    } catch (error) {
      lastError = error as Error;

      const isRetryable =
        error instanceof Anthropic.APIError &&
        (error.status === 429 ||
         error.status === 500 ||
         error.status === 503);

      if (!isRetryable || attempt === MAX_RETRIES - 1) {
        throw error;
      }

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
*/
