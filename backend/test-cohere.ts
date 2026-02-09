/**
 * Cohere API Key Validation Test
 * Tests if Cohere API key is valid and working
 */

import { CohereClient } from 'cohere-ai';
import * as dotenv from 'dotenv';
import * as path from 'path';

// Load environment variables
dotenv.config({ path: path.resolve(__dirname, '.env') });

async function testCohereAPI() {
  console.log('╔════════════════════════════════════════════════════════════════╗');
  console.log('║              COHERE API KEY VALIDATION TEST                    ║');
  console.log('╚════════════════════════════════════════════════════════════════╝\n');

  // Step 1: Check if COHERE_API_KEY is set
  console.log('1️⃣  Checking COHERE_API_KEY environment variable...');
  if (!process.env.COHERE_API_KEY) {
    console.error('❌ ERROR: COHERE_API_KEY is not set in .env file');
    console.log('   Fix: Add COHERE_API_KEY to backend/.env file');
    console.log('   Get key from: https://dashboard.cohere.com/api-keys');
    process.exit(1);
  }

  const apiKey = process.env.COHERE_API_KEY;
  const keyPrefix = apiKey.substring(0, 8);
  const keySuffix = apiKey.substring(apiKey.length - 4);
  console.log('✅ COHERE_API_KEY is set');
  console.log(`   Key: ${keyPrefix}...${keySuffix} (${apiKey.length} chars)\n`);

  // Step 2: Initialize Cohere client
  console.log('2️⃣  Initializing Cohere client...');
  try {
    const cohere = new CohereClient({
      token: apiKey,
    });
    console.log('✅ Cohere client initialized\n');

    // Step 3: Test chat endpoint (updated API - Sept 2025)
    console.log('3️⃣  Testing Cohere chat endpoint (replaces deprecated generate)...');
    console.log('   Sending test message: "What is ROS2?"');

    const startTime = Date.now();
    const response = await cohere.chat({
      model: 'command-r-08-2024', // Current model (command-r/command-r-plus removed Sept 2025)
      message: 'What is ROS2? Answer in one sentence.',
      maxTokens: 50,
      temperature: 0.3,
    });
    const duration = Date.now() - startTime;

    console.log('✅ Chat endpoint successful');
    console.log(`   Response time: ${duration}ms`);
    console.log(`   Model: command-r-08-2024`);
    console.log(`   Generation ID: ${response.generationId || 'N/A'}`);
    console.log('');

    if (response.text) {
      console.log('   💬 Chat response:');
      console.log(`   "${response.text.trim()}"`);
      console.log('');
    }

    // Step 4: Check API limits/metadata
    console.log('4️⃣  Checking API response metadata...');
    if (response.meta) {
      console.log('   Metadata:');
      if (response.meta.billedUnits) {
        console.log(`   - Billed units: ${JSON.stringify(response.meta.billedUnits)}`);
      }
      if (response.meta.warnings) {
        console.log(`   - Warnings: ${JSON.stringify(response.meta.warnings)}`);
      }
    }
    console.log('');

    // Step 5: Test chat endpoint (used in production)
    console.log('5️⃣  Testing Cohere chat endpoint (production path)...');
    console.log('   Sending test chat message...');

    const chatStartTime = Date.now();
    const chatResponse = await cohere.chat({
      model: 'command-r-08-2024', // Current model (command-r/command-r-plus removed Sept 2025)
      message: 'Explain ROS2 in simple terms for a beginner.',
    });
    const chatDuration = Date.now() - chatStartTime;

    console.log('✅ Chat endpoint successful');
    console.log(`   Response time: ${chatDuration}ms`);
    console.log(`   Conversation ID: ${chatResponse.conversationId || 'N/A'}`);
    console.log('');

    if (chatResponse.text) {
      console.log('   💬 Chat response:');
      console.log(`   "${chatResponse.text.substring(0, 150)}..."`);
      console.log('');
    }

    // Final summary
    console.log('╔════════════════════════════════════════════════════════════════╗');
    console.log('║                    TEST SUMMARY                                ║');
    console.log('╚════════════════════════════════════════════════════════════════╝');
    console.log('✅ API Key: VALID');
    console.log('✅ Chat endpoint (primary): WORKING');
    console.log('✅ Chat endpoint (production): WORKING');
    console.log(`✅ Total response time: ${duration + chatDuration}ms`);
    console.log('✅ API Key Status: ACTIVE (2026)');
    console.log('ℹ️  Note: /v1/generate deprecated Sept 2025, using /v1/chat');
    console.log('\n🎉 Cohere API test completed successfully!\n');

  } catch (error: any) {
    console.error('\n❌ COHERE API TEST FAILED:');

    if (error.status === 401 || error.statusCode === 401) {
      console.error('❌ Error: 401 Unauthorized');
      console.error('   Your API key is invalid or expired');
      console.error('   Fix:');
      console.error('   1. Go to https://dashboard.cohere.com/api-keys');
      console.error('   2. Generate a new API key');
      console.error('   3. Update COHERE_API_KEY in .env file');
    } else if (error.status === 429 || error.statusCode === 429) {
      console.error('❌ Error: 429 Rate Limit Exceeded');
      console.error('   You have hit the trial API limit');
      console.error('   Fix:');
      console.error('   1. Upgrade to paid plan at https://dashboard.cohere.com/billing');
      console.error('   2. Or wait until rate limit resets');
    } else if (error.status === 402 || error.statusCode === 402) {
      console.error('❌ Error: 402 Payment Required');
      console.error('   Trial credits exhausted or subscription needed');
      console.error('   Fix: Upgrade at https://dashboard.cohere.com/billing');
    } else if (error.message?.includes('fetch failed') || error.message?.includes('ENOTFOUND')) {
      console.error('❌ Error: Network connection failed');
      console.error('   Cannot reach Cohere API');
      console.error('   Fix:');
      console.error('   1. Check your internet connection');
      console.error('   2. Check if firewall is blocking api.cohere.ai');
      console.error('   3. Try again in a moment');
    } else {
      console.error(`❌ Error: ${error.message || String(error)}`);
      if (error.body) {
        console.error('   Response body:', JSON.stringify(error.body, null, 2));
      }
    }

    console.error('\n📝 Full error details:');
    console.error(error);
    console.error('');
    process.exit(1);
  }
}

// Run the test
testCohereAPI();
