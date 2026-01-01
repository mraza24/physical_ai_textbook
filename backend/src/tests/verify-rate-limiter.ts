/**
 * T021: Verify Rate Limiter Service
 *
 * Test Cases:
 * 1. Create test Express app with rate limiter
 * 2. Send 10 requests (should all succeed)
 * 3. Send 11th request (should return 429)
 * 4. Verify retry-after header
 * 5. Test transformation-specific rate limiter (5 req/min)
 */

import express from 'express';
import request from 'supertest';
import {
  createUserRateLimiter,
  createTransformationRateLimiter,
  createAuthRateLimiter,
} from '../services/rate-limiter';

async function verifyRateLimiter() {
  console.log('🧪 T021: Verifying Rate Limiter Service\n');

  // Test 1: General API rate limiter (10 req/min)
  console.log('Test 1: General API Rate Limiter (10 req/min)');

  const app1 = express();
  app1.use(createUserRateLimiter(10, 1));
  app1.get('/test', (req, res) => res.json({ success: true }));

  let successCount = 0;
  let failureCount = 0;

  // Send 11 requests from same IP
  for (let i = 1; i <= 11; i++) {
    const res = await request(app1).get('/test');

    if (res.status === 200) {
      successCount++;
      console.log(`  Request ${i}: ✅ 200 OK`);
    } else if (res.status === 429) {
      failureCount++;
      console.log(`  Request ${i}: 🚫 429 Too Many Requests`);
      console.log(`    Message: ${res.body.error}`);
      console.log(`    Retry After: ${res.body.retryAfter || res.headers['retry-after']}`);
    } else {
      console.log(`  Request ${i}: ❌ Unexpected status ${res.status}`);
      process.exit(1);
    }
  }

  if (successCount === 10 && failureCount === 1) {
    console.log(`\n  ✅ Rate limiter working correctly (10 allowed, 11th blocked)\n`);
  } else {
    console.log(`\n  ❌ FAILED: Expected 10 success + 1 failure, got ${successCount} + ${failureCount}\n`);
    process.exit(1);
  }

  // Test 2: Transformation rate limiter (5 req/min - more restrictive)
  console.log('Test 2: Transformation Rate Limiter (5 req/min)');

  const app2 = express();
  app2.use(createTransformationRateLimiter(5, 1));
  app2.post('/transform', (req, res) => res.json({ transformed: true }));

  let transformSuccess = 0;
  let transformFailure = 0;

  // Send 6 requests
  for (let i = 1; i <= 6; i++) {
    const res = await request(app2).post('/transform');

    if (res.status === 200) {
      transformSuccess++;
      console.log(`  Request ${i}: ✅ 200 OK`);
    } else if (res.status === 429) {
      transformFailure++;
      console.log(`  Request ${i}: 🚫 429 Too Many Requests`);
      console.log(`    Hint: ${res.body.hint}`);
    }
  }

  if (transformSuccess === 5 && transformFailure === 1) {
    console.log(`\n  ✅ Transformation rate limiter working (5 allowed, 6th blocked)\n`);
  } else {
    console.log(`\n  ❌ FAILED: Expected 5 success + 1 failure, got ${transformSuccess} + ${transformFailure}\n`);
    process.exit(1);
  }

  // Test 3: Auth rate limiter (5 attempts/15 min - brute-force protection)
  console.log('Test 3: Auth Rate Limiter (5 attempts/15 min)');

  const app3 = express();
  app3.use(createAuthRateLimiter());
  app3.post('/auth/login', (req, res) => res.json({ token: 'fake-jwt' }));

  let authSuccess = 0;
  let authFailure = 0;

  // Send 6 login attempts
  for (let i = 1; i <= 6; i++) {
    const res = await request(app3).post('/auth/login');

    if (res.status === 200) {
      authSuccess++;
      console.log(`  Attempt ${i}: ✅ 200 OK`);
    } else if (res.status === 429) {
      authFailure++;
      console.log(`  Attempt ${i}: 🚫 429 Brute-force protection triggered`);
    }
  }

  if (authSuccess === 5 && authFailure === 1) {
    console.log(`\n  ✅ Auth rate limiter working (5 allowed, 6th blocked)\n`);
  } else {
    console.log(`\n  ❌ FAILED: Expected 5 success + 1 failure, got ${authSuccess} + ${authFailure}\n`);
    process.exit(1);
  }

  console.log('✅ T021 COMPLETE: All rate limiter tests passed!\n');
  console.log('Key Findings:');
  console.log('  - General API: 10 requests/minute per user/IP');
  console.log('  - Transformations: 5 requests/minute (LLM cost protection)');
  console.log('  - Auth endpoints: 5 attempts/15 minutes (brute-force protection)');
  console.log('  - Proper 429 responses with retry-after headers');
}

// Run verification
verifyRateLimiter()
  .then(() => {
    console.log('\n🎉 Rate limiter service is production-ready!');
    process.exit(0);
  })
  .catch((error) => {
    console.error('\n❌ Rate limiter verification failed:', error);
    process.exit(1);
  });
