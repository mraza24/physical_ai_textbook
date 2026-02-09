/**
 * JWT Token Inspection Test
 * Generates and inspects JWT tokens to verify expiry dates
 */

import * as dotenv from 'dotenv';
import * as path from 'path';

// Load environment variables
dotenv.config({ path: path.resolve(__dirname, '.env') });

async function testJWT() {
  console.log('╔════════════════════════════════════════════════════════════════╗');
  console.log('║              JWT TOKEN INSPECTION TEST                         ║');
  console.log('╚════════════════════════════════════════════════════════════════╝\n');

  // Step 1: Check if JWT_SECRET is set
  console.log('1️⃣  Checking JWT_SECRET environment variable...');
  if (!process.env.JWT_SECRET) {
    console.error('❌ ERROR: JWT_SECRET is not set in .env file');
    console.log('   Fix: Add JWT_SECRET to backend/.env file');
    console.log('   Generate with: openssl rand -base64 32');
    process.exit(1);
  }
  console.log('✅ JWT_SECRET is set');
  console.log(`   Length: ${process.env.JWT_SECRET.length} characters\n`);

  // Step 2: Try to import and use Better-Auth
  console.log('2️⃣  Testing Better-Auth JWT generation...');
  try {
    const { auth } = await import('./src/auth/config');

    // Create a test user signup
    console.log('   Creating test JWT token...');
    const testEmail = `test-${Date.now()}@example.com`;

    const signupResult = await auth.api.signUpEmail({
      body: {
        name: 'Test User',
        email: testEmail,
        password: 'testpassword123',
      },
    });

    if (!signupResult || !signupResult.token) {
      console.error('❌ ERROR: Better-Auth did not return a token');
      console.log('   This might indicate a database issue');
      process.exit(1);
    }

    const token = signupResult.token;
    console.log('✅ JWT token generated successfully\n');

    // Step 3: Decode JWT token (without verification)
    console.log('3️⃣  Decoding JWT token...');
    const parts = token.split('.');
    if (parts.length !== 3) {
      console.error('❌ ERROR: Invalid JWT format (expected 3 parts)');
      process.exit(1);
    }

    const header = JSON.parse(Buffer.from(parts[0], 'base64url').toString());
    const payload = JSON.parse(Buffer.from(parts[1], 'base64url').toString());

    console.log('✅ JWT decoded successfully\n');

    // Step 4: Inspect token header
    console.log('4️⃣  JWT Header:');
    console.log(`   Algorithm: ${header.alg || 'unknown'}`);
    console.log(`   Type: ${header.typ || 'unknown'}`);
    console.log('');

    // Step 5: Inspect token payload
    console.log('5️⃣  JWT Payload:');
    console.log(`   User ID: ${payload.sub || payload.userId || 'unknown'}`);
    console.log(`   Email: ${payload.email || 'N/A'}`);

    if (payload.iat) {
      const issuedAt = new Date(payload.iat * 1000);
      console.log(`   Issued At (iat): ${issuedAt.toISOString()}`);
      console.log(`                    ${issuedAt.toLocaleString()}`);
    } else {
      console.log('   Issued At (iat): NOT SET ⚠️');
    }

    if (payload.exp) {
      const expiresAt = new Date(payload.exp * 1000);
      const now = new Date();
      const hoursUntilExpiry = (expiresAt.getTime() - now.getTime()) / (1000 * 60 * 60);

      console.log(`   Expires At (exp): ${expiresAt.toISOString()}`);
      console.log(`                     ${expiresAt.toLocaleString()}`);
      console.log(`   Time until expiry: ${hoursUntilExpiry.toFixed(2)} hours`);

      // Check if token is already expired
      if (expiresAt < now) {
        console.log('   ❌ WARNING: Token is ALREADY EXPIRED!');
        console.log('      This is a critical issue - tokens should not expire immediately');
      } else if (hoursUntilExpiry < 1) {
        console.log('   ⚠️  WARNING: Token expires in less than 1 hour');
      } else if (hoursUntilExpiry > 23 && hoursUntilExpiry < 25) {
        console.log('   ✅ Token expiry: 24 hours (expected)');
      } else {
        console.log(`   ℹ️  Token expiry: ${hoursUntilExpiry.toFixed(1)} hours`);
      }
    } else {
      console.log('   Expires At (exp): NOT SET ⚠️');
      console.log('      WARNING: Token never expires (security risk)');
    }

    console.log('');

    // Step 6: Check for suspicious dates
    console.log('6️⃣  Date sanity checks:');
    const currentYear = new Date().getFullYear();

    if (payload.iat) {
      const iatYear = new Date(payload.iat * 1000).getFullYear();
      if (iatYear < currentYear) {
        console.log(`   ⚠️  WARNING: Token issued in ${iatYear} (past year)`);
      } else if (iatYear === currentYear) {
        console.log(`   ✅ Issued date: ${iatYear} (current year)`);
      } else {
        console.log(`   ❌ ERROR: Token issued in ${iatYear} (future year!)`);
      }
    }

    if (payload.exp) {
      const expYear = new Date(payload.exp * 1000).getFullYear();
      if (expYear < currentYear) {
        console.log(`   ❌ ERROR: Token expires in ${expYear} (PAST YEAR - TOKEN IS EXPIRED!)`);
      } else if (expYear === currentYear) {
        console.log(`   ✅ Expiry date: ${expYear} (current year)`);
      } else {
        console.log(`   ⚠️  Expiry date: ${expYear} (future year)`);
      }
    }

    console.log('');

    // Step 7: Show full payload for debugging
    console.log('7️⃣  Full JWT Payload:');
    console.log(JSON.stringify(payload, null, 2));
    console.log('');

    // Step 8: Show token sample
    console.log('8️⃣  Sample Token (truncated):');
    console.log(`   ${token.substring(0, 50)}...${token.substring(token.length - 20)}`);
    console.log('');

    // Clean up test user
    console.log('9️⃣  Cleaning up test user...');
    try {
      const { db } = await import('./src/db/connection');
      const { user } = await import('./src/db/schema');
      const { eq } = await import('drizzle-orm');

      await db.delete(user).where(eq(user.email, testEmail));
      console.log('✅ Test user deleted\n');
    } catch (cleanupError) {
      console.log('⚠️  Could not delete test user (database might not be accessible)\n');
    }

    // Final summary
    console.log('╔════════════════════════════════════════════════════════════════╗');
    console.log('║                    TEST SUMMARY                                ║');
    console.log('╚════════════════════════════════════════════════════════════════╝');
    console.log('✅ JWT Secret: SET');
    console.log('✅ Token Generation: WORKING');
    console.log('✅ Token Format: VALID');

    if (payload.iat && payload.exp) {
      const iatYear = new Date(payload.iat * 1000).getFullYear();
      const expYear = new Date(payload.exp * 1000).getFullYear();
      const isExpired = new Date(payload.exp * 1000) < new Date();

      if (isExpired) {
        console.log('❌ Token Status: EXPIRED (Critical Issue!)');
      } else if (iatYear === currentYear && expYear === currentYear) {
        console.log('✅ Token Dates: VALID (2026)');
      } else {
        console.log('⚠️  Token Dates: CHECK PAYLOAD ABOVE');
      }
    } else {
      console.log('⚠️  Token Dates: INCOMPLETE (missing iat or exp)');
    }

    console.log('\n🎉 JWT test completed!\n');

  } catch (error) {
    console.error('\n❌ JWT TEST FAILED:');
    console.error(error instanceof Error ? error.message : String(error));
    console.error('\n📝 Troubleshooting:');
    console.error('   1. Ensure DATABASE_URL is set and database is accessible');
    console.error('   2. Verify JWT_SECRET is set in .env');
    console.error('   3. Check Better-Auth configuration in src/auth/config.ts');
    console.error('');
    if (error instanceof Error && error.stack) {
      console.error('Stack trace:');
      console.error(error.stack);
    }
    process.exit(1);
  }
}

// Run the test
testJWT();
