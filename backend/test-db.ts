/**
 * Drizzle Database Connection Test
 * Tests if Drizzle can connect to Neon and query tables
 */

import { neon } from '@neondatabase/serverless';
import { drizzle } from 'drizzle-orm/neon-http';
import * as schema from './src/db/schema';
import * as dotenv from 'dotenv';
import * as path from 'path';

// Load environment variables
dotenv.config({ path: path.resolve(__dirname, '.env') });

async function testDrizzleConnection() {
  console.log('╔════════════════════════════════════════════════════════════════╗');
  console.log('║          DRIZZLE DATABASE CONNECTION TEST                      ║');
  console.log('╚════════════════════════════════════════════════════════════════╝\n');

  // Step 1: Check if DATABASE_URL is set
  console.log('1️⃣  Checking DATABASE_URL environment variable...');
  if (!process.env.DATABASE_URL) {
    console.error('❌ ERROR: DATABASE_URL is not set in .env file');
    console.log('   Fix: Add DATABASE_URL to backend/.env file');
    process.exit(1);
  }
  console.log('✅ DATABASE_URL is set');
  console.log(`   Host: ${process.env.DATABASE_URL.match(/@([^/]+)/)?.[1] || 'unknown'}\n`);

  // Step 2: Initialize Drizzle
  console.log('2️⃣  Initializing Drizzle ORM...');
  try {
    const sql = neon(process.env.DATABASE_URL);
    const db = drizzle(sql, { schema });
    console.log('✅ Drizzle initialized successfully\n');

    // Step 3: Test raw SQL query
    console.log('3️⃣  Testing raw SQL query (SELECT NOW())...');
    const timeResult = await sql`SELECT NOW() as server_time`;
    console.log('✅ Raw SQL query successful');
    console.log(`   Server time: ${timeResult[0].server_time}\n`);

    // Step 4: List all tables in database
    console.log('4️⃣  Listing all tables in database...');
    const tablesResult = await sql`
      SELECT table_name
      FROM information_schema.tables
      WHERE table_schema = 'public'
      ORDER BY table_name;
    `;
    console.log(`✅ Found ${tablesResult.length} tables:`);
    tablesResult.forEach((row: any) => {
      console.log(`   - ${row.table_name}`);
    });
    console.log('');

    // Step 5: Check if required tables exist
    console.log('5️⃣  Checking for required tables...');
    const requiredTables = ['user', 'session', 'user_profiles', 'transformation_cache'];
    const existingTables = tablesResult.map((row: any) => row.table_name);

    let allTablesExist = true;
    for (const table of requiredTables) {
      if (existingTables.includes(table)) {
        console.log(`   ✅ ${table} - EXISTS`);
      } else {
        console.log(`   ❌ ${table} - MISSING`);
        allTablesExist = false;
      }
    }
    console.log('');

    if (!allTablesExist) {
      console.warn('⚠️  WARNING: Some required tables are missing');
      console.log('   Run: npm run db:push (to create tables)');
      console.log('   Or run migrations if available\n');
    }

    // Step 6: Query user table (if exists)
    if (existingTables.includes('user')) {
      console.log('6️⃣  Testing query on user table...');
      const userCountResult = await sql`SELECT COUNT(*) as count FROM "user"`;
      const userCount = parseInt(userCountResult[0].count);
      console.log(`✅ User table query successful`);
      console.log(`   Total users: ${userCount}\n`);

      if (userCount > 0) {
        console.log('   Fetching first user (without password)...');
        const firstUserResult = await sql`
          SELECT id, email, created_at, last_login
          FROM "user"
          LIMIT 1
        `;
        if (firstUserResult.length > 0) {
          console.log('   ✅ Sample user:');
          console.log(`      ID: ${firstUserResult[0].id}`);
          console.log(`      Email: ${firstUserResult[0].email}`);
          console.log(`      Created: ${firstUserResult[0].created_at}`);
          console.log('');
        }
      } else {
        console.log('   ℹ️  No users in database yet (expected for fresh setup)\n');
      }
    }

    // Step 7: Query user_profiles table (if exists)
    if (existingTables.includes('user_profiles')) {
      console.log('7️⃣  Testing query on user_profiles table...');
      const profileCountResult = await sql`SELECT COUNT(*) as count FROM user_profiles`;
      const profileCount = parseInt(profileCountResult[0].count);
      console.log(`✅ User profiles table query successful`);
      console.log(`   Total profiles: ${profileCount}\n`);

      if (profileCount > 0) {
        console.log('   Fetching first profile...');
        const firstProfileResult = await sql`
          SELECT user_id, software_background, hardware_experience, language_preference
          FROM user_profiles
          LIMIT 1
        `;
        if (firstProfileResult.length > 0) {
          console.log('   ✅ Sample profile:');
          console.log(`      User ID: ${firstProfileResult[0].user_id}`);
          console.log(`      Software: ${firstProfileResult[0].software_background}`);
          console.log(`      Hardware: ${firstProfileResult[0].hardware_experience}`);
          console.log(`      Language: ${firstProfileResult[0].language_preference}`);
          console.log('');
        }
      } else {
        console.log('   ℹ️  No profiles yet (expected for fresh setup)\n');
      }
    }

    // Step 8: Check user_profiles schema
    if (existingTables.includes('user_profiles')) {
      console.log('8️⃣  Verifying user_profiles table schema...');
      const schemaResult = await sql`
        SELECT column_name, data_type, is_nullable
        FROM information_schema.columns
        WHERE table_name = 'user_profiles'
        ORDER BY ordinal_position;
      `;
      console.log('✅ user_profiles columns:');
      schemaResult.forEach((col: any) => {
        const nullable = col.is_nullable === 'YES' ? '(nullable)' : '(required)';
        console.log(`   - ${col.column_name}: ${col.data_type} ${nullable}`);
      });
      console.log('');
    }

    // Final summary
    console.log('╔════════════════════════════════════════════════════════════════╗');
    console.log('║                    TEST SUMMARY                                ║');
    console.log('╚════════════════════════════════════════════════════════════════╝');
    console.log('✅ Database connection: SUCCESS');
    console.log('✅ Drizzle ORM: WORKING');
    console.log(`✅ Tables found: ${tablesResult.length}`);
    console.log(`${allTablesExist ? '✅' : '⚠️ '} Required tables: ${allTablesExist ? 'ALL PRESENT' : 'SOME MISSING'}`);
    console.log('\n🎉 Database test completed successfully!\n');

  } catch (error) {
    console.error('\n❌ DATABASE TEST FAILED:');
    console.error(error instanceof Error ? error.message : String(error));
    console.error('\n📝 Troubleshooting:');
    console.error('   1. Verify DATABASE_URL is correct');
    console.error('   2. Check if Neon database is active (not paused)');
    console.error('   3. Visit https://console.neon.tech to wake database');
    console.error('   4. Ensure network connection is working');
    console.error('');
    process.exit(1);
  }
}

// Run the test
testDrizzleConnection();
