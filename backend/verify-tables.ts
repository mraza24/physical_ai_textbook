import { db } from './src/db/index';
import { user, userProfiles, transformationCache } from './src/db/schema';

async function verifyTables() {
  try {
    console.log('🔍 Verifying database tables...\n');
    
    // Check user table
    const users = await db.select().from(user).limit(0);
    console.log('✅ user table exists');
    
    // Check user_profiles table
    const profiles = await db.select().from(userProfiles).limit(0);
    console.log('✅ user_profiles table exists');
    
    // Check transformation_cache table
    const cache = await db.select().from(transformationCache).limit(0);
    console.log('✅ transformation_cache table exists');
    
    console.log('\n🎉 All tables created successfully!');
  } catch (error: any) {
    console.error('❌ Error verifying tables:', error.message);
    process.exit(1);
  } finally {
    process.exit(0);
  }
}

verifyTables();
