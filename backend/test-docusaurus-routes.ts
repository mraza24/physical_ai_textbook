/**
 * Docusaurus Route Audit Script
 * Checks for 404 errors and route mismatches
 */

import * as fs from 'fs';
import * as path from 'path';

function testDocusaurusRoutes() {
  console.log('╔════════════════════════════════════════════════════════════════╗');
  console.log('║            DOCUSAURUS ROUTE AUDIT                              ║');
  console.log('╚════════════════════════════════════════════════════════════════╝\n');

  const textbookDir = path.resolve(__dirname, '../textbook');
  const configPath = path.join(textbookDir, 'docusaurus.config.ts');
  const pagesDir = path.join(textbookDir, 'src/pages');

  // Test 1: Check if textbook directory exists
  console.log('1️⃣  Checking Docusaurus directory...');
  if (!fs.existsSync(textbookDir)) {
    console.error('❌ ERROR: textbook directory not found');
    console.error(`   Expected: ${textbookDir}`);
    process.exit(1);
  }
  console.log('✅ Textbook directory exists');
  console.log(`   Path: ${textbookDir}\n`);

  // Test 2: Check docusaurus.config.ts
  console.log('2️⃣  Checking docusaurus.config.ts...');
  if (!fs.existsSync(configPath)) {
    console.error('❌ ERROR: docusaurus.config.ts not found');
    console.error(`   Expected: ${configPath}`);
    process.exit(1);
  }
  console.log('✅ docusaurus.config.ts exists\n');

  // Test 3: Read and parse config
  console.log('3️⃣  Reading Docusaurus configuration...');
  try {
    const configContent = fs.readFileSync(configPath, 'utf-8');

    // Extract baseUrl (regex since it's TypeScript, not JSON)
    const baseUrlMatch = configContent.match(/baseUrl:\s*['"](.+?)['"]/);
    const urlMatch = configContent.match(/url:\s*['"](.+?)['"]/);
    const customFieldsMatch = configContent.match(/customFields:\s*\{([^}]+)\}/s);

    if (baseUrlMatch) {
      console.log(`✅ baseUrl: "${baseUrlMatch[1]}"`);
    } else {
      console.log('⚠️  baseUrl: NOT FOUND');
    }

    if (urlMatch) {
      console.log(`✅ url: "${urlMatch[1]}"`);
    } else {
      console.log('⚠️  url: NOT FOUND');
    }

    if (customFieldsMatch) {
      console.log('✅ customFields: PRESENT');
      const backendUrlMatch = customFieldsMatch[1].match(/backendUrl:\s*['"](.+?)['"]/);
      if (backendUrlMatch) {
        console.log(`   backendUrl: "${backendUrlMatch[1]}"`);
      }
    } else {
      console.log('⚠️  customFields: NOT FOUND');
    }

    console.log('');

    // Test 4: Check for common issues
    console.log('4️⃣  Checking for common configuration issues...');

    const baseUrl = baseUrlMatch ? baseUrlMatch[1] : '/';
    const issues: string[] = [];

    if (baseUrl === '/') {
      console.log('   ℹ️  baseUrl is "/" (root) - works for local dev');
    } else if (baseUrl.startsWith('/') && baseUrl.endsWith('/')) {
      console.log(`   ✅ baseUrl format correct: "${baseUrl}"`);
      console.log(`      - All routes will be prefixed with: ${baseUrl}`);
      console.log(`      - /signup becomes: ${baseUrl}signup`);
      console.log(`      - /login becomes: ${baseUrl}login`);
    } else {
      issues.push(`baseUrl should start and end with "/" (got: "${baseUrl}")`);
    }

    if (issues.length > 0) {
      console.log('   ⚠️  Issues found:');
      issues.forEach(issue => console.log(`      - ${issue}`));
    } else {
      console.log('   ✅ No configuration issues found');
    }

    console.log('');

  } catch (error) {
    console.error('❌ ERROR reading config:', error);
    process.exit(1);
  }

  // Test 5: Check src/pages directory
  console.log('5️⃣  Checking src/pages directory...');
  if (!fs.existsSync(pagesDir)) {
    console.error('❌ ERROR: src/pages directory not found');
    console.error(`   Expected: ${pagesDir}`);
    process.exit(1);
  }
  console.log('✅ src/pages directory exists\n');

  // Test 6: List all page files
  console.log('6️⃣  Scanning for page files...');
  const pageFiles = fs.readdirSync(pagesDir, { recursive: true });
  const tsxFiles = (pageFiles as string[]).filter(f => f.endsWith('.tsx') || f.endsWith('.jsx'));

  console.log(`✅ Found ${tsxFiles.length} page files:\n`);

  const requiredPages = ['signup.tsx', 'login.tsx', 'index.tsx'];
  const foundPages: { [key: string]: boolean } = {};

  tsxFiles.forEach(file => {
    const fileName = path.basename(file);
    console.log(`   - ${file}`);

    // Check if it's a required page
    if (requiredPages.includes(fileName)) {
      foundPages[fileName] = true;
    }
  });

  console.log('');

  // Test 7: Verify required pages exist
  console.log('7️⃣  Verifying required pages...');

  const missingPages: string[] = [];

  requiredPages.forEach(requiredPage => {
    if (foundPages[requiredPage]) {
      console.log(`   ✅ ${requiredPage} - EXISTS`);
    } else {
      console.log(`   ❌ ${requiredPage} - MISSING`);
      missingPages.push(requiredPage);
    }
  });

  console.log('');

  if (missingPages.length > 0) {
    console.error('⚠️  WARNING: Missing required pages');
    console.log('   These pages must exist in src/pages/');
    console.log('   Missing:', missingPages.join(', '));
  }

  // Test 8: Check if pages use baseUrl correctly
  console.log('8️⃣  Checking if pages use baseUrl correctly...');

  const signupPath = path.join(pagesDir, 'signup.tsx');
  const loginPath = path.join(pagesDir, 'login.tsx');

  if (fs.existsSync(signupPath)) {
    const signupContent = fs.readFileSync(signupPath, 'utf-8');

    // Check if it uses siteConfig.baseUrl
    if (signupContent.includes('siteConfig.baseUrl')) {
      console.log('   ✅ signup.tsx uses siteConfig.baseUrl');
    } else {
      console.log('   ⚠️  signup.tsx might not use siteConfig.baseUrl');
    }

    // Check for hard-coded paths
    if (signupContent.match(/href=["']\/login["']/)) {
      console.log('   ⚠️  signup.tsx has hard-coded "/login" (should use baseUrl)');
    } else {
      console.log('   ✅ signup.tsx appears to use dynamic paths');
    }
  }

  if (fs.existsSync(loginPath)) {
    const loginContent = fs.readFileSync(loginPath, 'utf-8');

    if (loginContent.includes('siteConfig.baseUrl')) {
      console.log('   ✅ login.tsx uses siteConfig.baseUrl');
    } else {
      console.log('   ⚠️  login.tsx might not use siteConfig.baseUrl');
    }

    if (loginContent.match(/href=["']\/signup["']/)) {
      console.log('   ⚠️  login.tsx has hard-coded "/signup" (should use baseUrl)');
    } else {
      console.log('   ✅ login.tsx appears to use dynamic paths');
    }
  }

  console.log('');

  // Test 9: Check Root.tsx for AuthProvider
  console.log('9️⃣  Checking Root.tsx for AuthProvider...');
  const rootPath = path.join(textbookDir, 'src/theme/Root.tsx');

  if (fs.existsSync(rootPath)) {
    const rootContent = fs.readFileSync(rootPath, 'utf-8');

    console.log('   ✅ Root.tsx exists');

    if (rootContent.includes('AuthProvider')) {
      console.log('   ✅ Root.tsx imports AuthProvider');
    } else {
      console.log('   ❌ Root.tsx does NOT import AuthProvider');
      console.log('      This will cause "useAuth must be used within AuthProvider" error');
    }

    if (rootContent.includes('<AuthProvider>')) {
      console.log('   ✅ Root.tsx wraps children with <AuthProvider>');
    } else {
      console.log('   ❌ Root.tsx does NOT wrap children with <AuthProvider>');
    }
  } else {
    console.log('   ⚠️  Root.tsx not found');
    console.log(`      Expected: ${rootPath}`);
  }

  console.log('');

  // Test 10: Expected URLs
  console.log('🔟 Expected URLs (based on configuration):');
  const configContent = fs.readFileSync(configPath, 'utf-8');
  const baseUrlMatch = configContent.match(/baseUrl:\s*['"](.+?)['"]/);
  const baseUrl = baseUrlMatch ? baseUrlMatch[1] : '/';

  console.log(`   Local Development:`);
  console.log(`   - Home: http://localhost:3000${baseUrl}`);
  console.log(`   - Signup: http://localhost:3000${baseUrl}signup`);
  console.log(`   - Login: http://localhost:3000${baseUrl}login`);
  console.log(`   - Docs: http://localhost:3000${baseUrl}docs/intro`);
  console.log('');

  // Final Summary
  console.log('╔════════════════════════════════════════════════════════════════╗');
  console.log('║                    TEST SUMMARY                                ║');
  console.log('╚════════════════════════════════════════════════════════════════╝');

  const allPagesExist = missingPages.length === 0;
  const hasValidConfig = baseUrlMatch !== null;
  const hasAuthProvider = fs.existsSync(rootPath) &&
    fs.readFileSync(rootPath, 'utf-8').includes('<AuthProvider>');

  console.log(`✅ Configuration file: FOUND`);
  console.log(`${hasValidConfig ? '✅' : '❌'} baseUrl: ${hasValidConfig ? 'SET' : 'NOT SET'}`);
  console.log(`${allPagesExist ? '✅' : '⚠️ '} Required pages: ${allPagesExist ? 'ALL PRESENT' : 'SOME MISSING'}`);
  console.log(`${hasAuthProvider ? '✅' : '❌'} AuthProvider: ${hasAuthProvider ? 'CONFIGURED' : 'NOT CONFIGURED'}`);

  console.log('');

  if (!allPagesExist) {
    console.log('📝 Action needed:');
    console.log('   - Missing pages:', missingPages.join(', '));
    console.log('   - These files must exist in textbook/src/pages/');
  }

  if (!hasAuthProvider) {
    console.log('📝 Action needed:');
    console.log('   - Add AuthProvider to textbook/src/theme/Root.tsx');
    console.log('   - This will fix "useAuth must be used within AuthProvider" error');
  }

  console.log('\n🎉 Docusaurus route audit completed!\n');
}

// Run the test
testDocusaurusRoutes();
