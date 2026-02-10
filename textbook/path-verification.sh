#!/bin/bash

echo "🔍 PATH VERIFICATION AFTER FIX"
echo "============================="
echo ""

passed=0
total=0

check_file() {
  local file=$1
  local pattern=$2
  local description=$3
  
  total=$((total + 1))
  
  if grep -q "$pattern" "$file" 2>/dev/null; then
    echo "✅ $description"
    passed=$((passed + 1))
  else
    echo "❌ $description"
  fi
}

echo "📋 Navbar Configuration (docusaurus.config.ts)"
echo "---"
check_file "docusaurus.config.ts" "to: 'login'" "Navbar Login → points to 'login' page"
check_file "docusaurus.config.ts" "label: 'Login'" "Navbar Login → labeled correctly"
echo ""

echo "🏠 Homepage Buttons (src/pages/index.tsx)"
echo "---"
check_file "src/pages/index.tsx" 'to={`${baseUrl}login`}' "Homepage Login → points to login page"
check_file "src/pages/index.tsx" 'to={`${baseUrl}docs/intro`}' "Homepage Get Started → points to docs/intro"
check_file "src/pages/index.tsx" "📝 Login" "Homepage Login → labeled correctly"
check_file "src/pages/index.tsx" "🚀 Get Started" "Homepage Get Started → labeled correctly"
echo ""

echo "📁 Required Files Exist"
echo "---"
if [ -f "src/pages/login.tsx" ]; then
  echo "✅ src/pages/login.tsx exists"
  passed=$((passed + 1))
else
  echo "❌ src/pages/login.tsx missing"
fi
total=$((total + 1))

if [ -f "src/pages/signup.tsx" ]; then
  echo "✅ src/pages/signup.tsx exists"
  passed=$((passed + 1))
else
  echo "❌ src/pages/signup.tsx missing"
fi
total=$((total + 1))

if [ -f "docs/intro.md" ]; then
  echo "✅ docs/intro.md exists"
  passed=$((passed + 1))
else
  echo "❌ docs/intro.md missing"
fi
total=$((total + 1))

echo ""
echo "============================="
echo "📊 VERIFICATION SUMMARY"
echo "============================="
echo "Passed: $passed / $total checks"
echo ""

if [ $passed -eq $total ]; then
  echo "🎉 ALL PATHS FIXED!"
  echo ""
  echo "Expected Navigation Flow:"
  echo "1. Navbar 'Login' → /physical_ai_textbook/login ✅"
  echo "2. Homepage 'Login' → /physical_ai_textbook/login ✅"
  echo "3. Homepage 'Get Started' → /physical_ai_textbook/docs/intro ✅"
  echo ""
  echo "🚀 Ready to test!"
  echo ""
  echo "Test Steps:"
  echo "1. npm start"
  echo "2. Click 'Login' (navbar) → should show login form"
  echo "3. Click 'Login' (homepage) → should show login form"
  echo "4. Click 'Get Started' → should show intro page"
else
  echo "⚠️ SOME CHECKS FAILED"
  echo "Please review failed items above."
fi
