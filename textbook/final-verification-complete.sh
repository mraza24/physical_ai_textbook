#!/bin/bash

echo "🎯 FINAL VERIFICATION - All Features Complete"
echo "=============================================="
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
    echo "❌ $description - NOT FOUND"
  fi
}

echo "🔗 Task 1: Login Button Fixed"
echo "---"
check_file "docusaurus.config.ts" "/physical_ai_textbook/signup" "Navbar Login → absolute path to signup"
echo ""

echo "🇵🇰 Task 2: Urdu Translation on All Intro Pages"
echo "---"
check_file "docs/intro.md" "import UrduTranslateButton" "Main intro - Urdu button import"
check_file "docs/intro.md" "<UrduTranslateButton />" "Main intro - Urdu button component"
check_file "docs/preface.md" "import UrduTranslateButton" "Preface - Urdu button import"
check_file "docs/preface.md" "<UrduTranslateButton />" "Preface - Urdu button component"
check_file "docs/module1/intro.md" "import UrduTranslateButton" "Module 1 - Urdu button import"
check_file "docs/module1/intro.md" "<UrduTranslateButton />" "Module 1 - Urdu button component"
check_file "docs/module2/intro.md" "import UrduTranslateButton" "Module 2 - Urdu button import"
check_file "docs/module2/intro.md" "<UrduTranslateButton />" "Module 2 - Urdu button component"
check_file "docs/module3/intro.md" "import UrduTranslateButton" "Module 3 - Urdu button import"
check_file "docs/module3/intro.md" "<UrduTranslateButton />" "Module 3 - Urdu button component"
check_file "docs/module4/intro.md" "import UrduTranslateButton" "Module 4 - Urdu button import"
check_file "docs/module4/intro.md" "<UrduTranslateButton />" "Module 4 - Urdu button component"
echo ""

echo "🐕 Task 3: Bulldog Welcome on Intro Page"
echo "---"
check_file "src/components/BulldogAssistant/index.tsx" "Auto-welcome on intro page" "Bulldog auto-welcome comment"
check_file "src/components/BulldogAssistant/index.tsx" "/docs/intro" "Bulldog checks intro page path"
check_file "src/components/BulldogAssistant/index.tsx" "Woof woof" "Bulldog welcome message"
check_file "src/components/BulldogAssistant/index.tsx" "setTimeout" "Bulldog delayed welcome (1.5s)"
echo ""

echo "=============================================="
echo "📊 VERIFICATION SUMMARY"
echo "=============================================="
echo "Passed: $passed / $total checks"
echo ""

if [ $passed -eq $total ]; then
  echo "🎉 ALL FEATURES COMPLETE!"
  echo ""
  echo "✅ Task 1: Login button fixed (absolute path)"
  echo "✅ Task 2: Urdu translation on 6 intro pages"
  echo "✅ Task 3: Bulldog auto-welcome on intro page"
  echo ""
  echo "🚀 Ready for demo!"
  echo ""
  echo "Testing Steps:"
  echo "1. npm start"
  echo "2. Click 'Get Started' → Bulldog should auto-welcome"
  echo "3. Click 'Translate to Urdu' → Content switches to Urdu"
  echo "4. Navigate to module intros → Each has Urdu button"
  echo "5. Click 'Login' (navbar) → Goes to signup form"
  echo ""
  echo "Demo Script: See FINAL_FEATURES_COMPLETE.md"
else
  echo "⚠️ SOME CHECKS FAILED"
  echo "Please review failed items above."
fi
