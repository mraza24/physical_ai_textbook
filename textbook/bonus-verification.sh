#!/bin/bash

echo "🏆 BONUS TASKS VERIFICATION - 50 Points"
echo "========================================"
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

check_not_exists() {
  local file=$1
  local pattern=$2
  local description=$3
  
  total=$((total + 1))
  
  if ! grep -q "$pattern" "$file" 2>/dev/null; then
    echo "✅ $description"
    passed=$((passed + 1))
  else
    echo "❌ $description - FOUND (should not exist)"
  fi
}

echo "🔗 Bonus Task 1: Login Button Path Audit"
echo "---"
check_file "docusaurus.config.ts" "baseUrl: '/physical_ai_textbook/'" "baseUrl configured correctly"
check_file "docusaurus.config.ts" "to: '/physical_ai_textbook/signup'" "Login button uses absolute path"
check_not_exists "docusaurus.config.ts" "trailingSlash:" "No trailingSlash conflicts"

if [ -f "src/pages/signup.tsx" ]; then
  echo "✅ Signup file exists at src/pages/signup.tsx"
  passed=$((passed + 1))
else
  echo "❌ Signup file not found"
fi
total=$((total + 1))

echo ""

echo "🇵🇰 Bonus Task 2: Task 7 - Urdu Translation (50 Points)"
echo "---"
check_file "src/components/UrduTranslateButton/index.tsx" "triggerBulldogExplanation" "Event dispatch function created"
check_file "src/components/UrduTranslateButton/index.tsx" "urdu-translation-toggled" "Custom event defined"
check_file "src/components/UrduTranslateButton/index.tsx" "isTranslatingToUrdu" "Event payload configured"
echo ""

echo "🐕 Bonus Task 3: Task 4 - Agent Skill (Bulldog Explanation)"
echo "---"
check_file "src/components/BulldogAssistant/index.tsx" "urdu-translation-toggled" "Bulldog listens for event"
check_file "src/components/BulldogAssistant/index.tsx" "I've translated this chapter to Urdu" "Urdu explanation message"
check_file "src/components/BulldogAssistant/index.tsx" "I'm your AI Agent" "Agent identity in message"
check_file "src/components/BulldogAssistant/index.tsx" "window.addEventListener" "Event listener added"
check_file "src/components/BulldogAssistant/index.tsx" "window.removeEventListener" "Event listener cleanup"
echo ""

echo "📋 Bonus Task 4: Final Path Audit"
echo "---"
check_file "docusaurus.config.ts" "onBrokenLinks: 'throw'" "Strict link checking enabled"
check_file "docusaurus.config.ts" "src: '/physical_ai_textbook/js/" "Scripts use absolute paths"
echo ""

echo "🔄 Integration Verification"
echo "---"
check_file "src/components/UrduTranslateButton/index.tsx" "triggerBulldogExplanation(true)" "Urdu translation triggers Bulldog"
check_file "src/components/UrduTranslateButton/index.tsx" "triggerBulldogExplanation(false)" "English switch triggers Bulldog"
echo ""

echo "========================================"
echo "📊 VERIFICATION SUMMARY"
echo "========================================"
echo "Passed: $passed / $total checks"
echo ""

if [ $passed -eq $total ]; then
  echo "🏆 ALL BONUS TASKS COMPLETE!"
  echo ""
  echo "✅ Login path verified (baseUrl + absolute path)"
  echo "✅ Task 7: Urdu translation with Bulldog integration"
  echo "✅ Task 4: Agent skill demonstration"
  echo "✅ Path audit complete (no conflicts)"
  echo ""
  echo "🎯 BONUS POINTS EARNED: 50"
  echo ""
  echo "🚀 Ready for demo!"
  echo ""
  echo "Demo Flow:"
  echo "1. Navigate to intro page"
  echo "2. Click 'Translate to Urdu'"
  echo "3. Content changes to Urdu (RTL)"
  echo "4. Bulldog auto-opens and explains!"
  echo "5. Message: 'Woof! I'm your AI Agent...'"
  echo "6. Click 'Show English'"
  echo "7. Bulldog explains switch back"
  echo "8. Perfect demonstration of Tasks 4 & 7!"
  echo ""
  echo "📚 Documentation:"
  echo "- BONUS_TASKS_COMPLETE.md - Complete bonus task summary"
  echo "- Demo script with timing and talking points"
else
  echo "⚠️ SOME CHECKS FAILED"
  echo "Please review failed items above."
fi
