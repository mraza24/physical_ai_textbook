#!/bin/bash

echo "🔍 VERIFYING ALL NUCLEAR FIXES ARE APPLIED"
echo "=========================================="
echo ""

# Counter for passed checks
passed=0
total=0

# Function to check file content
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

# Function to check file exists
check_exists() {
  local file=$1
  local description=$2
  
  total=$((total + 1))
  
  if [ -f "$file" ]; then
    echo "✅ $description"
    passed=$((passed + 1))
  else
    echo "❌ $description - FILE MISSING"
  fi
}

echo "📁 Checking File Existence:"
echo "---"
check_exists "static/js/click-fixer.js" "Runtime click fixer script exists"
check_exists "static/js/ghost-div-detector.js" "Ghost div detector script exists"
check_exists "NUCLEAR_FIX_COMPLETE.md" "Complete documentation exists"
echo ""

echo "🎨 Checking CSS Fixes (src/css/custom.css):"
echo "---"
check_file "src/css/custom.css" "DEBUG MODE" "DEBUG MODE section present"
check_file "src/css/custom.css" "z-index: 9999 !important" "Navbar z-index: 9999"
check_file "src/css/custom.css" "border: 2px solid red !important" "Red border debug indicator"
check_file "src/css/custom.css" "outline: 2px solid green !important" "Green outline debug indicator"
check_file "src/css/custom.css" "NUCLEAR OPTION" "Nuclear Docusaurus overrides"
check_file "src/css/custom.css" "#__docusaurus" "Docusaurus main wrapper override"
echo ""

echo "🐕 Checking Bulldog CSS (src/components/BulldogAssistant/styles.module.css):"
echo "---"
check_file "src/components/BulldogAssistant/styles.module.css" "pointer-events: none !important" "Container non-blocking"
check_file "src/components/BulldogAssistant/styles.module.css" "bottom: 20px !important" "Bottom positioning with !important"
check_file "src/components/BulldogAssistant/styles.module.css" "right: 20px !important" "Right positioning with !important"
echo ""

echo "⚛️ Checking Bulldog React Portal (src/components/BulldogAssistant/index.tsx):"
echo "---"
check_file "src/components/BulldogAssistant/index.tsx" "createPortal" "React Portal import"
check_file "src/components/BulldogAssistant/index.tsx" "document.body" "Portal to document.body"
check_file "src/components/BulldogAssistant/index.tsx" "width: 0" "Zero-width wrapper"
echo ""

echo "🏠 Checking Homepage Buttons (src/pages/index.module.css):"
echo "---"
check_file "src/pages/index.module.css" "z-index: 9998 !important" "Homepage button z-index"
check_file "src/pages/index.module.css" "pointer-events: auto !important" "Homepage button pointer-events"
echo ""

echo "⚙️ Checking Docusaurus Config (docusaurus.config.ts):"
echo "---"
check_file "docusaurus.config.ts" "scripts:" "Scripts array added"
check_file "docusaurus.config.ts" "click-fixer.js" "Click fixer script loaded"
echo ""

echo "📜 Checking Runtime Scripts:"
echo "---"
check_file "static/js/click-fixer.js" "forceButtonClickability" "Click fixer function defined"
check_file "static/js/click-fixer.js" "z-index.*9999" "Forces navbar z-index: 9999"
check_file "static/js/click-fixer.js" "MutationObserver" "Route change detection"
check_file "static/js/ghost-div-detector.js" "Ghost Div Detector" "Ghost div detector header"
check_file "static/js/ghost-div-detector.js" "outline.*yellow" "Visual highlighting"
echo ""

echo "=========================================="
echo "📊 VERIFICATION SUMMARY"
echo "=========================================="
echo "Passed: $passed / $total checks"
echo ""

if [ $passed -eq $total ]; then
  echo "🎉 ALL CHECKS PASSED! Nuclear fix is complete."
  echo ""
  echo "Next steps:"
  echo "1. npm start"
  echo "2. Hard refresh browser (Ctrl+Shift+R)"
  echo "3. Check for red navbar border and green button outlines"
  echo "4. Test all button clicks"
  echo "5. Check browser console for 'Click Fixer - Complete!'"
  echo ""
  echo "Ready for Task 7 demo! 🚀"
else
  echo "⚠️ SOME CHECKS FAILED!"
  echo "Please review the failed items above."
  echo ""
  echo "Common fixes:"
  echo "- Re-run the fix scripts"
  echo "- Check file paths"
  echo "- Ensure all edits were saved"
fi
