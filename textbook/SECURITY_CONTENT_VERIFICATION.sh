#!/bin/bash

echo "=========================================="
echo "🔒 SECURITY & CONTENT VERIFICATION"
echo "=========================================="
echo ""

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PASS=0
FAIL=0

echo -e "${BLUE}Test 1: Auth Guard Implementation${NC}"
echo "----------------------------------------"

if grep -q "CRITICAL AUTH GUARD" src/components/personalization/ChapterActions.tsx; then
  echo -e "${GREEN}✅ Auth guard comments present${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ Auth guard comments missing${NC}"
  ((FAIL++))
fi

if grep -q "alert('Login required to access AI features" src/components/personalization/ChapterActions.tsx; then
  echo -e "${GREEN}✅ Alert popup implemented${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ Alert popup missing${NC}"
  ((FAIL++))
fi

if grep -q "setTimeout.*window.location.href = '/login'" src/components/personalization/ChapterActions.tsx; then
  echo -e "${GREEN}✅ Delayed redirect implemented${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ Delayed redirect missing${NC}"
  ((FAIL++))
fi

echo ""
echo -e "${BLUE}Test 2: Task 6 Logic (Profile Validation)${NC}"
echo "----------------------------------------"

if grep -q "software_background.*hardware_experience" src/components/personalization/ChapterActions.tsx; then
  echo -e "${GREEN}✅ Profile validation checks present${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ Profile validation missing${NC}"
  ((FAIL++))
fi

if grep -q "User background data not found" src/components/personalization/ChapterActions.tsx; then
  echo -e "${GREEN}✅ Error message for missing profile${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ Error message missing${NC}"
  ((FAIL++))
fi

echo ""
echo -e "${BLUE}Test 3: Visual Lock State${NC}"
echo "----------------------------------------"

if grep -q "lockedButton" src/components/personalization/ChapterActions.tsx; then
  echo -e "${GREEN}✅ Lock class applied to buttons${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ Lock class not applied${NC}"
  ((FAIL++))
fi

if grep -q ".lockedButton::before" src/components/personalization/ChapterActions.module.css; then
  echo -e "${GREEN}✅ Lock icon CSS implemented${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ Lock icon CSS missing${NC}"
  ((FAIL++))
fi

if grep -q "lockBounce" src/components/personalization/ChapterActions.module.css; then
  echo -e "${GREEN}✅ Lock animation implemented${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ Lock animation missing${NC}"
  ((FAIL++))
fi

echo ""
echo -e "${BLUE}Test 4: Chapter 1.3 Content${NC}"
echo "----------------------------------------"

CH13_LINES=$(wc -l < docs/module1/chapter1-3-launch-files.md)
CH13_PLACEHOLDERS=$(grep -c "Content to be added" docs/module1/chapter1-3-launch-files.md 2>/dev/null || echo "0")

if [ "$CH13_LINES" -ge 500 ]; then
  echo -e "${GREEN}✅ Chapter 1.3 has $CH13_LINES lines (target: 500+)${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ Chapter 1.3 only has $CH13_LINES lines${NC}"
  ((FAIL++))
fi

if [ "$CH13_PLACEHOLDERS" -eq 0 ]; then
  echo -e "${GREEN}✅ Chapter 1.3 has no placeholders${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ Chapter 1.3 has $CH13_PLACEHOLDERS placeholders${NC}"
  ((FAIL++))
fi

echo ""
echo -e "${BLUE}Test 5: Chapter 1.4 Content${NC}"
echo "----------------------------------------"

CH14_LINES=$(wc -l < docs/module1/chapter1-4-packages.md)
CH14_PLACEHOLDERS=$(grep -c "Content to be added" docs/module1/chapter1-4-packages.md 2>/dev/null || echo "0")

if [ "$CH14_LINES" -ge 600 ]; then
  echo -e "${GREEN}✅ Chapter 1.4 has $CH14_LINES lines (target: 600+)${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ Chapter 1.4 only has $CH14_LINES lines${NC}"
  ((FAIL++))
fi

if [ "$CH14_PLACEHOLDERS" -eq 0 ]; then
  echo -e "${GREEN}✅ Chapter 1.4 has no placeholders${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ Chapter 1.4 has $CH14_PLACEHOLDERS placeholders${NC}"
  ((FAIL++))
fi

echo ""
echo "=========================================="
echo "📊 FINAL SCORE"
echo "=========================================="

TOTAL=$((PASS + FAIL))
if [ "$TOTAL" -eq 0 ]; then
  PERCENTAGE=0
else
  PERCENTAGE=$((PASS * 100 / TOTAL))
fi

echo ""
echo "Passed: $PASS / $TOTAL tests"
echo "Percentage: $PERCENTAGE%"
echo ""

if [ "$PERCENTAGE" -eq 100 ]; then
  echo -e "${GREEN}✅ PERFECT SCORE - ALL SECURITY & CONTENT REQUIREMENTS MET!${NC}"
  echo ""
  echo "🔒 Security Features:"
  echo "  ✅ Auth guard with alert popup"
  echo "  ✅ Profile validation (Task 6)"
  echo "  ✅ Visual lock state with animation"
  echo "  ✅ Delayed redirect (500ms)"
  echo ""
  echo "📝 Content Status:"
  echo "  ✅ Chapter 1.3: $CH13_LINES lines, $CH13_PLACEHOLDERS placeholders"
  echo "  ✅ Chapter 1.4: $CH14_LINES lines, $CH14_PLACEHOLDERS placeholders"
  echo ""
  echo "  HACKATHON READY: 100% ✅"
elif [ "$PERCENTAGE" -ge 80 ]; then
  echo -e "${YELLOW}⚠️  GOOD - Most requirements met${NC}"
else
  echo -e "${RED}❌ NEEDS ATTENTION - Some requirements not met${NC}"
fi

echo ""
echo "=========================================="
echo "🎬 MANUAL VERIFICATION STEPS"
echo "=========================================="
echo ""
echo "1. Security Test (Not Logged In):"
echo "   - Open incognito browser"
echo "   - Go to Chapter 1.3 or 1.4"
echo "   - Verify lock icon 🔒 visible on buttons"
echo "   - Click Personalize → Should show alert"
echo "   - Verify redirect to /login"
echo ""
echo "2. Content Test:"
echo "   - Navigate to Chapter 1.3"
echo "   - Verify comprehensive ROS 2 Launch files content"
echo "   - Navigate to Chapter 1.4"
echo "   - Verify comprehensive ROS 2 Packages content"
echo ""
echo "3. Authenticated Test:"
echo "   - Login with credentials"
echo "   - Go to any chapter"
echo "   - Verify lock icons gone"
echo "   - Verify buttons work correctly"
echo ""
echo "Hackathon Deployment Ready! 🚀"
echo ""
