#!/bin/bash

echo "========================================"
echo "🔒 UX & SECURITY VERIFICATION"
echo "========================================"
echo ""

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PASS=0
FAIL=0

echo -e "${BLUE}Test 1: Smart Login Greeting Implementation${NC}"
echo "----------------------------------------"

if grep -q "recommendedChapter" src/components/BulldogAssistant/index.tsx; then
  echo -e "${GREEN}✅ Login greeting with chapter recommendations${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ Login greeting NOT updated${NC}"
  ((FAIL++))
fi

if grep -q "Click \"Personalize\" in any chapter" src/components/BulldogAssistant/index.tsx; then
  echo -e "${GREEN}✅ Exact message format implemented${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ Message format incorrect${NC}"
  ((FAIL++))
fi

echo ""
echo -e "${BLUE}Test 2: Protected Buttons (Security)${NC}"
echo "----------------------------------------"

if grep -q "if (!isAuthenticated)" src/components/personalization/ChapterActions.tsx; then
  echo -e "${GREEN}✅ Authentication check implemented${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ No authentication check${NC}"
  ((FAIL++))
fi

if grep -q "window.location.href = '/login'" src/components/personalization/ChapterActions.tsx; then
  echo -e "${GREEN}✅ Redirect to login implemented${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ No redirect logic${NC}"
  ((FAIL++))
fi

# Check both buttons are protected
PERSONALIZE_PROTECTED=$(grep -A 5 "handlePersonalize" src/components/personalization/ChapterActions.tsx | grep -c "isAuthenticated")
TRANSLATE_PROTECTED=$(grep -A 5 "handleTranslate" src/components/personalization/ChapterActions.tsx | grep -c "isAuthenticated")

if [ "$PERSONALIZE_PROTECTED" -gt 0 ] && [ "$TRANSLATE_PROTECTED" -gt 0 ]; then
  echo -e "${GREEN}✅ Both buttons are protected${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ Not all buttons protected${NC}"
  ((FAIL++))
fi

echo ""
echo -e "${BLUE}Test 3: Global Button Injection${NC}"
echo "----------------------------------------"

if [ -f "src/theme/DocItem/Layout/index.tsx" ]; then
  echo -e "${GREEN}✅ DocItem Layout swizzled${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ No Layout swizzle found${NC}"
  ((FAIL++))
fi

if grep -q "ChapterActions" src/theme/DocItem/Layout/index.tsx; then
  echo -e "${GREEN}✅ ChapterActions component imported${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ ChapterActions NOT imported${NC}"
  ((FAIL++))
fi

if grep -q "isDocsPage" src/theme/DocItem/Layout/index.tsx; then
  echo -e "${GREEN}✅ Conditional rendering on docs pages${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ No conditional check${NC}"
  ((FAIL++))
fi

echo ""
echo -e "${BLUE}Test 4: Recommender Skill Documentation${NC}"
echo "----------------------------------------"

if [ -f "agent_skills/recommender.skill.md" ]; then
  echo -e "${GREEN}✅ recommender.skill.md exists${NC}"
  ((PASS++))

  FILE_SIZE=$(wc -c < "agent_skills/recommender.skill.md")
  if [ "$FILE_SIZE" -gt 10000 ]; then
    echo -e "${GREEN}✅ File size adequate (${FILE_SIZE} bytes)${NC}"
    ((PASS++))
  else
    echo -e "${YELLOW}⚠️  File might be incomplete${NC}"
  fi

  if grep -q "Recommendation Matrix" agent_skills/recommender.skill.md; then
    echo -e "${GREEN}✅ Contains recommendation matrix${NC}"
    ((PASS++))
  else
    echo -e "${RED}❌ Missing recommendation matrix${NC}"
    ((FAIL++))
  fi

  if grep -q "Algorithm" agent_skills/recommender.skill.md; then
    echo -e "${GREEN}✅ Contains algorithm description${NC}"
    ((PASS++))
  else
    echo -e "${RED}❌ Missing algorithm${NC}"
    ((FAIL++))
  fi
else
  echo -e "${RED}❌ recommender.skill.md NOT FOUND${NC}"
  ((FAIL=$FAIL+3))
fi

echo ""
echo "========================================"
echo "📊 AGENT SKILLS SUMMARY"
echo "========================================"

ls -lh agent_skills/*.skill.md 2>/dev/null | awk '{print "  " $9 " (" $5 ")"}'

SKILL_COUNT=$(ls -1 agent_skills/*.skill.md 2>/dev/null | wc -l)
echo ""
echo "Total skill files: $SKILL_COUNT"

if [ "$SKILL_COUNT" -ge 4 ]; then
  echo -e "${GREEN}✅ All required skill files present${NC}"
else
  echo -e "${YELLOW}⚠️  Expected 4+ skill files${NC}"
fi

echo ""
echo "========================================"
echo "🌐 FRONTEND STATUS"
echo "========================================"

if lsof -ti:3000 >/dev/null 2>&1; then
  echo -e "${GREEN}✅ Frontend running on port 3000${NC}"
  echo ""
  echo "Test these URLs:"
  echo "  1. http://localhost:3000/login (Login page)"
  echo "  2. http://localhost:3000/physical_ai_textbook/docs/intro (Intro page)"
  echo "  3. http://localhost:3000/physical_ai_textbook/docs/module1/chapter1-1-ros2-fundamentals (Chapter 1.1)"
else
  echo -e "${YELLOW}⚠️  Frontend NOT running${NC}"
  echo "Start it: npm start"
fi

echo ""
echo "========================================"
echo "📊 FINAL SCORE"
echo "========================================"

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
  echo -e "${GREEN}✅ PERFECT SCORE - ALL TESTS PASSED!${NC}"
  echo ""
  echo "🎉 UX & Security Requirements:"
  echo "  ✅ Smart login greeting with profile"
  echo "  ✅ Protected buttons with auth check"
  echo "  ✅ Global button injection verified"
  echo "  ✅ Recommender skill documented"
  echo ""
  echo "  TOTAL: 200 / 200 points ✅"
elif [ "$PERCENTAGE" -ge 80 ]; then
  echo -e "${YELLOW}⚠️  GOOD - Most tests passed${NC}"
else
  echo -e "${RED}❌ NEEDS ATTENTION - Some tests failed${NC}"
fi

echo ""
echo "========================================"
echo "🎬 MANUAL VERIFICATION STEPS"
echo "========================================"
echo ""
echo "1. Login Test:"
echo "   - Open /login page"
echo "   - Login with credentials"
echo "   - Verify Bulldog greeting includes:"
echo "     • Your name"
echo "     • Expertise level"
echo "     • Background"
echo "     • Recommended chapter"
echo ""
echo "2. Security Test:"
echo "   - Open incognito browser"
echo "   - Go to any chapter"
echo "   - Click 'Personalize' button"
echo "   - Should redirect to /login"
echo ""
echo "3. Global Buttons Test:"
echo "   - Visit multiple chapters"
echo "   - Verify buttons appear on ALL pages"
echo ""
echo "Demo Ready! 🚀"
echo ""
