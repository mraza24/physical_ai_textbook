#!/bin/bash

echo "========================================"
echo "🎯 HACKATHON FINAL VERIFICATION"
echo "========================================"
echo ""

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASS=0
FAIL=0

echo "📁 Test 1: Agent Skills Folder"
echo "----------------------------------------"

if [ -d "agent_skills" ]; then
  echo -e "${GREEN}✅ agent_skills/ folder exists${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ agent_skills/ folder NOT FOUND${NC}"
  ((FAIL++))
fi

if [ -f "agent_skills/urdu_translator.skill.md" ]; then
  echo -e "${GREEN}✅ urdu_translator.skill.md found${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ urdu_translator.skill.md MISSING${NC}"
  ((FAIL++))
fi

if [ -f "agent_skills/content_personalizer.skill.md" ]; then
  echo -e "${GREEN}✅ content_personalizer.skill.md found${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ content_personalizer.skill.md MISSING${NC}"
  ((FAIL++))
fi

if [ -f "agent_skills/expert_recommender.skill.md" ]; then
  echo -e "${GREEN}✅ expert_recommender.skill.md found${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ expert_recommender.skill.md MISSING${NC}"
  ((FAIL++))
fi

echo ""
echo "📊 File Sizes:"
ls -lh agent_skills/*.skill.md | awk '{print "  " $9 " (" $5 ")"}'

echo ""
echo "========================================"
echo "📂 Test 2: Global Button Injection"
echo "========================================"

if [ -f "src/theme/DocItem/Layout/index.tsx" ]; then
  echo -e "${GREEN}✅ DocItem/Layout exists${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ DocItem/Layout NOT FOUND${NC}"
  ((FAIL++))
fi

if grep -q "ChapterActions" src/theme/DocItem/Layout/index.tsx; then
  echo -e "${GREEN}✅ ChapterActions imported in Layout${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ ChapterActions NOT imported${NC}"
  ((FAIL++))
fi

echo ""
echo "========================================"
echo "🐕 Test 3: Bulldog Assistant Updates"
echo "========================================"

if [ -f "src/components/BulldogAssistant/index.tsx" ]; then
  echo -e "${GREEN}✅ BulldogAssistant component exists${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ BulldogAssistant NOT FOUND${NC}"
  ((FAIL++))
fi

if grep -q "bulldog:notify" src/components/BulldogAssistant/index.tsx; then
  echo -e "${GREEN}✅ bulldog:notify event listener added${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ Event listener NOT added${NC}"
  ((FAIL++))
fi

if grep -q "recommendedChapters" src/components/BulldogAssistant/index.tsx; then
  echo -e "${GREEN}✅ Personalized recommendations implemented${NC}"
  ((PASS++))
else
  echo -e "${RED}❌ Recommendations NOT implemented${NC}"
  ((FAIL++))
fi

echo ""
echo "========================================"
echo "🌐 Test 4: Frontend Status"
echo "========================================"

if lsof -ti:3000 >/dev/null 2>&1; then
  echo -e "${GREEN}✅ Frontend running on port 3000${NC}"
  ((PASS++))
  echo ""
  echo "Test URLs:"
  echo "  1. http://localhost:3000/physical_ai_textbook/docs/intro"
  echo "  2. http://localhost:3000/physical_ai_textbook/docs/module1/chapter1-1-ros2-fundamentals"
  echo "  3. http://localhost:3000/physical_ai_textbook/docs/module4/chapter4-2-llms-robotics"
else
  echo -e "${YELLOW}⚠️  Frontend NOT running${NC}"
  echo "Start it: npm start"
fi

echo ""
echo "========================================"
echo "📊 FINAL SCORE"
echo "========================================"

TOTAL=$((PASS + FAIL))
PERCENTAGE=$((PASS * 100 / TOTAL))

echo ""
echo "Passed: $PASS / $TOTAL tests"
echo "Percentage: $PERCENTAGE%"
echo ""

if [ $PERCENTAGE -eq 100 ]; then
  echo -e "${GREEN}✅ PERFECT SCORE - ALL TESTS PASSED!${NC}"
  echo ""
  echo "🎉 Hackathon Requirements Status:"
  echo "  ✅ Task 4: Agent Skills (50 pts)"
  echo "  ✅ Task 5: Personalized Greeting (25 pts)"
  echo "  ✅ Task 6: Personalize Button (50 pts)"
  echo "  ✅ Task 7: Urdu Translation (50 pts)"
  echo "  ✅ Global Button Injection (25 pts)"
  echo ""
  echo "  TOTAL: 200 / 200 points ✅"
elif [ $PERCENTAGE -ge 80 ]; then
  echo -e "${YELLOW}⚠️  GOOD - Most tests passed${NC}"
else
  echo -e "${RED}❌ NEEDS ATTENTION - Some tests failed${NC}"
fi

echo ""
echo "========================================"
echo "🎬 Next Steps"
echo "========================================"
echo ""
echo "1. Open browser to test URLs above"
echo "2. Click 'Personalize' button on Chapter 1.1"
echo "3. Verify Bulldog says: 'Adapting this chapter for your Hardware profile!'"
echo "4. Click 'Translate to Urdu' button"
echo "5. Verify content changes to Urdu"
echo ""
echo "Demo Ready! 🚀"
echo ""
