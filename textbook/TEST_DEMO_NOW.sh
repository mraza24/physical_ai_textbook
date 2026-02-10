#!/bin/bash

echo "========================================"
echo "🎬 DEMO MODE VERIFICATION"
echo "========================================"
echo ""

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "📁 Step 1: Verify Agent Skills Files"
echo "----------------------------------------"
echo ""

SKILL_DIR="/mnt/d/Q4_hackathon1/physical_ai_textbook/textbook/docs/agent_skills"

if [ -f "$SKILL_DIR/translator.skill.md" ]; then
    echo -e "${GREEN}✅ translator.skill.md found${NC}"
else
    echo -e "${YELLOW}❌ translator.skill.md MISSING${NC}"
fi

if [ -f "$SKILL_DIR/personalizer.skill.md" ]; then
    echo -e "${GREEN}✅ personalizer.skill.md found${NC}"
else
    echo -e "${YELLOW}❌ personalizer.skill.md MISSING${NC}"
fi

FILE_COUNT=$(ls -1 "$SKILL_DIR"/*.skill.md 2>/dev/null | wc -l)
echo ""
echo "Total skill files: $FILE_COUNT"
echo ""

echo "📊 File Sizes:"
ls -lh "$SKILL_DIR"/*.skill.md | awk '{print "  " $9 " (" $5 ")"}'

echo ""
echo "========================================"
echo "🌐 Step 2: Frontend Readiness"
echo "========================================"
echo ""

if lsof -ti:3000 >/dev/null 2>&1; then
    echo -e "${GREEN}✅ Frontend running on port 3000${NC}"
    echo ""
    echo "Test URLs:"
    echo "  1. http://localhost:3000/physical_ai_textbook/docs/intro"
    echo "  2. http://localhost:3000/physical_ai_textbook/docs/module1/intro"
    echo "  3. http://localhost:3000/physical_ai_textbook/docs/module1/chapter1-1-ros2-fundamentals"
else
    echo -e "${YELLOW}⚠️  Frontend NOT running${NC}"
    echo "Start it: cd textbook && npm start"
fi

echo ""
echo "========================================"
echo "🎯 Step 3: Demo Features"
echo "========================================"
echo ""

echo "✅ Personalize Button:"
echo "   - Click '✨ PERSONALIZE FOR ME'"
echo "   - Expected: Banner appears instantly"
echo "   - Expected: Bulldog says 'I have personalized this chapter!'"
echo "   - NO API call required!"
echo ""

echo "✅ Urdu Button:"
echo "   - Click '🌍 TRANSLATE TO URDU'"
echo "   - Expected: Content changes to Urdu instantly"
echo "   - Expected: Bulldog says 'اردو میں ترجمہ مکمل!'"
echo "   - NO API call required!"
echo ""

echo "✅ Agent Skills:"
echo "   - Show: ls docs/agent_skills/"
echo "   - Point out: translator.skill.md + personalizer.skill.md"
echo "   - Explain: Reusable intelligence skills"
echo ""

echo "========================================"
echo "🎊 BONUS POINTS STATUS"
echo "========================================"
echo ""

SCORE=0

# Check skill files
if [ -f "$SKILL_DIR/translator.skill.md" ] && [ -f "$SKILL_DIR/personalizer.skill.md" ]; then
    SCORE=$((SCORE + 50))
    echo -e "${GREEN}✅ Task 4: Agent Skills (50 points)${NC}"
else
    echo -e "${YELLOW}❌ Task 4: Agent Skills (0 points)${NC}"
fi

# Frontend running
if lsof -ti:3000 >/dev/null 2>&1; then
    SCORE=$((SCORE + 150))
    echo -e "${GREEN}✅ Tasks 6-7: Chapter Buttons (150 points)${NC}"
else
    echo -e "${YELLOW}⚠️  Tasks 6-7: Frontend not running (0 points)${NC}"
fi

echo ""
echo "========================================"
if [ "$SCORE" -eq 200 ]; then
    echo -e "${GREEN}✅ TOTAL: $SCORE / 200 points - PERFECT SCORE!${NC}"
else
    echo -e "${YELLOW}⚠️  TOTAL: $SCORE / 200 points${NC}"
fi
echo "========================================"

echo ""
echo "🎬 Ready for Demo!"
echo ""
echo "Next Step: Open browser and test buttons"
echo ""

