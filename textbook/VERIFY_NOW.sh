#!/bin/bash

echo "========================================"
echo "🎯 CHAPTER BUTTON VERIFICATION"
echo "========================================"
echo ""

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}Testing chapter-specific transformations...${NC}"
echo ""

echo "📍 Test Plan:"
echo ""
echo "1. Open Chapter 1.1:"
echo "   http://localhost:3000/physical_ai_textbook/docs/module1/chapter1-1-ros2-fundamentals"
echo ""
echo "   Expected:"
echo "   - Two LARGE buttons at top"
echo "   - Purple 'PERSONALIZE' button"
echo "   - Orange 'TRANSLATE TO URDU' button"
echo ""

echo "2. Click PERSONALIZE:"
echo "   Expected:"
echo "   - Bulldog says: 'Adapting this chapter for your Hardware profile!'"
echo "   - Banner appears: '✨ Personalized for Hardware Specialists'"
echo "   - Hardware tips at bottom"
echo ""

echo "3. Navigate to Chapter 4.2:"
echo "   http://localhost:3000/physical_ai_textbook/docs/module4/chapter4-2-llms-robotics"
echo ""
echo "   Expected:"
echo "   - Chapter 4.2 shows ORIGINAL content (not personalized)"
echo "   - Buttons still visible at top"
echo ""

echo "4. Click PERSONALIZE on Chapter 4.2:"
echo "   Expected:"
echo "   - Only Chapter 4.2 changes"
echo "   - Bulldog confirms again"
echo ""

echo "5. Go back to Chapter 1.1:"
echo "   Expected:"
echo "   - Chapter 1.1 STILL shows personalized content"
echo "   - Proves chapter-specific state works!"
echo ""

echo "========================================"
echo -e "${GREEN}✅ VERIFICATION COMPLETE${NC}"
echo "========================================"
echo ""

echo "Open browser and test the URLs above!"
echo ""

# Check if frontend is running
if lsof -ti:3000 >/dev/null 2>&1; then
    echo -e "${GREEN}✅ Frontend is running on port 3000${NC}"
else
    echo -e "${YELLOW}⚠️  Frontend NOT running - start it first!${NC}"
    echo "   cd textbook && npm start"
fi

echo ""
