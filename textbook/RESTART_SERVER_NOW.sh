#!/bin/bash

echo "🔄 Restarting Docusaurus Dev Server..."
echo "========================================"

# Kill existing process on port 3000
if lsof -ti:3000 >/dev/null 2>&1; then
    echo "🛑 Stopping current dev server on port 3000..."
    kill -9 $(lsof -ti:3000)
    sleep 2
    echo "✅ Server stopped"
else
    echo "ℹ️ No server running on port 3000"
fi

echo ""
echo "🚀 Starting fresh dev server..."
echo "========================================"

cd /mnt/d/Q4_hackathon1/physical_ai_textbook/textbook

# Start dev server in background
npm start &

echo ""
echo "⏳ Waiting for server to start (30 seconds)..."
sleep 30

echo ""
echo "========================================"
echo "✅ Server should be ready!"
echo "========================================"
echo ""
echo "📱 Open these URLs to test:"
echo ""
echo "1. Homepage:"
echo "   http://localhost:3000/physical_ai_textbook/"
echo ""
echo "2. Intro page (BUTTONS SHOULD BE HERE):"
echo "   http://localhost:3000/physical_ai_textbook/docs/intro"
echo ""
echo "3. Module 1 Chapter 1.1 (BUTTONS SHOULD BE HERE):"
echo "   http://localhost:3000/physical_ai_textbook/docs/module1/chapter1-1-ros2-fundamentals"
echo ""
echo "🔍 What to look for:"
echo "   - TWO LARGE buttons at the TOP of every chapter page"
echo "   - 🌍 TRANSLATE TO URDU (bright orange, pulsing)"
echo "   - ✨ PERSONALIZE FOR ME (purple gradient)"
echo ""
echo "⚠️ If buttons still don't show:"
echo "   1. Hard refresh: Ctrl+Shift+R (Windows) or Cmd+Shift+R (Mac)"
echo "   2. Clear browser cache completely"
echo "   3. Try incognito/private mode"
echo "   4. Check browser console (F12) for errors"
echo ""
echo "========================================"
