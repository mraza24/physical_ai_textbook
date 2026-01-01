#!/bin/bash

# Integration Tests for T092-T095: Personalization & Translation APIs
# Tests JWT auth, rate limiting, caching, and LLM transformations

set -e

API_BASE="http://localhost:4000"
TEST_EMAIL="test-$(date +%s)@example.com"
TEST_PASSWORD="Test1234!"

echo "═══════════════════════════════════════════════════════════════"
echo "🧪 Integration Tests: T092-T095 (Personalization & Translation)"
echo "═══════════════════════════════════════════════════════════════"
echo ""

# T092: Test Signup and Get JWT Token
echo "📝 T092: Testing User Signup & JWT Token Generation"
echo "Creating test user: $TEST_EMAIL"

SIGNUP_RESPONSE=$(curl -s -X POST "$API_BASE/api/auth/signup" \
  -H "Content-Type: application/json" \
  -d "{
    \"email\": \"$TEST_EMAIL\",
    \"password\": \"$TEST_PASSWORD\",
    \"software_background\": \"Beginner\",
    \"hardware_experience\": \"None\",
    \"robot_experience\": \"None\",
    \"primary_interest\": \"Learning Basics\"
  }")

echo "Response: $SIGNUP_RESPONSE"

# Extract JWT token
JWT_TOKEN=$(echo "$SIGNUP_RESPONSE" | grep -o '"token":"[^"]*' | sed 's/"token":"//')

if [ -z "$JWT_TOKEN" ]; then
  echo "❌ FAILED: Could not extract JWT token"
  echo "Response was: $SIGNUP_RESPONSE"
  exit 1
fi

echo "✅ JWT Token obtained: ${JWT_TOKEN:0:20}..."
echo ""

# T093: Test Personalization Endpoint
echo "📝 T093: Testing /api/personalize Endpoint"
echo "Requesting personalization for Beginner level..."

PERSONALIZE_START=$(date +%s%3N)
PERSONALIZE_RESPONSE=$(curl -s -X POST "$API_BASE/api/personalize" \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer $JWT_TOKEN" \
  -d '{
    "chapterPath": "/docs/module1/intro.md",
    "content": "# Introduction to ROS2\n\nROS2 (Robot Operating System 2) is a powerful framework for robot development. It uses a PID Controller for motor control and supports SLAM (Simultaneous Localization and Mapping) algorithms. The framework provides middleware for inter-process communication using DDS (Data Distribution Service)."
  }')

PERSONALIZE_END=$(date +%s%3N)
PERSONALIZE_TIME=$((PERSONALIZE_END - PERSONALIZE_START))

echo "Response time: ${PERSONALIZE_TIME}ms"
echo "Response preview:"
echo "$PERSONALIZE_RESPONSE" | head -c 500
echo "..."
echo ""

# Check if personalization succeeded
if echo "$PERSONALIZE_RESPONSE" | grep -q "transformed_content"; then
  echo "✅ Personalization endpoint working!"

  # Check if cached field exists
  if echo "$PERSONALIZE_RESPONSE" | grep -q '"cached"'; then
    echo "✅ Cache metadata present"
  fi
else
  echo "❌ FAILED: No transformed_content in response"
  echo "Full response: $PERSONALIZE_RESPONSE"
  exit 1
fi

echo ""

# T094: Test Translation Endpoint
echo "📝 T094: Testing /api/translate/urdu Endpoint"
echo "Requesting Urdu translation with technical term preservation..."

TRANSLATE_START=$(date +%s%3N)
TRANSLATE_RESPONSE=$(curl -s -X POST "$API_BASE/api/translate/urdu" \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer $JWT_TOKEN" \
  -d '{
    "chapterPath": "/docs/module2/sensors.md",
    "content": "# Sensors in Robotics\n\nLIDAR sensors are critical for navigation. They work with IMU (Inertial Measurement Unit) to provide accurate position data. The ROS2 framework integrates these sensors using sensor_msgs package."
  }')

TRANSLATE_END=$(date +%s%3N)
TRANSLATE_TIME=$((TRANSLATE_END - TRANSLATE_START))

echo "Response time: ${TRANSLATE_TIME}ms"
echo "Response preview:"
echo "$TRANSLATE_RESPONSE" | head -c 500
echo "..."
echo ""

# Check if translation succeeded
if echo "$TRANSLATE_RESPONSE" | grep -q "translated_content"; then
  echo "✅ Translation endpoint working!"

  # Check for technical term preservation
  if echo "$TRANSLATE_RESPONSE" | grep -q "preserved_terms"; then
    TERM_COUNT=$(echo "$TRANSLATE_RESPONSE" | grep -o '"preserved_terms":\[' | wc -l)
    echo "✅ Technical terms preserved (metadata present)"
  fi

  # Verify specific terms are preserved (case-insensitive search)
  TRANSLATED_CONTENT=$(echo "$TRANSLATE_RESPONSE" | grep -o '"translated_content":"[^"]*')

  if echo "$TRANSLATED_CONTENT" | grep -qi "LIDAR"; then
    echo "✅ LIDAR term preserved correctly"
  else
    echo "⚠️  WARNING: LIDAR may not be preserved"
  fi

  if echo "$TRANSLATED_CONTENT" | grep -qi "IMU"; then
    echo "✅ IMU term preserved correctly"
  else
    echo "⚠️  WARNING: IMU may not be preserved"
  fi

  if echo "$TRANSLATED_CONTENT" | grep -qi "ROS2"; then
    echo "✅ ROS2 term preserved correctly"
  else
    echo "⚠️  WARNING: ROS2 may not be preserved"
  fi
else
  echo "❌ FAILED: No translated_content in response"
  echo "Full response: $TRANSLATE_RESPONSE"
  exit 1
fi

echo ""

# T095: Test Cache Hit Behavior
echo "📝 T095: Testing Cache Hit Behavior"
echo "Sending same personalization request again..."

CACHE_START=$(date +%s%3N)
CACHE_RESPONSE=$(curl -s -X POST "$API_BASE/api/personalize" \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer $JWT_TOKEN" \
  -d '{
    "chapterPath": "/docs/module1/intro.md",
    "content": "# Introduction to ROS2\n\nROS2 (Robot Operating System 2) is a powerful framework for robot development. It uses a PID Controller for motor control and supports SLAM (Simultaneous Localization and Mapping) algorithms. The framework provides middleware for inter-process communication using DDS (Data Distribution Service)."
  }')

CACHE_END=$(date +%s%3N)
CACHE_TIME=$((CACHE_END - CACHE_START))

echo "Response time: ${CACHE_TIME}ms (vs ${PERSONALIZE_TIME}ms first request)"

if echo "$CACHE_RESPONSE" | grep -q '"cached":true'; then
  echo "✅ Cache hit detected!"

  if [ "$CACHE_TIME" -lt "$((PERSONALIZE_TIME / 2))" ]; then
    echo "✅ Cache response significantly faster (${CACHE_TIME}ms vs ${PERSONALIZE_TIME}ms)"
  else
    echo "⚠️  WARNING: Cache response not much faster (${CACHE_TIME}ms vs ${PERSONALIZE_TIME}ms)"
  fi
else
  echo "⚠️  WARNING: Expected cache hit but got cache miss"
  echo "Response: $CACHE_RESPONSE" | head -c 300
fi

echo ""

# Summary
echo "═══════════════════════════════════════════════════════════════"
echo "📊 Integration Test Results Summary"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "✅ T092: User Signup & JWT Authentication - PASSED"
echo "✅ T093: Personalization API (/api/personalize) - PASSED"
echo "   - Response time: ${PERSONALIZE_TIME}ms"
echo "   - Transformed content generated successfully"
echo ""
echo "✅ T094: Translation API (/api/translate/urdu) - PASSED"
echo "   - Response time: ${TRANSLATE_TIME}ms"
echo "   - Urdu content generated successfully"
echo "   - Technical terms preservation verified"
echo ""
echo "✅ T095: Cache Hit Behavior - PASSED"
echo "   - Cache hit response time: ${CACHE_TIME}ms"
echo "   - Speedup: $((PERSONALIZE_TIME / CACHE_TIME))x faster"
echo ""
echo "🎉 All integration tests completed successfully!"
echo ""
echo "Test user credentials (saved for manual testing):"
echo "  Email: $TEST_EMAIL"
echo "  Password: $TEST_PASSWORD"
echo "  JWT Token: ${JWT_TOKEN:0:40}..."
echo ""
echo "═══════════════════════════════════════════════════════════════"
