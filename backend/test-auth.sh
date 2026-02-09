#!/bin/bash

# ============================================================================
# Auth Route Direct API Test
# Tests POST /api/auth/signup endpoint directly with curl
# ============================================================================

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║              AUTH ROUTE DIRECT API TEST                        ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Backend URL
BACKEND_URL="http://localhost:4000"

# Test 1: Check if backend is running
echo "1️⃣  Checking if backend is running..."
if curl -s -f "$BACKEND_URL/health" > /dev/null 2>&1; then
    echo -e "${GREEN}✅ Backend is running on $BACKEND_URL${NC}"
else
    echo -e "${RED}❌ ERROR: Backend is not running on $BACKEND_URL${NC}"
    echo "   Start backend with: cd backend && npm run dev"
    exit 1
fi
echo ""

# Test 2: Check health endpoint
echo "2️⃣  Testing /health endpoint..."
HEALTH_RESPONSE=$(curl -s "$BACKEND_URL/health")
echo "   Response:"
echo "$HEALTH_RESPONSE" | jq '.' 2>/dev/null || echo "$HEALTH_RESPONSE"
echo ""

# Test 3: Test signup endpoint with valid data
echo "3️⃣  Testing POST /api/auth/signup with VALID data..."
TIMESTAMP=$(date +%s)
TEST_EMAIL="test-${TIMESTAMP}@example.com"

echo "   Email: $TEST_EMAIL"
echo "   Password: testpassword123"
echo "   Software Background: Beginner"
echo "   Hardware Experience: None"
echo ""
echo "   Sending request..."

SIGNUP_RESPONSE=$(curl -s -w "\n%{http_code}" -X POST "$BACKEND_URL/api/auth/signup" \
  -H "Content-Type: application/json" \
  -d "{
    \"email\": \"$TEST_EMAIL\",
    \"password\": \"testpassword123\",
    \"software_background\": \"Beginner\",
    \"hardware_experience\": \"None\",
    \"language_preference\": \"English\"
  }")

# Extract status code (last line)
HTTP_CODE=$(echo "$SIGNUP_RESPONSE" | tail -n 1)
RESPONSE_BODY=$(echo "$SIGNUP_RESPONSE" | sed '$d')

echo "   HTTP Status: $HTTP_CODE"
echo ""
echo "   Response Body:"
echo "$RESPONSE_BODY" | jq '.' 2>/dev/null || echo "$RESPONSE_BODY"
echo ""

# Analyze response
if [ "$HTTP_CODE" == "201" ]; then
    echo -e "${GREEN}✅ Signup successful! (HTTP 201 Created)${NC}"

    # Check if token is present
    if echo "$RESPONSE_BODY" | jq -e '.session.token' > /dev/null 2>&1; then
        TOKEN=$(echo "$RESPONSE_BODY" | jq -r '.session.token')
        echo -e "${GREEN}✅ JWT token received${NC}"
        echo "   Token (first 50 chars): ${TOKEN:0:50}..."
    else
        echo -e "${YELLOW}⚠️  No token in response${NC}"
    fi

    # Check if user is present
    if echo "$RESPONSE_BODY" | jq -e '.user' > /dev/null 2>&1; then
        USER_ID=$(echo "$RESPONSE_BODY" | jq -r '.user.id')
        USER_EMAIL=$(echo "$RESPONSE_BODY" | jq -r '.user.email')
        echo -e "${GREEN}✅ User created${NC}"
        echo "   User ID: $USER_ID"
        echo "   Email: $USER_EMAIL"
    fi

    # Check if profile is present
    if echo "$RESPONSE_BODY" | jq -e '.user.profile' > /dev/null 2>&1; then
        SOFTWARE=$(echo "$RESPONSE_BODY" | jq -r '.user.profile.software_background')
        HARDWARE=$(echo "$RESPONSE_BODY" | jq -r '.user.profile.hardware_experience')
        echo -e "${GREEN}✅ Profile created${NC}"
        echo "   Software: $SOFTWARE"
        echo "   Hardware: $HARDWARE"
    fi

elif [ "$HTTP_CODE" == "500" ]; then
    echo -e "${RED}❌ Internal Server Error (HTTP 500)${NC}"
    echo -e "${RED}   This is the error we're debugging!${NC}"
    echo ""
    echo "   Error details:"
    echo "$RESPONSE_BODY" | jq '.' 2>/dev/null || echo "$RESPONSE_BODY"
    echo ""
    echo "   Possible causes:"
    echo "   1. Database connection issue"
    echo "   2. Better-Auth configuration problem"
    echo "   3. Missing database tables (run migrations)"
    echo "   4. Schema mismatch (user table vs additionalFields)"

elif [ "$HTTP_CODE" == "400" ]; then
    echo -e "${YELLOW}⚠️  Bad Request (HTTP 400)${NC}"
    echo "   Error details:"
    echo "$RESPONSE_BODY" | jq '.' 2>/dev/null || echo "$RESPONSE_BODY"

elif [ "$HTTP_CODE" == "409" ]; then
    echo -e "${YELLOW}⚠️  Conflict (HTTP 409) - Email already exists${NC}"
    echo "   This is normal if you ran the test before"

else
    echo -e "${RED}❌ Unexpected status code: $HTTP_CODE${NC}"
    echo "   Response:"
    echo "$RESPONSE_BODY"
fi

echo ""

# Test 4: Test signup with INVALID data (missing required fields)
echo "4️⃣  Testing POST /api/auth/signup with MISSING fields..."
echo "   (Should return HTTP 400 Bad Request)"
echo ""

INVALID_RESPONSE=$(curl -s -w "\n%{http_code}" -X POST "$BACKEND_URL/api/auth/signup" \
  -H "Content-Type: application/json" \
  -d "{
    \"email\": \"test@example.com\",
    \"password\": \"test123\"
  }")

HTTP_CODE_INVALID=$(echo "$INVALID_RESPONSE" | tail -n 1)
RESPONSE_BODY_INVALID=$(echo "$INVALID_RESPONSE" | sed '$d')

echo "   HTTP Status: $HTTP_CODE_INVALID"
echo "   Response:"
echo "$RESPONSE_BODY_INVALID" | jq '.' 2>/dev/null || echo "$RESPONSE_BODY_INVALID"
echo ""

if [ "$HTTP_CODE_INVALID" == "400" ]; then
    echo -e "${GREEN}✅ Validation working correctly (HTTP 400 for invalid data)${NC}"
elif [ "$HTTP_CODE_INVALID" == "500" ]; then
    echo -e "${RED}❌ Server error on invalid data (should be 400, not 500)${NC}"
else
    echo -e "${YELLOW}⚠️  Unexpected status code: $HTTP_CODE_INVALID${NC}"
fi

echo ""

# Test 5: Test Better-Auth standard endpoint
echo "5️⃣  Testing Better-Auth standard endpoint: /api/auth/sign-in/email..."
echo "   (This is what login.tsx uses)"
echo ""

# First, try to sign in with the test user we created (if signup was successful)
if [ "$HTTP_CODE" == "201" ]; then
    LOGIN_RESPONSE=$(curl -s -w "\n%{http_code}" -X POST "$BACKEND_URL/api/auth/sign-in/email" \
      -H "Content-Type: application/json" \
      -d "{
        \"email\": \"$TEST_EMAIL\",
        \"password\": \"testpassword123\"
      }")

    HTTP_CODE_LOGIN=$(echo "$LOGIN_RESPONSE" | tail -n 1)
    RESPONSE_BODY_LOGIN=$(echo "$LOGIN_RESPONSE" | sed '$d')

    echo "   HTTP Status: $HTTP_CODE_LOGIN"
    echo "   Response:"
    echo "$RESPONSE_BODY_LOGIN" | jq '.' 2>/dev/null || echo "$RESPONSE_BODY_LOGIN"
    echo ""

    if [ "$HTTP_CODE_LOGIN" == "200" ]; then
        echo -e "${GREEN}✅ Login successful (HTTP 200)${NC}"
    elif [ "$HTTP_CODE_LOGIN" == "401" ]; then
        echo -e "${RED}❌ Login failed: Invalid credentials (HTTP 401)${NC}"
    elif [ "$HTTP_CODE_LOGIN" == "500" ]; then
        echo -e "${RED}❌ Login failed: Server error (HTTP 500)${NC}"
    else
        echo -e "${YELLOW}⚠️  Unexpected login status: $HTTP_CODE_LOGIN${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  Skipping login test (signup failed)${NC}"
fi

echo ""

# Summary
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                    TEST SUMMARY                                ║"
echo "╚════════════════════════════════════════════════════════════════╝"

if [ "$HTTP_CODE" == "201" ]; then
    echo -e "${GREEN}✅ Signup endpoint: WORKING${NC}"
    echo -e "${GREEN}✅ Database: ACCESSIBLE${NC}"
    echo -e "${GREEN}✅ Better-Auth: CONFIGURED CORRECTLY${NC}"
else
    echo -e "${RED}❌ Signup endpoint: FAILING (HTTP $HTTP_CODE)${NC}"
    echo -e "${RED}❌ This is the root cause of the frontend 500 error${NC}"
    echo ""
    echo "📝 Next steps:"
    echo "   1. Check backend terminal logs for detailed error"
    echo "   2. Run: npm run test:db (to verify database connection)"
    echo "   3. Run: npm run test:jwt (to verify JWT generation)"
    echo "   4. Check if database tables exist (run migrations)"
fi

echo ""
echo "🎉 Auth route test completed!"
echo ""
