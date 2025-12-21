#!/bin/bash
# Configure Vercel Frontend with Backend URL
# Usage: ./configure-vercel.sh <backend-url>

set -e  # Exit on error

echo "======================================================"
echo "  Configure Vercel Frontend for Production"
echo "======================================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if backend URL is provided
if [ -z "$1" ]; then
    echo -e "${RED}Error: Backend URL not provided${NC}"
    echo ""
    echo "Usage: ./configure-vercel.sh <backend-url>"
    echo ""
    echo "Example:"
    echo "  ./configure-vercel.sh https://rag-chatbot-api.onrender.com"
    echo "  ./configure-vercel.sh https://your-app.railway.app"
    echo ""
    exit 1
fi

BACKEND_URL=$1

# Remove trailing slash if present
BACKEND_URL=${BACKEND_URL%/}

echo "Backend URL: $BACKEND_URL"
echo ""

# Validate URL format
if [[ ! $BACKEND_URL =~ ^https?:// ]]; then
    echo -e "${RED}Error: Invalid URL format${NC}"
    echo "URL must start with http:// or https://"
    exit 1
fi

echo -e "${YELLOW}Step 1: Testing backend health...${NC}"

if curl -f "$BACKEND_URL/api/health" > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Backend health check passed${NC}"
    curl "$BACKEND_URL/api/health"
    echo ""
else
    echo -e "${RED}✗ Backend health check failed${NC}"
    echo "Cannot reach $BACKEND_URL/api/health"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
echo -e "${YELLOW}Step 2: Checking Vercel CLI...${NC}"

# Check if Vercel CLI is installed
if ! command -v vercel &> /dev/null; then
    echo "Vercel CLI not found. Installing..."
    npm install -g vercel
    echo -e "${GREEN}✓ Vercel CLI installed${NC}"
else
    echo -e "${GREEN}✓ Vercel CLI already installed${NC}"
fi

echo ""
echo -e "${YELLOW}Step 3: Configuring Vercel environment variable...${NC}"

# Navigate to frontend directory
cd ../textbook

# Set environment variable for production
echo "Setting REACT_APP_API_URL for production..."
vercel env add REACT_APP_API_URL production <<EOF
$BACKEND_URL
EOF

# Set for preview
echo "Setting REACT_APP_API_URL for preview..."
vercel env add REACT_APP_API_URL preview <<EOF
$BACKEND_URL
EOF

# Set for development (optional)
echo "Setting REACT_APP_API_URL for development..."
vercel env add REACT_APP_API_URL development <<EOF
$BACKEND_URL
EOF

echo -e "${GREEN}✓ Environment variables configured${NC}"

echo ""
echo -e "${YELLOW}Step 4: Updating local .env.local...${NC}"

# Update local .env.local for reference
cat > .env.local << EOF
# RAG Backend API URL
# For production, this is set in Vercel Dashboard
REACT_APP_API_URL=$BACKEND_URL
EOF

echo -e "${GREEN}✓ Updated .env.local${NC}"

echo ""
echo -e "${YELLOW}Step 5: Deploying to Vercel...${NC}"

read -p "Deploy to production now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    vercel --prod
    echo -e "${GREEN}✓ Deployment triggered${NC}"
else
    echo "Skipping deployment. You can deploy later with: vercel --prod"
fi

echo ""
echo -e "${GREEN}======================================================"
echo "  Vercel Configuration Complete"
echo "======================================================${NC}"
echo ""
echo "Configuration Summary:"
echo "  Backend URL: $BACKEND_URL"
echo "  Frontend URL: https://physical-ai-textbook-jet.vercel.app"
echo "  Environment Variable: REACT_APP_API_URL=$BACKEND_URL"
echo ""
echo "Next steps:"
echo "1. Wait for Vercel deployment to complete"
echo "2. Open: https://physical-ai-textbook-jet.vercel.app"
echo "3. Test chatbot functionality"
echo "4. Verify no CORS errors in browser console"
echo ""
echo "Troubleshooting:"
echo "  - View Vercel logs: vercel logs"
echo "  - List env vars: vercel env ls"
echo "  - Remove env var: vercel env rm REACT_APP_API_URL production"
echo ""

cd ../rag-backend  # Return to backend directory
