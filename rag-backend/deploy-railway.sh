#!/bin/bash
# RAG Chatbot Backend Deployment Script for Railway.app
# Usage: ./deploy-railway.sh

set -e  # Exit on error

echo "======================================================"
echo "  RAG Chatbot Backend - Railway.app Deployment"
echo "======================================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if .env file exists
if [ ! -f .env ]; then
    echo -e "${RED}Error: .env file not found${NC}"
    echo "Please create .env file with required variables before deploying"
    exit 1
fi

echo -e "${YELLOW}Step 1: Checking Railway CLI...${NC}"

# Check if Railway CLI is installed
if ! command -v railway &> /dev/null; then
    echo "Railway CLI not found. Installing..."
    npm install -g @railway/cli
    echo -e "${GREEN}✓ Railway CLI installed${NC}"
else
    echo -e "${GREEN}✓ Railway CLI already installed${NC}"
fi

echo ""
echo -e "${YELLOW}Step 2: Login to Railway...${NC}"
railway login

echo ""
echo -e "${YELLOW}Step 3: Initializing Railway project...${NC}"

# Check if railway.json exists
if [ ! -f railway.json ]; then
    echo "Creating railway.json configuration..."
    cat > railway.json << 'EOF'
{
  "$schema": "https://railway.app/railway.schema.json",
  "build": {
    "builder": "NIXPACKS"
  },
  "deploy": {
    "startCommand": "uvicorn app.main:app --host 0.0.0.0 --port $PORT",
    "restartPolicyType": "ON_FAILURE",
    "restartPolicyMaxRetries": 10
  }
}
EOF
    echo -e "${GREEN}✓ Created railway.json${NC}"
fi

# Initialize Railway project if not already initialized
if [ ! -f railway.toml ]; then
    railway init
    echo -e "${GREEN}✓ Railway project initialized${NC}"
else
    echo -e "${GREEN}✓ Railway project already initialized${NC}"
fi

echo ""
echo -e "${YELLOW}Step 4: Setting environment variables...${NC}"

# Function to set Railway environment variable
set_railway_var() {
    local key=$1
    local value=$2
    echo "Setting $key..."
    railway variables set "$key=$value" 2>/dev/null || echo "  (already set or failed)"
}

# Read from .env and set variables
echo "Reading variables from .env..."
while IFS='=' read -r key value; do
    # Skip comments and empty lines
    [[ $key =~ ^#.*$ ]] && continue
    [[ -z $key ]] && continue

    # Remove leading/trailing whitespace
    key=$(echo "$key" | xargs)
    value=$(echo "$value" | xargs)

    # Skip if value is empty
    [[ -z $value ]] && continue

    set_railway_var "$key" "$value"
done < .env

# Set additional production-specific variables
echo ""
echo "Setting production-specific variables..."
set_railway_var "REQUEST_TIMEOUT_SECONDS" "300"
set_railway_var "ALLOWED_ORIGINS" "https://physical-ai-textbook-jet.vercel.app,http://localhost:3000"

echo -e "${GREEN}✓ Environment variables configured${NC}"

echo ""
echo -e "${YELLOW}Step 5: Deploying to Railway...${NC}"
railway up

echo ""
echo -e "${YELLOW}Step 6: Getting deployment URL...${NC}"
RAILWAY_URL=$(railway domain 2>/dev/null || echo "")

if [ -z "$RAILWAY_URL" ]; then
    echo "Creating public domain..."
    railway domain
    RAILWAY_URL=$(railway domain 2>/dev/null || echo "")
fi

echo ""
echo -e "${GREEN}✓ Deployment complete!${NC}"
echo ""
echo "Your backend is deployed at:"
echo -e "${GREEN}https://$RAILWAY_URL${NC}"
echo ""

echo ""
echo -e "${YELLOW}Step 7: Testing deployment...${NC}"
echo "Testing health endpoint..."

sleep 10  # Wait for deployment to be ready

if curl -f "https://$RAILWAY_URL/api/health" > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Health check passed${NC}"
    curl "https://$RAILWAY_URL/api/health"
else
    echo -e "${RED}✗ Health check failed${NC}"
    echo "Check Railway logs: railway logs"
fi

echo ""
echo -e "${YELLOW}Step 8: Next Steps${NC}"
echo ""
echo "1. Update Vercel frontend environment variable:"
echo "   REACT_APP_API_URL=https://$RAILWAY_URL"
echo ""
echo "2. Configure in Vercel Dashboard:"
echo "   https://vercel.com/dashboard"
echo "   → Settings → Environment Variables"
echo "   → Add REACT_APP_API_URL"
echo "   → Redeploy frontend"
echo ""
echo "3. Test end-to-end integration:"
echo "   Open: https://physical-ai-textbook-jet.vercel.app"
echo "   Try a chatbot query"
echo ""

echo ""
echo -e "${GREEN}======================================================"
echo "  Railway Deployment Complete"
echo "======================================================${NC}"
echo ""
echo "Useful Railway commands:"
echo "  railway logs          - View deployment logs"
echo "  railway status        - Check service status"
echo "  railway variables     - List environment variables"
echo "  railway domain        - Show deployment URL"
echo "  railway open          - Open Railway dashboard"
echo ""
