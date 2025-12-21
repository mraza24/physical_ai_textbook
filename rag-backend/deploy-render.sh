#!/bin/bash
# RAG Chatbot Backend Deployment Script for Render.com
# Usage: ./deploy-render.sh

set -e  # Exit on error

echo "======================================================"
echo "  RAG Chatbot Backend - Render.com Deployment"
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

echo -e "${YELLOW}Step 1: Checking prerequisites...${NC}"

# Check if render.yaml exists, create if not
if [ ! -f render.yaml ]; then
    echo "Creating render.yaml configuration..."
    cat > render.yaml << 'EOF'
services:
  - type: web
    name: rag-chatbot-api
    env: python
    region: oregon  # Choose: oregon, frankfurt, singapore
    plan: free
    buildCommand: pip install -r requirements.txt
    startCommand: uvicorn app.main:app --host 0.0.0.0 --port $PORT
    envVars:
      - key: PYTHON_VERSION
        value: 3.11.0
      - key: REQUEST_TIMEOUT_SECONDS
        value: 300
      - key: SIMILARITY_THRESHOLD
        value: 0.7
      # Add other env vars via Render Dashboard after deployment
EOF
    echo -e "${GREEN}✓ Created render.yaml${NC}"
fi

echo ""
echo -e "${YELLOW}Step 2: Environment Variables Checklist${NC}"
echo "The following variables must be set in Render Dashboard after deployment:"
echo ""
echo "  Required:"
echo "  - COHERE_API_KEY=<your-cohere-api-key>"
echo "  - QDRANT_URL=<your-qdrant-cluster-url>"
echo "  - QDRANT_API_KEY=<your-qdrant-api-key>"
echo "  - DATABASE_URL=<your-neon-postgres-url>"
echo "  - ALLOWED_ORIGINS=https://physical-ai-textbook-jet.vercel.app,http://localhost:3000"
echo "  - ADMIN_API_KEY=<generate-secure-random-key>"
echo ""
echo "  Optional (already in render.yaml):"
echo "  - REQUEST_TIMEOUT_SECONDS=300"
echo "  - SIMILARITY_THRESHOLD=0.7"
echo ""

read -p "Have you created a Render account? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "Please create a Render account at: https://render.com"
    echo "Then run this script again."
    exit 1
fi

echo ""
echo -e "${YELLOW}Step 3: Deployment Instructions${NC}"
echo ""
echo "Manual Deployment Steps:"
echo ""
echo "1. Go to: https://dashboard.render.com/select-repo"
echo ""
echo "2. Connect your GitHub repository containing rag-backend/"
echo ""
echo "3. Configure the web service:"
echo "   - Name: rag-chatbot-api"
echo "   - Region: Oregon (or closest to your Qdrant cluster)"
echo "   - Branch: main (or your branch name)"
echo "   - Root Directory: rag-backend"
echo "   - Runtime: Python 3"
echo "   - Build Command: pip install -r requirements.txt"
echo "   - Start Command: uvicorn app.main:app --host 0.0.0.0 --port \$PORT"
echo ""
echo "4. Add Environment Variables (from checklist above)"
echo ""
echo "5. Click 'Create Web Service'"
echo ""
echo "6. Wait for deployment (5-10 minutes)"
echo ""
echo "7. Once deployed, note your URL: https://rag-chatbot-api.onrender.com"
echo ""
echo "8. Test health endpoint:"
echo "   curl https://rag-chatbot-api.onrender.com/api/health"
echo ""
echo -e "${GREEN}Expected response: {\"status\":\"healthy\",...}${NC}"
echo ""

echo ""
echo -e "${YELLOW}Step 4: Post-Deployment Configuration${NC}"
echo ""
echo "After backend is deployed, configure Vercel frontend:"
echo ""
echo "1. Go to: https://vercel.com/dashboard"
echo "2. Select your project: physical-ai-textbook-jet"
echo "3. Settings → Environment Variables"
echo "4. Add variable:"
echo "   Name: REACT_APP_API_URL"
echo "   Value: https://rag-chatbot-api.onrender.com"
echo "   Environments: Production, Preview, Development"
echo "5. Redeploy frontend (Deployments → Redeploy)"
echo ""

echo ""
echo -e "${GREEN}======================================================"
echo "  Deployment Instructions Complete"
echo "======================================================${NC}"
echo ""
echo "Next steps:"
echo "1. Follow manual deployment steps above"
echo "2. Configure environment variables in Render Dashboard"
echo "3. Test backend health endpoint"
echo "4. Configure Vercel frontend with backend URL"
echo "5. Test end-to-end integration"
echo ""
echo "For troubleshooting, see: DEPLOYMENT.md"
echo ""
