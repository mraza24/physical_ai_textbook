#!/bin/bash
set -e

echo "Starting deployment to GitHub Pages..."
echo ""

# Build the project
echo "[1/3] Building Docusaurus project..."
npm run build

# Deploy to GitHub Pages
echo ""
echo "[2/3] Deploying to GitHub Pages..."
GIT_USER=mraza24 npm run deploy

echo ""
echo "[3/3] Deployment complete!"
echo ""
echo "Your site will be available at:"
echo "https://mraza24.github.io/physical_ai_textbook/"
echo ""
echo "Note: It may take a few minutes for GitHub Pages to update."
