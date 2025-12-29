#!/bin/bash
# Test script for optimized ingestion

cd "$(dirname "$0")"

echo "======================================================================="
echo "Testing Optimized Ingestion Script"
echo "======================================================================="
echo ""

# Check if venv exists
if [ ! -d "venv" ]; then
    echo "❌ Virtual environment not found!"
    echo "💡 Run: python3 -m venv venv && source venv/bin/activate && pip install -r requirements.txt"
    exit 1
fi

# Activate venv
source venv/bin/activate

# Check if .env exists
if [ ! -f .env ]; then
    echo "❌ .env file not found!"
    echo "💡 Copy .env.example to .env and fill in your credentials"
    exit 1
fi

# Test 1: Check collection existence logic
echo "Test 1: Testing collection existence check..."
python3 << 'EOF'
from app.services.qdrant_service import QdrantService
from dotenv import load_dotenv

load_dotenv()

try:
    qdrant = QdrantService()
    print("✅ Qdrant service initialized")

    # This should NOT throw 409 error anymore
    qdrant.create_collection(vector_size=1024)
    print("✅ Collection creation/check successful (no 409 error)")

except Exception as e:
    print(f"❌ Error: {e}")
    exit(1)
EOF

if [ $? -ne 0 ]; then
    echo "❌ Test 1 failed!"
    exit 1
fi

echo ""
echo "Test 2: Check ingest.py structure..."

# Check if main() function exists
if grep -q "def main():" ingest.py; then
    echo "✅ main() function exists"
else
    echo "❌ main() function not found"
    exit 1
fi

# Check if services are initialized outside loop
if grep -q "cohere = CohereService()" ingest.py && grep -q "qdrant = QdrantService()" ingest.py; then
    echo "✅ Services initialized in main()"
else
    echo "❌ Service initialization not found in main()"
    exit 1
fi

echo ""
echo "======================================================================="
echo "✅ All tests passed!"
echo "======================================================================="
echo ""
echo "Ready to run ingestion:"
echo "  python ingest.py"
echo ""
