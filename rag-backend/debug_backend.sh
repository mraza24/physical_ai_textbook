#!/bin/bash
# RAG Backend Debugging Script
# Checks all common issues causing HTTP 500 errors

cd "$(dirname "$0")"

echo "================================================================="
echo "RAG Backend Debug - HTTP 500 Error Diagnosis"
echo "================================================================="
echo ""

# 1. Check .env file exists
echo "1️⃣  Checking .env file..."
if [ ! -f .env ]; then
    echo "   ❌ .env file not found!"
    echo "   💡 Solution: Copy .env.example to .env and fill in credentials"
    echo "   Command: cp .env.example .env"
    exit 1
else
    echo "   ✅ .env file exists"
fi
echo ""

# 2. Check environment variables are set
echo "2️⃣  Checking environment variables..."
python3 << 'EOF'
import os
from dotenv import load_dotenv

load_dotenv()

required = {
    "COHERE_API_KEY": os.getenv("COHERE_API_KEY"),
    "QDRANT_URL": os.getenv("QDRANT_URL"),
    "QDRANT_API_KEY": os.getenv("QDRANT_API_KEY")
}

missing = []
for key, value in required.items():
    if not value or value.startswith("your_"):
        print(f"   ❌ {key} is not set or still has placeholder value")
        missing.append(key)
    else:
        masked = value[:8] + "..." if len(value) > 8 else "***"
        print(f"   ✅ {key} = {masked}")

if missing:
    print(f"\n   💡 Solution: Edit .env file and set: {', '.join(missing)}")
    exit(1)
EOF

if [ $? -ne 0 ]; then
    exit 1
fi
echo ""

# 3. Test Cohere API connection
echo "3️⃣  Testing Cohere API connection..."
python3 << 'EOF'
import sys
try:
    from app.services.cohere_service import CohereService
    from dotenv import load_dotenv

    load_dotenv()
    cohere = CohereService()

    print("   Testing API key validity...")
    is_valid = cohere.check_api_key()

    if is_valid:
        print("   ✅ Cohere API key is valid")
        print(f"   ✅ Embedding model: {cohere.embedding_model}")
        print(f"   ✅ Generation model: {cohere.generation_model}")
    else:
        print("   ❌ Cohere API key is invalid")
        print("   💡 Solution: Get new key from https://dashboard.cohere.com")
        sys.exit(1)

except ValueError as e:
    print(f"   ❌ Configuration error: {e}")
    sys.exit(1)
except Exception as e:
    print(f"   ❌ Cohere connection failed: {e}")
    print("   💡 Check your COHERE_API_KEY and internet connection")
    sys.exit(1)
EOF

if [ $? -ne 0 ]; then
    exit 1
fi
echo ""

# 4. Test Qdrant connection
echo "4️⃣  Testing Qdrant connection..."
python3 << 'EOF'
import sys
try:
    from app.services.qdrant_service import QdrantService
    from dotenv import load_dotenv

    load_dotenv()
    qdrant = QdrantService()

    print("   ✅ Connected to Qdrant")
    print(f"   ✅ Collection name: {qdrant.collection_name}")

except ValueError as e:
    print(f"   ❌ Configuration error: {e}")
    sys.exit(1)
except Exception as e:
    print(f"   ❌ Qdrant connection failed: {e}")
    print("   💡 Check QDRANT_URL and QDRANT_API_KEY in .env")
    sys.exit(1)
EOF

if [ $? -ne 0 ]; then
    exit 1
fi
echo ""

# 5. Check if collection exists and has data
echo "5️⃣  Checking Qdrant collection status..."
python3 << 'EOF'
import sys
try:
    from app.services.qdrant_service import QdrantService
    from dotenv import load_dotenv

    load_dotenv()
    qdrant = QdrantService()

    try:
        info = qdrant.get_collection_info()
        print(f"   ✅ Collection '{info['name']}' exists")
        print(f"   📊 Points count: {info['points_count']}")
        print(f"   📊 Vectors count: {info['vectors_count']}")
        print(f"   📊 Status: {info['status']}")

        if info['points_count'] == 0:
            print("\n   ⚠️  WARNING: Collection is EMPTY!")
            print("   💡 Solution: Run data ingestion to populate the collection")
            print("   Command: python ingest.py")
            sys.exit(2)  # Exit code 2 = warning (not fatal)

    except Exception as e:
        if "not found" in str(e).lower():
            print(f"   ❌ Collection 'textbook_chunks' does not exist")
            print("   💡 Solution: Create collection with vector_size=1024")
            print("   Options:")
            print("      1. Via Qdrant Dashboard: https://cloud.qdrant.io")
            print("      2. Via Python:")
            print("         python3 -c \"from app.services.qdrant_service import QdrantService; from dotenv import load_dotenv; load_dotenv(); QdrantService().create_collection(vector_size=1024)\"")
            sys.exit(1)
        else:
            raise

except Exception as e:
    print(f"   ❌ Error checking collection: {e}")
    sys.exit(1)
EOF

COLLECTION_STATUS=$?
echo ""

# 6. Test embedding generation
echo "6️⃣  Testing embedding generation..."
python3 << 'EOF'
import sys
try:
    from app.services.cohere_service import CohereService
    from dotenv import load_dotenv

    load_dotenv()
    cohere = CohereService()

    test_text = "What are humanoid robots?"
    print(f"   Generating embedding for: '{test_text}'")

    embedding = cohere.embed_text(test_text, input_type="search_query")

    print(f"   ✅ Embedding generated successfully")
    print(f"   ✅ Vector dimension: {len(embedding)}")

    if len(embedding) != 1024:
        print(f"   ⚠️  WARNING: Expected 1024 dimensions, got {len(embedding)}")
        print("   💡 This may cause dimension mismatch with Qdrant")

except Exception as e:
    print(f"   ❌ Embedding generation failed: {e}")
    sys.exit(1)
EOF

if [ $? -ne 0 ]; then
    exit 1
fi
echo ""

# 7. Summary and recommendations
echo "================================================================="
echo "✅ DIAGNOSIS COMPLETE"
echo "================================================================="
echo ""

if [ $COLLECTION_STATUS -eq 2 ]; then
    echo "⚠️  Your collection exists but is EMPTY!"
    echo ""
    echo "Next Steps:"
    echo "1. Run data ingestion:"
    echo "   python ingest.py"
    echo ""
    echo "2. Verify ingestion succeeded:"
    echo "   Check Qdrant dashboard: https://cloud.qdrant.io"
    echo "   Or run this script again to see points_count > 0"
    echo ""
elif [ $COLLECTION_STATUS -eq 0 ]; then
    echo "✅ All checks passed!"
    echo ""
    echo "Your backend should be working. If you still get HTTP 500:"
    echo ""
    echo "1. Check Render environment variables:"
    echo "   Dashboard → Your Service → Environment"
    echo "   Ensure COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY are set"
    echo ""
    echo "2. Check Render logs:"
    echo "   Dashboard → Your Service → Logs"
    echo "   Look for specific error messages when you send a query"
    echo ""
    echo "3. Test your backend locally:"
    echo "   uvicorn app.main:app --reload --port 8000"
    echo "   Then: curl -X POST http://localhost:8000/api/query -H 'Content-Type: application/json' -d '{\"query_text\":\"test\"}'"
    echo ""
fi

echo "For detailed debugging guide, see: DEBUG_HTTP_500.md"
echo "================================================================="
