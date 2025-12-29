#!/usr/bin/env python3
"""
Qdrant Connection Test Script
Tests Qdrant URL and API key configuration to diagnose 403 errors
"""

import os
import sys
from dotenv import load_dotenv


def test_env_variables():
    """Test if environment variables are set correctly"""
    print("=" * 70)
    print("Test 1: Environment Variables")
    print("=" * 70)

    load_dotenv()

    url = os.getenv("QDRANT_URL")
    key = os.getenv("QDRANT_API_KEY")

    if not url:
        print("❌ QDRANT_URL is not set in .env")
        print("💡 Add: QDRANT_URL=https://your-cluster.qdrant.io")
        return False

    if not key:
        print("❌ QDRANT_API_KEY is not set in .env")
        print("💡 Add: QDRANT_API_KEY=your-api-key-here")
        return False

    # Check for common mistakes
    if url.endswith("/dashboard"):
        print(f"⚠️  URL has /dashboard suffix: {url}")
        print("💡 This will be auto-sanitized, but update .env to:")
        print(f"   QDRANT_URL={url.replace('/dashboard', '')}")

    if url.endswith("/"):
        print(f"⚠️  URL has trailing slash: {url}")
        print("💡 This will be auto-sanitized")

    # Show masked credentials
    masked_url = url
    masked_key = key[:8] + "..." if len(key) > 8 else "***"

    print(f"✅ QDRANT_URL: {masked_url}")
    print(f"✅ QDRANT_API_KEY: {masked_key}")
    print()
    return True


def test_qdrant_service():
    """Test Qdrant service initialization"""
    print("=" * 70)
    print("Test 2: Qdrant Service Initialization")
    print("=" * 70)

    try:
        from app.services.qdrant_service import QdrantService

        qdrant = QdrantService()
        print("\n✅ Qdrant service initialized successfully!")
        print()
        return True

    except ValueError as e:
        print(f"\n❌ Configuration error: {e}")
        print()
        return False

    except Exception as e:
        print(f"\n❌ Connection error: {e}")
        print()
        if "403" in str(e) or "Forbidden" in str(e):
            print("💡 This is a 403 Forbidden error.")
            print("   Check the detailed error messages above for guidance.")
        print()
        return False


def test_collection_access():
    """Test collection access"""
    print("=" * 70)
    print("Test 3: Collection Access Test")
    print("=" * 70)

    try:
        from app.services.qdrant_service import QdrantService
        from dotenv import load_dotenv

        load_dotenv()
        qdrant = QdrantService()

        # Try to get collections (authentication test)
        print("🔍 Fetching collections from Qdrant...")
        collections = qdrant.client.get_collections()

        print(f"✅ Successfully connected to Qdrant!")
        print(f"📊 Found {len(collections.collections)} existing collections")

        if collections.collections:
            print("\nExisting collections:")
            for col in collections.collections:
                print(f"  - {col.name}")
        print()
        return True

    except Exception as e:
        print(f"❌ Failed to access collections: {e}")
        print()
        if "403" in str(e) or "Forbidden" in str(e):
            print("💡 403 Forbidden - Your API key is being rejected.")
            print("   Follow the steps in the error message above to fix.")
        print()
        return False


def main():
    """Run all tests"""
    print("\n" + "=" * 70)
    print("QDRANT CONNECTION DIAGNOSTIC TOOL")
    print("=" * 70)
    print()

    # Test 1: Environment variables
    env_ok = test_env_variables()
    if not env_ok:
        print("❌ Environment variables test failed.")
        print("💡 Fix your .env file before proceeding.")
        sys.exit(1)

    # Test 2: Service initialization
    service_ok = test_qdrant_service()
    if not service_ok:
        print("❌ Service initialization failed.")
        print("💡 Check the error messages above for details.")
        sys.exit(1)

    # Test 3: Collection access
    collection_ok = test_collection_access()
    if not collection_ok:
        print("❌ Collection access test failed.")
        print("💡 Your credentials may be invalid or cluster may be paused.")
        sys.exit(1)

    # All tests passed
    print("=" * 70)
    print("✅ ALL TESTS PASSED!")
    print("=" * 70)
    print()
    print("Your Qdrant configuration is correct.")
    print("You can now run: python ingest.py")
    print()


if __name__ == "__main__":
    main()
