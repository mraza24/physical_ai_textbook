#!/usr/bin/env python3
"""
Simple server test script to verify FastAPI app starts correctly.
"""
import sys
import time
import subprocess
import requests

def test_server():
    """Test if the FastAPI server starts and responds to health checks."""
    print("Starting FastAPI server...")

    # Start uvicorn server
    process = subprocess.Popen(
        ["./venv/bin/uvicorn", "app.main:app", "--host", "127.0.0.1", "--port", "8003"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # Wait for server to start
    print("Waiting for server to start...")
    time.sleep(3)

    try:
        # Test root endpoint
        print("\nTesting root endpoint (/)...")
        response = requests.get("http://127.0.0.1:8003/")
        print(f"Status: {response.status_code}")
        print(f"Response: {response.json()}")

        # Test health endpoint
        print("\nTesting health endpoint (/api/health)...")
        response = requests.get("http://127.0.0.1:8003/api/health")
        print(f"Status: {response.status_code}")
        print(f"Response: {response.json()}")

        print("\n✅ Server test PASSED!")
        return True

    except Exception as e:
        print(f"\n❌ Server test FAILED: {e}")
        return False

    finally:
        # Stop the server
        print("\nStopping server...")
        process.terminate()
        process.wait(timeout=5)
        print("Server stopped.")

if __name__ == "__main__":
    success = test_server()
    sys.exit(0 if success else 1)
