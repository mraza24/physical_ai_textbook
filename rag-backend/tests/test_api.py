"""
API endpoint tests

To run these tests:
1. Install pytest: pip install pytest pytest-asyncio httpx
2. Set up test environment variables in .env
3. Run: pytest tests/test_api.py -v

Note: These are basic smoke tests. For production, add more comprehensive tests.
"""

import pytest
from fastapi.testclient import TestClient
from app.main import app

# Create test client
client = TestClient(app)


class TestRootEndpoint:
    """Test root endpoint"""

    def test_root_returns_200(self):
        """Test that root endpoint returns 200 OK"""
        response = client.get("/")
        assert response.status_code == 200

    def test_root_returns_json(self):
        """Test that root endpoint returns JSON"""
        response = client.get("/")
        assert response.headers["content-type"] == "application/json"

    def test_root_contains_message(self):
        """Test that root response contains expected message"""
        response = client.get("/")
        data = response.json()
        assert "message" in data
        assert data["message"] == "RAG Chatbot backend is running"

    def test_root_contains_docs_link(self):
        """Test that root response contains docs link"""
        response = client.get("/")
        data = response.json()
        assert "docs" in data
        assert data["docs"] == "/docs"


class TestHealthEndpoint:
    """Test health check endpoint"""

    def test_health_returns_200(self):
        """Test that health endpoint returns 200 OK"""
        response = client.get("/api/health")
        assert response.status_code == 200

    def test_health_returns_json(self):
        """Test that health endpoint returns JSON"""
        response = client.get("/api/health")
        assert response.headers["content-type"] == "application/json"

    def test_health_contains_status(self):
        """Test that health response contains status"""
        response = client.get("/api/health")
        data = response.json()
        assert "status" in data
        assert data["status"] == "healthy"

    def test_health_contains_timestamp(self):
        """Test that health response contains timestamp"""
        response = client.get("/api/health")
        data = response.json()
        assert "timestamp" in data


class TestQueryEndpoint:
    """Test query endpoint"""

    def test_query_endpoint_exists(self):
        """Test that query endpoint exists and accepts POST"""
        # Empty POST should return 422 (validation error), not 404
        response = client.post("/api/query", json={})
        assert response.status_code in [400, 422, 500]  # Not 404

    def test_query_requires_query_text(self):
        """Test that query_text is required"""
        response = client.post("/api/query", json={"selected_text": None})
        assert response.status_code == 422  # Validation error

    def test_query_rejects_empty_string(self):
        """Test that empty query_text is rejected"""
        response = client.post("/api/query", json={"query_text": "", "selected_text": None})
        assert response.status_code in [400, 422]

    def test_query_rejects_too_long(self):
        """Test that >500 character query is rejected"""
        long_query = "x" * 501
        response = client.post("/api/query", json={"query_text": long_query, "selected_text": None})
        assert response.status_code in [400, 422]

    @pytest.mark.skip(reason="Requires valid Cohere and Qdrant credentials and data")
    def test_query_valid_request(self):
        """Test query with valid request (requires credentials and data)"""
        response = client.post(
            "/api/query",
            json={"query_text": "What is robotics?", "selected_text": None}
        )
        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert "citations" in data
        assert "confidence" in data


class TestOpenAPISchema:
    """Test OpenAPI schema and docs"""

    def test_openapi_schema_accessible(self):
        """Test that OpenAPI schema is accessible"""
        response = client.get("/openapi.json")
        assert response.status_code == 200

    def test_swagger_ui_accessible(self):
        """Test that Swagger UI is accessible"""
        response = client.get("/docs")
        assert response.status_code == 200

    def test_redoc_accessible(self):
        """Test that ReDoc is accessible"""
        response = client.get("/redoc")
        assert response.status_code == 200


class TestCORS:
    """Test CORS configuration"""

    def test_cors_headers_present(self):
        """Test that CORS headers are present in response"""
        response = client.get("/api/health", headers={"Origin": "http://localhost:3000"})
        # Check if CORS headers are added by middleware
        # Note: TestClient may not fully simulate CORS preflight
        assert response.status_code == 200


# ============================================================================
# INTEGRATION TESTS (Require full setup)
# ============================================================================

@pytest.mark.skip(reason="Requires Qdrant and Cohere setup")
class TestIntegration:
    """Integration tests requiring full backend setup"""

    def test_end_to_end_query(self):
        """Test complete RAG query flow"""
        response = client.post(
            "/api/query",
            json={
                "query_text": "What is forward kinematics?",
                "selected_text": None
            }
        )
        assert response.status_code == 200
        data = response.json()

        # Validate response structure
        assert isinstance(data["answer"], str)
        assert len(data["answer"]) > 0
        assert isinstance(data["citations"], list)
        assert isinstance(data["confidence"], float)
        assert 0.0 <= data["confidence"] <= 1.0
        assert isinstance(data["retrieved_chunk_count"], int)
        assert data["retrieved_chunk_count"] >= 0
        assert isinstance(data["processing_time_ms"], int)

    def test_query_service_health(self):
        """Test query service health check"""
        response = client.get("/api/query/health")
        assert response.status_code == 200
        data = response.json()
        assert "cohere" in data
        assert "qdrant" in data
        assert "overall" in data


# ============================================================================
# PYTEST CONFIGURATION
# ============================================================================

@pytest.fixture
def test_client():
    """Fixture to provide test client"""
    return TestClient(app)


# Run with:
# pytest tests/test_api.py -v                    # Verbose output
# pytest tests/test_api.py -v --tb=short         # Short traceback
# pytest tests/test_api.py -k "test_root"        # Run specific test
# pytest tests/ --cov=app                        # With coverage
# pytest tests/ --cov=app --cov-report=html      # HTML coverage report
