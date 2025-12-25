# RAG Chatbot Backend - Tests

## Overview

This directory contains tests for the RAG Chatbot backend API.

## Test Structure

```
tests/
├── __init__.py          - Test package initialization
├── test_api.py          - API endpoint tests
└── README.md            - This file
```

## Running Tests

### Setup

1. **Install test dependencies:**
   ```bash
   pip install pytest pytest-asyncio httpx
   ```

2. **Configure environment:**
   - Ensure `.env` file is configured with valid credentials
   - For unit tests (no external services), credentials not required
   - For integration tests, Qdrant and Cohere must be accessible

### Run Tests

**All tests:**
```bash
pytest tests/
```

**Verbose output:**
```bash
pytest tests/ -v
```

**Specific test file:**
```bash
pytest tests/test_api.py -v
```

**Specific test class:**
```bash
pytest tests/test_api.py::TestRootEndpoint -v
```

**Specific test:**
```bash
pytest tests/test_api.py::TestRootEndpoint::test_root_returns_200 -v
```

**With coverage:**
```bash
pytest tests/ --cov=app
pytest tests/ --cov=app --cov-report=html  # HTML report
```

**Skip integration tests:**
```bash
pytest tests/ -m "not integration"
```

## Test Categories

### Unit Tests

Tests that don't require external services:
- `TestRootEndpoint` - Root endpoint tests
- `TestHealthEndpoint` - Health check tests
- `TestOpenAPISchema` - API documentation tests

**Run with:** `pytest tests/test_api.py::TestRootEndpoint -v`

### Integration Tests

Tests that require full setup (Qdrant + Cohere):
- `TestIntegration` - End-to-end query tests
- `TestQueryEndpoint` - Query endpoint with real data

**Run with:** `pytest tests/test_api.py::TestIntegration -v`

**Note:** Integration tests are skipped by default. Remove `@pytest.mark.skip` decorator to enable.

## Writing New Tests

### Example Test

```python
import pytest
from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)

class TestMyEndpoint:
    """Test my new endpoint"""

    def test_endpoint_returns_200(self):
        """Test that endpoint returns 200 OK"""
        response = client.get("/api/my-endpoint")
        assert response.status_code == 200

    def test_endpoint_returns_json(self):
        """Test that endpoint returns JSON"""
        response = client.get("/api/my-endpoint")
        data = response.json()
        assert "key" in data
```

### Fixtures

Use fixtures for shared setup:

```python
@pytest.fixture
def mock_qdrant_service():
    """Mock Qdrant service for testing"""
    # Return mock service
    pass

def test_with_mock(mock_qdrant_service):
    """Test using mock service"""
    # Test code here
    pass
```

### Mocking External Services

For testing without Cohere/Qdrant:

```python
from unittest.mock import patch

@patch('app.services.cohere_service.CohereService')
def test_query_with_mock(mock_cohere):
    """Test query with mocked Cohere"""
    mock_cohere.return_value.embed_text.return_value = [0.1, 0.2, 0.3]
    # Test code
```

## Continuous Integration

### GitHub Actions

Add to `.github/workflows/test.yml`:

```yaml
name: Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest

    steps:
    - uses: actions/checkout@v3

    - name: Set up Python
      uses: actions/setup-python@v4
      with:
        python-version: '3.12'

    - name: Install dependencies
      run: |
        pip install -r requirements.txt
        pip install pytest pytest-asyncio httpx pytest-cov

    - name: Run tests
      run: pytest tests/ -v --cov=app

    - name: Upload coverage
      uses: codecov/codecov-action@v3
```

## Best Practices

1. **Test naming:** Use descriptive names (`test_query_rejects_empty_string`)
2. **One assertion per test:** Focus tests on single behaviors
3. **Use fixtures:** Share common setup code
4. **Mock external services:** Don't hit real APIs in unit tests
5. **Document tests:** Add docstrings explaining what's being tested
6. **Test edge cases:** Empty strings, null values, max lengths
7. **Use parametrize:** Test multiple inputs efficiently

## Example: Parametrized Tests

```python
@pytest.mark.parametrize("query_text,expected_status", [
    ("", 422),  # Empty
    ("x" * 501, 422),  # Too long
    ("Valid query", 200),  # Valid
])
def test_query_validation(query_text, expected_status):
    """Test query validation with multiple inputs"""
    response = client.post(
        "/api/query",
        json={"query_text": query_text, "selected_text": None}
    )
    assert response.status_code == expected_status
```

## Coverage Goals

- **Aim for >80% code coverage** for critical paths
- **100% coverage** for API endpoints
- **Focus on:**
  - Happy path (valid inputs)
  - Error cases (invalid inputs)
  - Edge cases (empty, null, max values)
  - Authentication/authorization
  - Error handling

## Troubleshooting

### Import Errors

If you get import errors:

```bash
# Add parent directory to Python path
export PYTHONPATH="${PYTHONPATH}:$(pwd)"

# Or use pytest's path resolution
pytest --import-mode=importlib tests/
```

### Environment Variables

If tests fail due to missing env vars:

```bash
# Create test .env file
cp .env.example .env.test

# Run with test env
ENV_FILE=.env.test pytest tests/
```

### Async Tests

For testing async endpoints:

```python
import pytest

@pytest.mark.asyncio
async def test_async_endpoint():
    """Test async endpoint"""
    async with AsyncClient(app=app, base_url="http://test") as ac:
        response = await ac.get("/api/endpoint")
    assert response.status_code == 200
```

## Resources

- [Pytest Documentation](https://docs.pytest.org/)
- [FastAPI Testing](https://fastapi.tiangolo.com/tutorial/testing/)
- [Pytest Fixtures](https://docs.pytest.org/en/stable/fixture.html)
- [Pytest Parametrize](https://docs.pytest.org/en/stable/how-to/parametrize.html)

---

**Note:** This is a basic test suite. For production, add more comprehensive tests covering:
- Authentication/authorization
- Rate limiting
- Database operations
- External service failures
- Concurrency
- Performance benchmarks
