# API Contracts

This directory contains OpenAPI 3.0 specifications for the Authentication & Content Personalization System.

## Files

- **auth.openapi.yaml**: Authentication endpoints (signup, signin, profile management)
- **personalize.openapi.yaml**: Content personalization endpoint
- **translate.openapi.yaml**: Urdu translation endpoint

## Usage

### Validation
```bash
npx swagger-cli validate contracts/auth.openapi.yaml
npx swagger-cli validate contracts/personalize.openapi.yaml
npx swagger-cli validate contracts/translate.openapi.yaml
```

### Documentation Generation
```bash
npx redoc-cli bundle contracts/auth.openapi.yaml -o docs/api/auth.html
```

### Code Generation (TypeScript)
```bash
npx openapi-typescript contracts/auth.openapi.yaml --output src/types/auth-api.ts
```

## Base URL

- **Development**: `http://localhost:4000`
- **Production**: `https://api.physical-ai-textbook.com` (TBD)

## Authentication

All endpoints except `/api/auth/signup` and `/api/auth/signin` require Bearer token authentication:

```bash
curl -H "Authorization: Bearer <JWT_TOKEN>" https://api.example.com/api/personalize
```

## Rate Limits

- **Personalization/Translation**: 10 requests per user per minute
- **Authentication**: 5 requests per IP per minute (signup/signin)

## Error Codes

| Status | Description |
|--------|-------------|
| 200 | Success |
| 201 | Created (signup) |
| 400 | Validation error |
| 401 | Unauthorized (invalid/expired token) |
| 409 | Conflict (email exists) |
| 429 | Rate limit exceeded |
| 500 | Server error |
| 503 | Service unavailable (LLM API down) |

---

**Last Updated**: 2026-01-01
