# API Contracts Reference

The complete OpenAPI 3.0 specifications for all API endpoints are embedded in:

**`../plan.md`** (Lines 358-591)

This includes:

1. **Authentication API** (`auth.openapi.yaml`)
   - POST `/api/auth/signup`
   - POST `/api/auth/signin`
   - GET `/api/auth/profile`
   - PUT `/api/auth/profile`

2. **Personalization API** (`personalize.openapi.yaml`)
   - POST `/api/personalize`

3. **Translation API** (`translate.openapi.yaml`)
   - POST `/api/translate`

## Why Embedded in plan.md?

To avoid duplication and ensure single source of truth during planning phase. Extract to standalone `.yaml` files during implementation (Task phase).

## Extraction Command

When implementing, extract contracts from plan.md:

```bash
# Extract auth contract
sed -n '/#### File: `auth.openapi.yaml`/,/^#### File:/p' ../plan.md | \
  sed '1d;$d' | sed 's/^```yaml//;s/^```//' > auth.openapi.yaml

# Extract personalize contract
sed -n '/#### File: `personalize.openapi.yaml`/,/^#### File:/p' ../plan.md | \
  sed '1d;$d' | sed 's/^```yaml//;s/^```//' > personalize.openapi.yaml

# Extract translate contract
sed -n '/#### File: `translate.openapi.yaml`/,/^### 3\. Quickstart Guide/p' ../plan.md | \
  sed '1d;$d' | sed 's/^```yaml//;s/^```//' > translate.openapi.yaml
```

---

**Status**: Contracts defined in plan.md
**Next Step**: Extract to standalone files during `/sp.tasks` implementation phase
