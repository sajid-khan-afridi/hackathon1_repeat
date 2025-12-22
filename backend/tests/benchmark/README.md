# Authentication Performance Benchmarks

This directory contains performance testing scripts for authentication endpoints.

## Requirements

Performance tests require the backend server to be running and accessible.

### Install Dependencies

```bash
cd backend
pip install httpx  # For async HTTP client
```

## Running Performance Tests

### 1. Start Backend Server

```bash
cd backend
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

### 2. Run Performance Benchmark

In a separate terminal:

```bash
cd backend
python -m tests.benchmark.auth_performance_test
```

## Performance Requirements (FR-029)

All authentication endpoints must respond within **500ms p95 latency**:

| Endpoint | Method | Requirement |
|----------|--------|-------------|
| `/auth/signup` | POST | p95 < 500ms |
| `/auth/login` | POST | p95 < 500ms |
| `/auth/logout` | POST | p95 < 500ms |
| `/auth/refresh` | POST | p95 < 500ms |
| `/auth/me` | GET | p95 < 500ms |

## Expected Results

With bcrypt cost=12:
- **Signup**: ~300-400ms (bcrypt hashing dominates)
- **Login**: ~300-400ms (bcrypt verification dominates)
- **Logout**: <50ms (database update only)
- **Refresh**: <100ms (JWT operations + database)
- **Me**: <50ms (JWT validation only)

All endpoints should comfortably meet the p95 < 500ms requirement.

## Interpreting Results

The benchmark script will output:
- ✅ **PASS**: p95 latency < 500ms
- ❌ **FAIL**: p95 latency ≥ 500ms

Example output:
```
📈 PERFORMANCE SUMMARY
POST /auth/signup               p95= 387.23ms  ✅ PASS
POST /auth/login                p95= 395.41ms  ✅ PASS
GET /auth/me                    p95=  42.18ms  ✅ PASS
POST /auth/refresh              p95=  89.52ms  ✅ PASS
POST /auth/logout               p95=  38.76ms  ✅ PASS

✅ ALL ENDPOINTS MEET PERFORMANCE REQUIREMENTS (p95 < 500ms)
✅ FR-029 COMPLIANCE: PASSED
```

## Troubleshooting

### Server Not Running

```
❌ Cannot connect to server: Connection refused
Please ensure backend is running at http://localhost:8000
```

**Solution**: Start the backend server first

### Slow Performance

If p95 latencies exceed 500ms:
- Check database connection latency (Neon PostgreSQL)
- Verify bcrypt cost factor is 12 (not higher)
- Check for network latency issues
- Monitor server CPU/memory usage

### Database Connection Errors

```
❌ Error: connection refused or timeout
```

**Solution**: Verify `DATABASE_URL` in `.env` is correct and Neon database is active

## Manual Testing

You can also test endpoints manually with curl:

```bash
# Signup
time curl -X POST http://localhost:8000/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "TestPass123"}' \
  -c cookies.txt

# Login
time curl -X POST http://localhost:8000/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "TestPass123"}' \
  -c cookies.txt -b cookies.txt

# Get current user
time curl -X GET http://localhost:8000/auth/me \
  -b cookies.txt
```

The `time` command will show execution duration.
