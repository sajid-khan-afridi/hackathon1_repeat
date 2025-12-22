"""
Performance benchmarking for authentication endpoints.

Tests all auth endpoints against FR-029 requirement: p95 latency < 500ms

Usage:
    python -m backend.tests.benchmark.auth_performance_test

Requirements:
    - Backend server running on localhost:8000
    - Clean database state
"""

import asyncio
import time
import statistics
from typing import List, Dict, Tuple
import httpx
import json


class AuthPerformanceBenchmark:
    """Performance benchmark for auth endpoints."""

    def __init__(self, base_url: str = "http://localhost:8000"):
        self.base_url = base_url
        self.results: Dict[str, List[float]] = {}
        self.test_email = f"perftest_{int(time.time())}@example.com"
        self.test_password = "TestPass123"
        self.access_token = None
        self.refresh_token = None
        self.cookies = {}

    async def _make_request(
        self,
        method: str,
        endpoint: str,
        json_data: dict = None,
        cookies: dict = None,
    ) -> Tuple[float, httpx.Response]:
        """
        Make HTTP request and measure latency.

        Returns:
            Tuple of (latency_ms, response)
        """
        async with httpx.AsyncClient() as client:
            start = time.perf_counter()

            if method == "GET":
                response = await client.get(
                    f"{self.base_url}{endpoint}",
                    cookies=cookies or {},
                    timeout=10.0,
                )
            elif method == "POST":
                response = await client.post(
                    f"{self.base_url}{endpoint}",
                    json=json_data,
                    cookies=cookies or {},
                    timeout=10.0,
                )
            else:
                raise ValueError(f"Unsupported method: {method}")

            end = time.perf_counter()
            latency_ms = (end - start) * 1000  # Convert to milliseconds

            return latency_ms, response

    async def benchmark_endpoint(
        self,
        name: str,
        method: str,
        endpoint: str,
        json_data: dict = None,
        cookies: dict = None,
        iterations: int = 100,
    ) -> Dict[str, float]:
        """
        Benchmark an endpoint multiple times.

        Returns:
            Dictionary with performance metrics
        """
        print(f"\n📊 Benchmarking: {name}")
        print(f"   Method: {method} {endpoint}")
        print(f"   Iterations: {iterations}")

        latencies = []

        for i in range(iterations):
            try:
                latency_ms, response = await self._make_request(
                    method, endpoint, json_data, cookies
                )

                latencies.append(latency_ms)

                # Update cookies from response if present
                if response.cookies:
                    for cookie_name, cookie_value in response.cookies.items():
                        if cookies is not None:
                            cookies[cookie_name] = cookie_value

                # Show progress every 20 iterations
                if (i + 1) % 20 == 0:
                    print(f"   Progress: {i + 1}/{iterations}", end="\r")

            except Exception as e:
                print(f"\n   ❌ Error on iteration {i + 1}: {e}")
                continue

        if not latencies:
            print(f"   ❌ All requests failed for {name}")
            return {}

        # Calculate statistics
        latencies.sort()
        p50 = statistics.median(latencies)
        p95_index = int(len(latencies) * 0.95)
        p95 = latencies[p95_index]
        p99_index = int(len(latencies) * 0.99)
        p99 = latencies[p99_index]
        mean = statistics.mean(latencies)
        min_latency = min(latencies)
        max_latency = max(latencies)

        self.results[name] = latencies

        # Print results
        print(f"\n   ✅ Results:")
        print(f"      Mean:    {mean:.2f}ms")
        print(f"      Median:  {p50:.2f}ms")
        print(f"      p95:     {p95:.2f}ms {'✅' if p95 < 500 else '❌ FAILED'}")
        print(f"      p99:     {p99:.2f}ms")
        print(f"      Min:     {min_latency:.2f}ms")
        print(f"      Max:     {max_latency:.2f}ms")

        return {
            "mean": mean,
            "median": p50,
            "p95": p95,
            "p99": p99,
            "min": min_latency,
            "max": max_latency,
        }

    async def test_signup(self) -> bool:
        """Test /auth/signup endpoint."""
        metrics = await self.benchmark_endpoint(
            name="POST /auth/signup",
            method="POST",
            endpoint="/auth/signup",
            json_data={
                "email": self.test_email,
                "password": self.test_password,
            },
            iterations=1,  # Only signup once
        )

        # Store cookies for subsequent requests
        _, response = await self._make_request(
            "POST",
            "/auth/signup",
            json_data={
                "email": f"bench_{int(time.time())}@example.com",
                "password": self.test_password,
            },
        )

        if response.status_code == 201:
            self.cookies = dict(response.cookies)
            return True

        return False

    async def test_login(self) -> bool:
        """Test /auth/login endpoint."""
        # First, create a test user if not exists
        await self._make_request(
            "POST",
            "/auth/signup",
            json_data={
                "email": self.test_email,
                "password": self.test_password,
            },
        )

        # Benchmark login
        test_cookies = {}
        metrics = await self.benchmark_endpoint(
            name="POST /auth/login",
            method="POST",
            endpoint="/auth/login",
            json_data={
                "email": self.test_email,
                "password": self.test_password,
            },
            cookies=test_cookies,
            iterations=100,
        )

        self.cookies = test_cookies
        return metrics.get("p95", 1000) < 500

    async def test_me(self) -> bool:
        """Test /auth/me endpoint."""
        if not self.cookies:
            print("   ⚠️  Skipping /auth/me - no auth cookies available")
            return False

        metrics = await self.benchmark_endpoint(
            name="GET /auth/me",
            method="GET",
            endpoint="/auth/me",
            cookies=self.cookies,
            iterations=100,
        )

        return metrics.get("p95", 1000) < 500

    async def test_refresh(self) -> bool:
        """Test /auth/refresh endpoint."""
        if not self.cookies:
            print("   ⚠️  Skipping /auth/refresh - no auth cookies available")
            return False

        metrics = await self.benchmark_endpoint(
            name="POST /auth/refresh",
            method="POST",
            endpoint="/auth/refresh",
            cookies=self.cookies,
            iterations=100,
        )

        return metrics.get("p95", 1000) < 500

    async def test_logout(self) -> bool:
        """Test /auth/logout endpoint."""
        if not self.cookies:
            print("   ⚠️  Skipping /auth/logout - no auth cookies available")
            return False

        metrics = await self.benchmark_endpoint(
            name="POST /auth/logout",
            method="POST",
            endpoint="/auth/logout",
            cookies=self.cookies,
            iterations=100,
        )

        return metrics.get("p95", 1000) < 500

    async def run_all_benchmarks(self):
        """Run all performance benchmarks."""
        print("=" * 70)
        print("🚀 Authentication Endpoints Performance Benchmark")
        print("=" * 70)
        print(f"Base URL: {self.base_url}")
        print(f"Target: p95 latency < 500ms (FR-029)")
        print("=" * 70)

        results = {}

        # Test health endpoint first
        print("\n🔍 Testing server connectivity...")
        try:
            latency, response = await self._make_request("GET", "/health")
            if response.status_code == 200:
                print(f"   ✅ Server is running (latency: {latency:.2f}ms)")
            else:
                print(f"   ❌ Server returned status {response.status_code}")
                return
        except Exception as e:
            print(f"   ❌ Cannot connect to server: {e}")
            print(f"   Please ensure backend is running at {self.base_url}")
            return

        # Run benchmarks
        results["signup"] = await self.test_signup()
        results["login"] = await self.test_login()
        results["me"] = await self.test_me()
        results["refresh"] = await self.test_refresh()
        results["logout"] = await self.test_logout()

        # Summary
        print("\n" + "=" * 70)
        print("📈 PERFORMANCE SUMMARY")
        print("=" * 70)

        all_passed = True
        for endpoint, metrics in self.results.items():
            if metrics:
                metrics_sorted = sorted(metrics)
                p95_index = int(len(metrics_sorted) * 0.95)
                p95 = metrics_sorted[p95_index]
                status = "✅ PASS" if p95 < 500 else "❌ FAIL"
                print(f"{endpoint:30} p95={p95:6.2f}ms  {status}")

                if p95 >= 500:
                    all_passed = False

        print("=" * 70)

        if all_passed:
            print("✅ ALL ENDPOINTS MEET PERFORMANCE REQUIREMENTS (p95 < 500ms)")
            print("✅ FR-029 COMPLIANCE: PASSED")
        else:
            print("❌ SOME ENDPOINTS FAILED PERFORMANCE REQUIREMENTS")
            print("❌ FR-029 COMPLIANCE: FAILED")

        print("=" * 70)

        return all_passed


async def main():
    """Main entry point."""
    benchmark = AuthPerformanceBenchmark()
    await benchmark.run_all_benchmarks()


if __name__ == "__main__":
    asyncio.run(main())
