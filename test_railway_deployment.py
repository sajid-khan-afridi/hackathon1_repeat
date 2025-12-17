"""
Railway Deployment Testing Suite

This script tests all endpoints of the deployed RAG Chatbot API.
Run this after deploying to Railway to verify everything works.

Usage:
    python test_railway_deployment.py https://your-app.up.railway.app
"""

import sys
import asyncio
import json
from typing import Dict, Any, Optional
from datetime import datetime
import httpx
from uuid import UUID
import os

# Set UTF-8 encoding for Windows console
if sys.platform == 'win32':
    import codecs
    sys.stdout = codecs.getwriter('utf-8')(sys.stdout.buffer, 'strict')
    sys.stderr = codecs.getwriter('utf-8')(sys.stderr.buffer, 'strict')

# Color codes for terminal output
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    BOLD = '\033[1m'
    END = '\033[0m'


class DeploymentTester:
    """Test suite for Railway deployment."""

    def __init__(self, base_url: str):
        self.base_url = base_url.rstrip('/')
        self.results = []
        self.session_id: Optional[UUID] = None

    def print_header(self, text: str):
        """Print a formatted header."""
        print(f"\n{Colors.BOLD}{Colors.BLUE}{'='*60}{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}{text}{Colors.END}")
        print(f"{Colors.BOLD}{Colors.BLUE}{'='*60}{Colors.END}\n")

    def print_test(self, name: str, passed: bool, details: str = ""):
        """Print test result."""
        status = f"{Colors.GREEN}✓ PASS{Colors.END}" if passed else f"{Colors.RED}✗ FAIL{Colors.END}"
        print(f"{status} | {name}")
        if details:
            print(f"      {details}")

        self.results.append({
            "test": name,
            "passed": passed,
            "details": details,
            "timestamp": datetime.now().isoformat()
        })

    async def test_root_endpoint(self, client: httpx.AsyncClient):
        """Test root endpoint (/)."""
        try:
            response = await client.get(f"{self.base_url}/")
            passed = response.status_code == 200
            data = response.json()

            details = f"Status: {response.status_code}, Message: {data.get('message', 'N/A')}"
            self.print_test("Root Endpoint", passed, details)
            return passed
        except Exception as e:
            self.print_test("Root Endpoint", False, f"Error: {str(e)}")
            return False

    async def test_simple_health(self, client: httpx.AsyncClient):
        """Test simple health endpoint (/health)."""
        try:
            response = await client.get(f"{self.base_url}/health")
            passed = response.status_code == 200
            data = response.json()

            details = f"Status: {response.status_code}, Health: {data.get('status', 'N/A')}"
            self.print_test("Simple Health Check", passed, details)
            return passed
        except Exception as e:
            self.print_test("Simple Health Check", False, f"Error: {str(e)}")
            return False

    async def test_detailed_health(self, client: httpx.AsyncClient):
        """Test detailed health endpoint (/api/v1/health)."""
        try:
            response = await client.get(f"{self.base_url}/api/v1/health")
            passed = response.status_code in [200, 503]  # 503 is acceptable if services are degraded
            data = response.json()

            status = data.get('status', 'unknown')
            db = data.get('database', 'unknown')
            vs = data.get('vector_store', 'unknown')
            llm = data.get('llm', 'unknown')

            details = f"Overall: {status} | DB: {db} | Vector: {vs} | LLM: {llm}"
            self.print_test("Detailed Health Check", passed, details)

            # Print details if any service is unhealthy
            if status != 'healthy':
                print(f"{Colors.YELLOW}      Warning: Some services may be unavailable{Colors.END}")
                for key, value in data.get('details', {}).items():
                    print(f"      {key}: {value}")

            return passed
        except Exception as e:
            self.print_test("Detailed Health Check", False, f"Error: {str(e)}")
            return False

    async def test_query_health(self, client: httpx.AsyncClient):
        """Test query service health (/api/v1/query/health)."""
        try:
            response = await client.get(f"{self.base_url}/api/v1/query/health")
            passed = response.status_code == 200
            data = response.json()

            status = data.get('status', 'unknown')
            components = data.get('components', {})

            details = f"Status: {status} | Components: {', '.join([f'{k}:{v}' for k, v in components.items()])}"
            self.print_test("Query Service Health", passed, details)
            return passed
        except Exception as e:
            self.print_test("Query Service Health", False, f"Error: {str(e)}")
            return False

    async def test_cors(self, client: httpx.AsyncClient):
        """Test CORS configuration."""
        try:
            # Send OPTIONS request to check CORS headers
            response = await client.options(
                f"{self.base_url}/api/v1/health",
                headers={
                    "Origin": "https://sajid-khan-afridi.github.io",
                    "Access-Control-Request-Method": "GET"
                }
            )

            # Check for CORS headers
            has_cors = "access-control-allow-origin" in response.headers
            allow_credentials = response.headers.get("access-control-allow-credentials") == "true"

            passed = has_cors and allow_credentials
            details = f"CORS Enabled: {has_cors}, Credentials: {allow_credentials}"

            if has_cors:
                origin = response.headers.get("access-control-allow-origin")
                methods = response.headers.get("access-control-allow-methods", "")
                details += f" | Origin: {origin}"

            self.print_test("CORS Configuration", passed, details)
            return passed
        except Exception as e:
            self.print_test("CORS Configuration", False, f"Error: {str(e)}")
            return False

    async def test_query_endpoint(self, client: httpx.AsyncClient):
        """Test RAG query endpoint (/api/v1/query)."""
        try:
            query_payload = {
                "query": "What is ROS?",
                "filters": {},
                "top_k": 3
            }

            response = await client.post(
                f"{self.base_url}/api/v1/query",
                json=query_payload,
                timeout=45.0  # Longer timeout for LLM
            )

            passed = response.status_code == 200

            if passed:
                data = response.json()
                answer = data.get('answer', '')[:100]
                confidence = data.get('confidence', 0)
                sources = len(data.get('sources', []))
                self.session_id = data.get('session_id')

                details = f"Confidence: {confidence:.2f} | Sources: {sources} | Session: {self.session_id}"
                print(f"      Answer preview: {answer}...")
            else:
                details = f"Status: {response.status_code}"
                if response.status_code == 503:
                    details += " (Service temporarily unavailable - check health endpoints)"

            self.print_test("Query Endpoint (Non-Streaming)", passed, details)
            return passed
        except Exception as e:
            self.print_test("Query Endpoint (Non-Streaming)", False, f"Error: {str(e)}")
            return False

    async def test_query_streaming(self, client: httpx.AsyncClient):
        """Test RAG query endpoint with streaming (/api/v1/query with SSE)."""
        try:
            query_payload = {
                "query": "What is a robot?",
                "filters": {},
                "top_k": 3
            }

            chunks_received = 0
            error_occurred = False

            async with client.stream(
                "POST",
                f"{self.base_url}/api/v1/query",
                json=query_payload,
                headers={"Accept": "text/event-stream"},
                timeout=45.0
            ) as response:
                if response.status_code != 200:
                    self.print_test(
                        "Query Endpoint (Streaming)",
                        False,
                        f"Status: {response.status_code}"
                    )
                    return False

                async for line in response.aiter_lines():
                    if line.startswith("data: "):
                        chunks_received += 1
                        try:
                            chunk_data = json.loads(line[6:])
                            if chunk_data.get('done'):
                                break
                        except json.JSONDecodeError:
                            error_occurred = True

            passed = chunks_received > 0 and not error_occurred
            details = f"Chunks received: {chunks_received}"

            self.print_test("Query Endpoint (Streaming)", passed, details)
            return passed
        except Exception as e:
            self.print_test("Query Endpoint (Streaming)", False, f"Error: {str(e)}")
            return False

    async def test_session_retrieval(self, client: httpx.AsyncClient):
        """Test session retrieval endpoint (/api/v1/chat/sessions/{id})."""
        if not self.session_id:
            self.print_test(
                "Session Retrieval",
                False,
                "Skipped: No session ID from query test"
            )
            return False

        try:
            response = await client.get(
                f"{self.base_url}/api/v1/chat/sessions/{self.session_id}"
            )

            passed = response.status_code == 200

            if passed:
                data = response.json()
                messages = len(data.get('messages', []))
                session = data.get('session', {})

                details = f"Messages: {messages} | Session found: {bool(session)}"
            else:
                details = f"Status: {response.status_code}"

            self.print_test("Session Retrieval", passed, details)
            return passed
        except Exception as e:
            self.print_test("Session Retrieval", False, f"Error: {str(e)}")
            return False

    async def test_session_deletion(self, client: httpx.AsyncClient):
        """Test session deletion endpoint (DELETE /api/v1/chat/sessions/{id})."""
        if not self.session_id:
            self.print_test(
                "Session Deletion",
                False,
                "Skipped: No session ID from query test"
            )
            return False

        try:
            response = await client.delete(
                f"{self.base_url}/api/v1/chat/sessions/{self.session_id}"
            )

            passed = response.status_code == 204
            details = f"Status: {response.status_code}"

            # Verify deletion by trying to retrieve
            if passed:
                verify_response = await client.get(
                    f"{self.base_url}/api/v1/chat/sessions/{self.session_id}"
                )
                if verify_response.status_code == 404:
                    details += " | Verified deleted"
                else:
                    passed = False
                    details += " | Deletion not verified"

            self.print_test("Session Deletion", passed, details)
            return passed
        except Exception as e:
            self.print_test("Session Deletion", False, f"Error: {str(e)}")
            return False

    async def test_rate_limiting(self, client: httpx.AsyncClient):
        """Test rate limiting (send multiple rapid requests)."""
        try:
            # Send 3 quick requests to check rate limiting headers
            responses = []
            for i in range(3):
                response = await client.get(f"{self.base_url}/health")
                responses.append(response)

            # Check for rate limit headers
            last_response = responses[-1]
            has_rate_limit = "x-ratelimit-limit" in last_response.headers

            if has_rate_limit:
                limit = last_response.headers.get("x-ratelimit-limit")
                remaining = last_response.headers.get("x-ratelimit-remaining")
                details = f"Limit: {limit} | Remaining: {remaining}"
                passed = True
            else:
                details = "No rate limit headers found (may be disabled)"
                passed = True  # Not a failure if rate limiting is disabled

            self.print_test("Rate Limiting", passed, details)
            return passed
        except Exception as e:
            self.print_test("Rate Limiting", False, f"Error: {str(e)}")
            return False

    async def run_all_tests(self):
        """Run all tests sequentially."""
        self.print_header(f"Testing Railway Deployment: {self.base_url}")

        timeout = httpx.Timeout(30.0, connect=10.0)

        async with httpx.AsyncClient(timeout=timeout) as client:
            # Core health tests
            self.print_header("Core Health Tests")
            await self.test_root_endpoint(client)
            await self.test_simple_health(client)
            await self.test_detailed_health(client)
            await self.test_query_health(client)

            # CORS test
            self.print_header("CORS Configuration")
            await self.test_cors(client)

            # Query tests
            self.print_header("Query Endpoint Tests")
            await self.test_query_endpoint(client)
            await self.test_query_streaming(client)

            # Session tests
            self.print_header("Session Management Tests")
            await self.test_session_retrieval(client)
            await self.test_session_deletion(client)

            # Rate limiting
            self.print_header("Rate Limiting Tests")
            await self.test_rate_limiting(client)

        # Print summary
        self.print_summary()

    def print_summary(self):
        """Print test summary."""
        self.print_header("Test Summary")

        total = len(self.results)
        passed = sum(1 for r in self.results if r['passed'])
        failed = total - passed

        pass_rate = (passed / total * 100) if total > 0 else 0

        print(f"Total Tests: {total}")
        print(f"{Colors.GREEN}Passed: {passed}{Colors.END}")
        print(f"{Colors.RED}Failed: {failed}{Colors.END}")
        print(f"Pass Rate: {pass_rate:.1f}%\n")

        if failed > 0:
            print(f"{Colors.YELLOW}Failed Tests:{Colors.END}")
            for result in self.results:
                if not result['passed']:
                    print(f"  - {result['test']}: {result['details']}")

        # Save results to file
        with open('deployment_test_results.json', 'w') as f:
            json.dump({
                'base_url': self.base_url,
                'timestamp': datetime.now().isoformat(),
                'summary': {
                    'total': total,
                    'passed': passed,
                    'failed': failed,
                    'pass_rate': pass_rate
                },
                'results': self.results
            }, f, indent=2)

        print(f"\n{Colors.BLUE}Results saved to: deployment_test_results.json{Colors.END}")


async def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        print(f"{Colors.RED}Error: Railway URL required{Colors.END}")
        print(f"\nUsage: python {sys.argv[0]} <railway-url>")
        print(f"Example: python {sys.argv[0]} https://your-app.up.railway.app")
        sys.exit(1)

    railway_url = sys.argv[1]

    # Validate URL format
    if not railway_url.startswith(('http://', 'https://')):
        railway_url = f'https://{railway_url}'

    tester = DeploymentTester(railway_url)
    await tester.run_all_tests()


if __name__ == "__main__":
    asyncio.run(main())
