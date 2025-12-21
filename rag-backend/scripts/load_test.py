#!/usr/bin/env python3
"""
Load test script for RAG Chatbot API.

Simulates 10 concurrent users submitting queries and measures:
- Response times
- Error rates
- Throughput
- Concurrent request handling
"""
import asyncio
import time
import statistics
from typing import List, Dict
import httpx


class LoadTester:
    """Load tester for RAG Chatbot API."""

    def __init__(self, base_url: str = "http://localhost:8000"):
        """
        Initialize load tester.

        Args:
            base_url: Base URL of the API
        """
        self.base_url = base_url.rstrip("/")
        self.results = []

    async def send_query(self, client: httpx.AsyncClient, query_text: str, user_id: int) -> Dict:
        """
        Send a single query and measure response time.

        Args:
            client: HTTP client
            query_text: Query text
            user_id: User identifier

        Returns:
            Result dictionary with timing and status
        """
        start_time = time.time()

        try:
            response = await client.post(
                f"{self.base_url}/api/query",
                json={
                    "query_text": query_text,
                    "selected_text": None,
                },
                timeout=10.0,
            )

            duration = time.time() - start_time

            return {
                "user_id": user_id,
                "query": query_text,
                "status_code": response.status_code,
                "duration": duration,
                "success": response.status_code == 200,
                "error": None if response.status_code == 200 else response.text[:200],
            }

        except Exception as e:
            duration = time.time() - start_time

            return {
                "user_id": user_id,
                "query": query_text,
                "status_code": 0,
                "duration": duration,
                "success": False,
                "error": str(e)[:200],
            }

    async def run_concurrent_test(self, num_users: int = 10) -> None:
        """
        Run concurrent load test with multiple users.

        Args:
            num_users: Number of concurrent users (default: 10)
        """
        print(f"\n{'='*60}")
        print(f"RAG Chatbot Load Test - {num_users} Concurrent Users")
        print(f"{'='*60}\n")

        # Test queries
        test_queries = [
            "What is forward kinematics?",
            "Explain robot inverse kinematics",
            "What are the types of robot joints?",
            "How does sensor fusion work?",
            "What is localization in robotics?",
            "Explain path planning algorithms",
            "What is SLAM?",
            "Describe robot control systems",
            "What are end-effectors?",
            "Explain coordinate transformations",
        ]

        async with httpx.AsyncClient() as client:
            # Create tasks for concurrent queries
            tasks = []
            for user_id in range(num_users):
                query = test_queries[user_id % len(test_queries)]
                task = self.send_query(client, query, user_id)
                tasks.append(task)

            # Execute all queries concurrently
            print(f"Sending {num_users} concurrent queries...")
            start_time = time.time()

            results = await asyncio.gather(*tasks)

            total_duration = time.time() - start_time

            # Store results
            self.results = results

            # Print summary
            self.print_summary(total_duration)

    def print_summary(self, total_duration: float) -> None:
        """
        Print load test summary.

        Args:
            total_duration: Total test duration
        """
        if not self.results:
            print("No results to display")
            return

        # Calculate statistics
        successful = [r for r in self.results if r["success"]]
        failed = [r for r in self.results if not r["success"]]

        success_rate = (len(successful) / len(self.results)) * 100
        error_rate = (len(failed) / len(self.results)) * 100

        durations = [r["duration"] for r in successful]

        print(f"\n{'='*60}")
        print("LOAD TEST RESULTS")
        print(f"{'='*60}\n")

        print(f"Total Requests:     {len(self.results)}")
        print(f"Successful:         {len(successful)} ({success_rate:.1f}%)")
        print(f"Failed:             {len(failed)} ({error_rate:.1f}%)")
        print(f"Total Duration:     {total_duration:.2f}s")
        print()

        if durations:
            print("Response Time Statistics:")
            print(f"  Min:              {min(durations):.2f}s")
            print(f"  Max:              {max(durations):.2f}s")
            print(f"  Mean:             {statistics.mean(durations):.2f}s")
            print(f"  Median:           {statistics.median(durations):.2f}s")

            if len(durations) > 1:
                print(f"  Std Dev:          {statistics.stdev(durations):.2f}s")

            # Check if all responses are within 3s
            within_3s = [d for d in durations if d < 3.0]
            within_3s_rate = (len(within_3s) / len(durations)) * 100
            print(f"  Within 3s:        {len(within_3s)}/{len(durations)} ({within_3s_rate:.1f}%)")
            print()

        # Print failures if any
        if failed:
            print("\nFAILED REQUESTS:")
            for i, result in enumerate(failed, 1):
                print(f"  {i}. User {result['user_id']}: {result['query'][:50]}...")
                print(f"     Status: {result['status_code']}")
                print(f"     Error: {result['error']}")
                print()

        # Verdict
        print(f"{'='*60}")
        if success_rate == 100 and all(d < 3.0 for d in durations):
            print("✅ PASS: All queries successful, all responses < 3s")
        elif success_rate == 100:
            print("⚠️  PARTIAL: All queries successful, but some > 3s")
        else:
            print("❌ FAIL: Some queries failed or timed out")
        print(f"{'='*60}\n")


async def main():
    """Run load test."""
    tester = LoadTester()

    # Test 10 concurrent users
    await tester.run_concurrent_test(num_users=10)


if __name__ == "__main__":
    asyncio.run(main())
