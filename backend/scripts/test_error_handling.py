"""
Test Error Handling & Out-of-Scope Detection

Tests the chatbot's ability to handle errors gracefully and provide helpful suggestions.
"""

import requests
import json


# Configuration
API_URL = "http://127.0.0.1:8001/api/chat/query"


def print_section(title: str):
    """Print a formatted section header."""
    print(f"\n{'='*70}")
    print(f"{title}")
    print(f"{'='*70}\n")


def send_query(query: str, verbose: bool = True):
    """
    Send a query to the chat API.

    Args:
        query: User's question
        verbose: Whether to print detailed output

    Returns:
        Response data or None if failed
    """
    payload = {
        "query": query,
        "conversation_history": []
    }

    try:
        response = requests.post(API_URL, json=payload, timeout=30)

        if response.status_code == 200:
            data = response.json()
            if verbose:
                print(f"[SUCCESS] Status: {response.status_code}")
                print(f"\n{'-'*70}")
                print("CHATBOT RESPONSE:")
                print(f"{'-'*70}")
                # Handle encoding issues for Windows console
                try:
                    print(data['response'])
                except UnicodeEncodeError:
                    print(data['response'].encode('ascii', 'replace').decode('ascii'))
                print(f"\n{'-'*70}")
                print(f"Sources: {len(data.get('sources', []))}")
            return data
        else:
            if verbose:
                print(f"[ERROR] Status: {response.status_code}")
                print(f"Response: {response.text}")
            return None

    except requests.exceptions.ConnectionError:
        print("[ERROR] Could not connect to API")
        print("Make sure the server is running: uvicorn app.main:app --port 8001")
        return None
    except Exception as e:
        print(f"[ERROR] {e}")
        return None


def test_out_of_scope_queries():
    """Test chatbot's handling of out-of-scope questions."""
    print_section("TEST 1: Out-of-Scope Query Detection")

    out_of_scope_queries = [
        "What's the weather today?",
        "Who won the world cup?",
        "How do I install Python on Windows?",
        "Tell me a joke about robots",
        "What's for dinner tonight?"
    ]

    results = []

    for i, query in enumerate(out_of_scope_queries, 1):
        print(f"\n--- Test 1.{i}: Out-of-Scope Query ---")
        print(f"Query: {query}")

        response = send_query(query)

        if response:
            # Check if response mentions "outside the scope" or suggests topics
            response_text = response['response'].lower()
            is_out_of_scope_detected = (
                "outside" in response_text and "scope" in response_text
            ) or (
                "can help you with" in response_text
            )

            if is_out_of_scope_detected:
                print(f"[PASS] Out-of-scope correctly detected")
                results.append(True)
            else:
                print(f"[FAIL] Out-of-scope NOT detected properly")
                results.append(False)
        else:
            print(f"[FAIL] No response received")
            results.append(False)

    passed = sum(results)
    total = len(results)
    print(f"\n{'='*70}")
    print(f"Out-of-Scope Detection Results: {passed}/{total} passed")
    print(f"{'='*70}")

    return passed == total


def test_no_results_queries():
    """Test chatbot's handling of queries with no matching results."""
    print_section("TEST 2: No Results Found Handling")

    no_results_queries = [
        "Tell me about quantum entanglement in robotics",
        "How do unicorns use ROS2?",
        "Explain the theory of everything for robots",
        "What is xyzabc123?",  # Nonsense term
    ]

    results = []

    for i, query in enumerate(no_results_queries, 1):
        print(f"\n--- Test 2.{i}: No Results Query ---")
        print(f"Query: {query}")

        response = send_query(query)

        if response:
            response_text = response['response'].lower()

            # Check if response provides helpful suggestions
            has_suggestions = (
                "couldn't find" in response_text or
                "don't have enough information" in response_text or
                "here are topics" in response_text or
                "rephrase" in response_text
            )

            if has_suggestions:
                print(f"[PASS] Helpful suggestions provided")
                results.append(True)
            else:
                print(f"[FAIL] No helpful suggestions provided")
                results.append(False)
        else:
            print(f"[FAIL] No response received")
            results.append(False)

    passed = sum(results)
    total = len(results)
    print(f"\n{'='*70}")
    print(f"No Results Handling Results: {passed}/{total} passed")
    print(f"{'='*70}")

    return passed == total


def test_in_scope_queries():
    """Test that in-scope queries still work properly."""
    print_section("TEST 3: In-Scope Queries (Baseline)")

    in_scope_queries = [
        "What is ROS2?",
        "Tell me about sensors in robotics",
        "What are nodes in ROS2?",
    ]

    results = []

    for i, query in enumerate(in_scope_queries, 1):
        print(f"\n--- Test 3.{i}: In-Scope Query ---")
        print(f"Query: {query}")

        response = send_query(query, verbose=False)

        if response and response.get('response'):
            # Check that we get a real response (not out-of-scope message)
            response_text = response['response'].lower()
            is_proper_response = (
                len(response_text) > 100 and  # Substantial response
                "outside the scope" not in response_text  # Not rejected
            )

            if is_proper_response:
                print(f"[PASS] Proper response received ({len(response_text)} chars)")
                results.append(True)
            else:
                print(f"[FAIL] Response seems incorrect")
                results.append(False)
        else:
            print(f"[FAIL] No response received")
            results.append(False)

    passed = sum(results)
    total = len(results)
    print(f"\n{'='*70}")
    print(f"In-Scope Query Results: {passed}/{total} passed")
    print(f"{'='*70}")

    return passed == total


def run_all_tests():
    """Run all error handling tests."""
    print("="*70)
    print("ERROR HANDLING & OUT-OF-SCOPE DETECTION TEST SUITE")
    print("="*70)

    results = {
        "Out-of-Scope Detection": False,
        "No Results Handling": False,
        "In-Scope Queries (Baseline)": False
    }

    # Run tests
    results["Out-of-Scope Detection"] = test_out_of_scope_queries()
    input("\nPress Enter to continue to next test...")

    results["No Results Handling"] = test_no_results_queries()
    input("\nPress Enter to continue to next test...")

    results["In-Scope Queries (Baseline)"] = test_in_scope_queries()

    # Print summary
    print_section("TEST SUMMARY")
    for test_name, passed in results.items():
        status = "[PASS]" if passed else "[FAIL]"
        print(f"{status} {test_name}")

    passed_count = sum(1 for passed in results.values() if passed)
    total_count = len(results)

    print(f"\n{'='*70}")
    print(f"Results: {passed_count}/{total_count} test groups passed")
    print(f"{'='*70}")

    return passed_count == total_count


if __name__ == "__main__":
    success = run_all_tests()
    exit(0 if success else 1)
