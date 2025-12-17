#!/usr/bin/env python3
"""
Test chatbot with improved indexing.
"""
import os
import requests
import json
from dotenv import load_dotenv

load_dotenv()

# Railway backend URL
API_URL = "https://hackathon1repeat-production.up.railway.app/api/chat/query"

print("=" * 60)
print("Testing Chatbot with Improved Indexing")
print("=" * 60)

# Test query
query = "What is ROS 2?"

print(f"\nQuery: {query}")
print("-" * 60)

# Send request
response = requests.post(
    API_URL,
    json={"query": query},
    headers={"Content-Type": "application/json"}
)

if response.status_code == 200:
    data = response.json()

    print(f"\nAnswer:\n{data['answer']}\n")
    print(f"Confidence: {data.get('confidence', 0) * 100:.1f}%")
    print(f"Session ID: {data.get('session_id', 'N/A')}")

    print(f"\nSources ({len(data.get('sources', []))}):")
    for idx, source in enumerate(data.get('sources', [])[:5], start=1):
        print(f"\n{idx}. {source['chapter_title']}")
        print(f"   Relevance: {source['relevance_score'] * 100:.1f}%")
        print(f"   Preview: {source['excerpt'][:100]}...")

    print(f"\nTokens Used: {data.get('tokens_used', {})}")
else:
    print(f"Error: {response.status_code}")
    print(response.text)

print("\n" + "=" * 60)
print("Test completed!")
print("=" * 60)
