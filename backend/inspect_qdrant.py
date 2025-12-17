#!/usr/bin/env python3
"""
Inspect what's actually stored in Qdrant collection.
"""
import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv
import json

# Load environment variables
load_dotenv()

# Initialize Qdrant client
client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

collection_name = "robotics_textbook"

print("=" * 60)
print("Qdrant Collection Inspection")
print("=" * 60)

# Get collection info
collection_info = client.get_collection(collection_name)
print(f"Total points: {collection_info.points_count}")

# Retrieve a sample of points
points = client.scroll(
    collection_name=collection_name,
    limit=3,
    with_payload=True,
    with_vectors=False
)

print("\n" + "=" * 60)
print("Sample Points (showing payload structure):")
print("=" * 60)

for idx, point in enumerate(points[0], start=1):
    print(f"\n--- Point {idx} ---")
    print(f"ID: {point.id}")
    print(f"Payload keys: {list(point.payload.keys())}")
    print(f"chapter_id: {point.payload.get('chapter_id', 'NOT FOUND')}")
    print(f"title: {point.payload.get('title', 'NOT FOUND')}")
    print(f"chapter_title: {point.payload.get('chapter_title', 'NOT FOUND')}")
    print(f"module: {point.payload.get('module', 'NOT FOUND')}")
    print(f"tags: {point.payload.get('tags', 'NOT FOUND')}")
    print(f"chunk_index: {point.payload.get('chunk_index', 'NOT FOUND')}")
    print(f"chunk_type: {point.payload.get('chunk_type', 'NOT FOUND')}")
    print(f"text_preview: {point.payload.get('text', '')[:100]}...")

print("\n" + "=" * 60)
print("Analysis Complete!")
print("=" * 60)
