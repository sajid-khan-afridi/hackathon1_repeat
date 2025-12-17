#!/usr/bin/env python3
"""
Clean up old FAQ entries from Qdrant collection.
Delete all points with chapter_id="unknown"
"""
import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize Qdrant client
client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

collection_name = "robotics_textbook"

print("=" * 60)
print("Qdrant Cleanup - Remove Old FAQ Entries")
print("=" * 60)

# Check current count
collection_info = client.get_collection(collection_name)
print(f"Current points: {collection_info.points_count}")

# Delete all points with chapter_id="unknown"
print("\nDeleting FAQ entries with chapter_id='unknown'...")
from qdrant_client.models import Filter, FieldCondition, MatchValue

client.delete(
    collection_name=collection_name,
    points_selector=Filter(
        must=[
            FieldCondition(
                key="chapter_id",
                match=MatchValue(value="unknown")
            )
        ]
    )
)

print("[OK] Deleted old FAQ entries")

# Check new count
import time
time.sleep(2)  # Wait for deletion to complete

collection_info = client.get_collection(collection_name)
print(f"Points after cleanup: {collection_info.points_count}")
print("\n" + "=" * 60)
print("[SUCCESS] Cleanup completed!")
print("=" * 60)
