#!/usr/bin/env python3
"""
Reset Qdrant collection - delete and let content-indexer recreate it fresh.
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
print("Qdrant Reset - Delete Collection")
print("=" * 60)

try:
    # Check if collection exists
    collection_info = client.get_collection(collection_name)
    print(f"Current collection: {collection_name}")
    print(f"Points: {collection_info.points_count}")

    # Delete the collection
    print(f"\nDeleting collection '{collection_name}'...")
    client.delete_collection(collection_name)
    print("[OK] Collection deleted successfully!")

except Exception as e:
    print(f"[INFO] Collection might not exist: {e}")

print("\n" + "=" * 60)
print("[SUCCESS] Reset completed!")
print("Now run the content-indexer to recreate it with fresh data.")
print("=" * 60)
