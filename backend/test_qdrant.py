#!/usr/bin/env python3
"""
Test Qdrant connectivity and collection status.
"""
import os
from pathlib import Path
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

# Load environment variables
env_path = Path(__file__).parent.parent / ".env"
load_dotenv(env_path)

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION = os.getenv("QDRANT_COLLECTION", "robotics_textbook")


def main():
    """Test Qdrant connection."""
    if not QDRANT_URL or not QDRANT_API_KEY:
        print("ERROR: QDRANT_URL or QDRANT_API_KEY not set")
        return

    print("=" * 60)
    print("Qdrant Vector Store - Connection Test")
    print("=" * 60)
    print(f"URL: {QDRANT_URL}")
    print(f"Collection: {QDRANT_COLLECTION}")
    print("=" * 60)

    try:
        # Connect to Qdrant
        print("\nConnecting to Qdrant...")
        client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
        print("[OK] Connected successfully")

        # List all collections
        print("\n=== ALL COLLECTIONS ===")
        collections = client.get_collections()

        if not collections.collections:
            print("No collections found")
        else:
            print(f"Found {len(collections.collections)} collection(s):")
            for collection in collections.collections:
                print(f"  - {collection.name}")

        # Check if target collection exists
        print(f"\n=== CHECKING COLLECTION: {QDRANT_COLLECTION} ===")
        collection_exists = any(
            c.name == QDRANT_COLLECTION for c in collections.collections
        )

        if collection_exists:
            print(f"[OK] Collection '{QDRANT_COLLECTION}' exists")

            # Get collection info
            collection_info = client.get_collection(QDRANT_COLLECTION)
            print(f"\nCollection Details:")
            print(f"  - Points count: {collection_info.points_count}")

            # Handle different vector config structures
            try:
                if hasattr(collection_info.config.params, 'vectors'):
                    vector_config = collection_info.config.params.vectors
                    if hasattr(vector_config, 'size'):
                        print(f"  - Vector size: {vector_config.size}")
                    if hasattr(vector_config, 'distance'):
                        print(f"  - Distance: {vector_config.distance}")
            except Exception as e:
                print(f"  - Vector config: {collection_info.config.params}")

            # Try a test search
            print("\n=== TEST SEARCH ===")
            try:
                # Get vector size for test
                vector_size = 1536  # Default for text-embedding-3-small
                if hasattr(collection_info.config.params, 'vectors'):
                    if hasattr(collection_info.config.params.vectors, 'size'):
                        vector_size = collection_info.config.params.vectors.size

                # Search with a zero vector (just to test connectivity)
                results = client.search(
                    collection_name=QDRANT_COLLECTION,
                    query_vector=[0.0] * vector_size,
                    limit=1
                )
                print(f"[OK] Search successful (returned {len(results)} results)")
                if results:
                    print(f"  - Sample result score: {results[0].score}")
            except Exception as e:
                print(f"[WARN] Search test failed: {e}")

        else:
            print(f"[WARN] Collection '{QDRANT_COLLECTION}' NOT FOUND")
            print("\nTo create the collection, you need to:")
            print(f"  1. Index your textbook content using the content-indexer skill")
            print(f"  2. Or manually create collection with vector size 1536 (text-embedding-3-small)")
            print("\nExample creation code:")
            print(f"""
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

client = QdrantClient(url="{QDRANT_URL}", api_key="***")
client.create_collection(
    collection_name="{QDRANT_COLLECTION}",
    vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
)
""")

        print("\n" + "=" * 60)
        print("[SUCCESS] Qdrant connectivity test completed")
        print("=" * 60)

    except Exception as e:
        print(f"\n[FAIL] Qdrant test failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
