#!/usr/bin/env python3
"""
Index MDX chapter content into Qdrant vector database for semantic search.
Chunks content by sections and generates embeddings using sentence-transformers.
"""

import os
import re
import json
import sys
from pathlib import Path
from typing import List, Dict, Any
import hashlib

try:
    from qdrant_client import QdrantClient
    from qdrant_client.models import Distance, VectorParams, PointStruct
    from sentence_transformers import SentenceTransformer
    import yaml
except ImportError as e:
    print(f"Error: Missing required dependency - {e}")
    print("Install with: pip install qdrant-client sentence-transformers pyyaml")
    sys.exit(1)

# Configuration
COLLECTION_NAME = "robotics_textbook_chapters"
EMBEDDING_MODEL = "sentence-transformers/all-MiniLM-L6-v2"
VECTOR_SIZE = 384

class ContentIndexer:
    def __init__(self, qdrant_url: str = "localhost", qdrant_port: int = 6333):
        """Initialize the content indexer."""
        self.client = QdrantClient(host=qdrant_url, port=qdrant_port)
        self.model = SentenceTransformer(EMBEDDING_MODEL)
        self.collection_name = COLLECTION_NAME

    def parse_frontmatter(self, content: str) -> Dict[str, Any]:
        """Parse YAML frontmatter from MDX content."""
        match = re.match(r'^---\n(.*?)\n---\n', content, re.DOTALL)
        if match:
            try:
                return yaml.safe_load(match.group(1))
            except yaml.YAMLError as e:
                print(f"Warning: Could not parse frontmatter: {e}")
                return {}
        return {}

    def chunk_content_by_sections(self, file_path: Path) -> List[Dict[str, Any]]:
        """Split MDX content into chunks by h2 sections."""
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        frontmatter = self.parse_frontmatter(content)
        content_without_frontmatter = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)

        # Split by h2 headings
        sections = re.split(r'\n(?=## )', content_without_frontmatter)
        chunks = []

        for i, section in enumerate(sections):
            if not section.strip():
                continue

            # Extract section title
            lines = section.split('\n')
            title = lines[0].replace('#', '').strip() if lines else f"Section {i+1}"
            section_content = '\n'.join(lines[1:]) if len(lines) > 1 else ""

            # Create chunk
            chunk = {
                "content": section_content.strip(),
                "section_title": title,
                "section_number": str(i + 1),
                **frontmatter
            }
            chunks.append(chunk)

        return chunks

    def generate_chunks(self, docs_dir: Path) -> List[Dict[str, Any]]:
        """Generate chunks from all MDX files in the docs directory."""
        all_chunks = []

        for mdx_file in docs_dir.rglob("*.mdx"):
            print(f"Processing: {mdx_file}")

            # Extract chapter info from path
            relative_path = mdx_file.relative_to(docs_dir)
            parts = str(relative_path).split('/')

            if len(parts) >= 2:
                module = parts[0]
                chapter_file = parts[-1]
                chapter_num = chapter_file.split('-')[1] if '-' in chapter_file else '00'
                chapter_id = f"ch{chapter_num}"
            else:
                chapter_id = "unknown"
                module = "unknown"

            # Chunk content by sections
            file_chunks = self.chunk_content_by_sections(mdx_file)

            for chunk in file_chunks:
                chunk.update({
                    "chapter_id": chapter_id,
                    "module": module.replace('module-', '').replace('-', '_'),
                    "file_path": str(relative_path),
                    "section_id": f"sec-{chapter_id}-{chunk['section_number']}"
                })

                all_chunks.append(chunk)

        return all_chunks

    def create_collection(self):
        """Create Qdrant collection if it doesn't exist."""
        try:
            self.client.get_collection(self.collection_name)
            print(f"Collection '{self.collection_name}' already exists")
        except:
            print(f"Creating collection '{self.collection_name}'")
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=VECTOR_SIZE,
                    distance=Distance.COSINE
                ),
                optimizers_config={
                    "default_index": {
                        "hnsw": {
                            "m": 16,
                            "ef_construct": 100
                        }
                    }
                }
            )

    def index_chunks(self, chunks: List[Dict[str, Any]]):
        """Index chunks into Qdrant with embeddings."""
        print(f"\nIndexing {len(chunks)} chunks...")

        # Prepare points
        points = []
        for i, chunk in enumerate(chunks):
            # Generate embedding
            embedding = self.model.encode(chunk['content'])

            # Create unique point ID
            content_hash = hashlib.md5(chunk['content'].encode()).hexdigest()[:8]
            point_id = f"{chunk['section_id']}-{content_hash}"

            # Prepare payload
            payload = {
                "chapter_id": chunk.get('chapter_id', ''),
                "chapter_title": chunk.get('title', ''),
                "section_id": chunk.get('section_id', ''),
                "section_title": chunk.get('section_title', ''),
                "content": chunk['content'],
                "tags": chunk.get('tags', []),
                "difficulty_level": chunk.get('difficulty_level', 'beginner'),
                "ros_version": chunk.get('ros_version', 'humble'),
                "module": chunk.get('module', ''),
                "learning_objectives": chunk.get('learning_objectives', []),
                "estimated_time": chunk.get('estimated_time', 0)
            }

            points.append(PointStruct(
                id=point_id,
                vector=embedding.tolist(),
                payload=payload
            ))

            if (i + 1) % 100 == 0:
                print(f"Processed {i + 1}/{len(chunks)} chunks...")

        # Index in batches
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            self.client.upsert(
                collection_name=self.collection_name,
                points=batch
            )
            print(f"Indexed batch {i//batch_size + 1}/{(len(points) - 1)//batch_size + 1}")

        print(f"\n✅ Successfully indexed {len(points)} chunks")

def main():
    if len(sys.argv) < 2:
        print("Usage: python index-content-to-qdrant.py <docs_directory> [qdrant_host] [qdrant_port]")
        sys.exit(1)

    docs_dir = Path(sys.argv[1])
    if not docs_dir.exists():
        print(f"Error: Directory {docs_dir} does not exist")
        sys.exit(1)

    qdrant_host = sys.argv[2] if len(sys.argv) > 2 else "localhost"
    qdrant_port = int(sys.argv[3]) if len(sys.argv) > 3 else 6333

    # Initialize indexer
    indexer = ContentIndexer(qdrant_host, qdrant_port)

    # Create collection if needed
    indexer.create_collection()

    # Generate chunks
    chunks = indexer.generate_chunks(docs_dir)
    print(f"Generated {len(chunks)} chunks")

    # Index chunks
    if chunks:
        indexer.index_chunks(chunks)
    else:
        print("No chunks to index")

if __name__ == "__main__":
    main()