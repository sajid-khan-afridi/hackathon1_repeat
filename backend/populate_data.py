"""
Script to populate Qdrant with textbook content
"""
import asyncio
import json
import os
from pathlib import Path
from typing import List, Dict, Any

from app.services.vector_store import vector_store_service
from app.models.document import DocumentChunk
from app.core.config import settings


async def load_textbook_from_directory(directory_path: str) -> List[DocumentChunk]:
    """
    Load textbook content from a directory of MDX files
    """
    chunks = []
    directory = Path(directory_path)

    # Supported file extensions
    supported_extensions = ['.md', '.mdx', '.txt']

    for file_path in directory.rglob('*'):
        if file_path.suffix.lower() in supported_extensions:
            # Extract module, chapter from path
            path_parts = file_path.relative_to(directory).parts
            module = path_parts[0] if len(path_parts) > 0 else "unknown"
            chapter = path_parts[1] if len(path_parts) > 1 else "unknown"

            # Read file content
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Create metadata
            metadata = {
                "module": module,
                "chapter": chapter,
                "file_path": str(file_path.relative_to(directory)),
                "file_name": file_path.name,
                "type": "textbook_content"
            }

            # Split content into chunks (simple approach - you might want more sophisticated chunking)
            paragraphs = content.split('\n\n')
            chunk_text = ""

            for i, paragraph in enumerate(paragraphs):
                if len(chunk_text + paragraph) > 1000:  # Chunk size limit
                    if chunk_text:
                        chunk = DocumentChunk(
                            content=chunk_text.strip(),
                            metadata={
                                **metadata,
                                "chunk_id": f"{file_path.stem}_{len(chunks)}",
                                "chunk_index": len(chunks)
                            }
                        )
                        chunks.append(chunk)
                    chunk_text = paragraph
                else:
                    chunk_text += "\n\n" + paragraph if chunk_text else paragraph

            # Add remaining content
            if chunk_text.strip():
                chunk = DocumentChunk(
                    content=chunk_text.strip(),
                    metadata={
                        **metadata,
                        "chunk_id": f"{file_path.stem}_{len(chunks)}",
                        "chunk_index": len(chunks)
                    }
                )
                chunks.append(chunk)

    return chunks


async def load_from_json(json_file_path: str) -> List[DocumentChunk]:
    """
    Load textbook content from a JSON file
    Expected format:
    [
        {
            "content": "Text content here",
            "metadata": {
                "module": "module-1",
                "chapter": "introduction",
                "section": "what-is-robotics"
            }
        },
        ...
    ]
    """
    chunks = []

    with open(json_file_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    for i, item in enumerate(data):
        chunk = DocumentChunk(
            content=item['content'],
            metadata={
                **item.get('metadata', {}),
                "chunk_id": item.get('id', f"chunk_{i}"),
                "chunk_index": i
            }
        )
        chunks.append(chunk)

    return chunks


async def populate_vector_store():
    """
    Main function to populate the vector store with textbook content
    """
    print("📚 Starting to populate Qdrant with textbook content...")

    # Clear existing collection (optional)
    print("🗑️  Clearing existing collection...")
    await vector_store_service.delete_collection()
    await vector_store_service.create_collection()

    # Load content from different sources

    # Option 1: Load from directory of MDX files
    content_dir = os.path.join(os.path.dirname(__file__), '..', 'static', 'data', 'textbook')
    if os.path.exists(content_dir):
        print(f"📁 Loading content from {content_dir}")
        chunks = await load_textbook_from_directory(content_dir)
        print(f"   Found {len(chunks)} chunks")
    else:
        # Option 2: Load from JSON file
        json_file = os.path.join(os.path.dirname(__file__), '..', 'static', 'data', 'textbook.json')
        if os.path.exists(json_file):
            print(f"📄 Loading content from {json_file}")
            chunks = await load_from_json(json_file)
            print(f"   Found {len(chunks)} chunks")
        else:
            print("⚠️  No textbook content found!")
            print("   Add content to:")
            print(f"   - Directory: {content_dir}")
            print(f"   - Or JSON file: {json_file}")
            return

    # Add chunks to vector store
    if chunks:
        print("\n🚀 Adding chunks to Qdrant...")
        batch_size = 100

        for i in range(0, len(chunks), batch_size):
            batch = chunks[i:i+batch_size]
            await vector_store_service.add_documents(batch)
            print(f"   Added {i + len(batch)}/{len(chunks)} chunks")

        print(f"\n✅ Successfully populated vector store with {len(chunks)} chunks!")

        # Verify the collection
        collection_info = await vector_store_service.get_collection_info()
        print(f"\n📊 Collection info:")
        print(f"   - Points stored: {collection_info.points_count}")
        print(f"   - Vector size: {collection_info.vector_size}")
        print(f"   - Collection status: {collection_info.status}")


# Sample content generator for testing
async def generate_sample_content():
    """
    Generate sample robotics textbook content for testing
    """
    print("📝 Generating sample robotics content...")

    sample_content = [
        {
            "content": "Robotics is an interdisciplinary field that combines computer science, engineering, and physics to design, build, and operate robots. Robots are programmable machines that can perform tasks autonomously or with human guidance.",
            "metadata": {
                "module": "introduction",
                "chapter": "what-is-robotics",
                "section": "definition"
            }
        },
        {
            "content": "The main components of a robot include: 1) Sensors - to perceive the environment, 2) Actuators - to perform physical actions, 3) Controller - the brain that processes information, and 4) Power system - to provide energy.",
            "metadata": {
                "module": "introduction",
                "chapter": "robot-components",
                "section": "overview"
            }
        },
        {
            "content": "Forward kinematics is the calculation of the position and orientation of a robot's end-effector given the joint parameters. It involves using transformation matrices to compute the final position in 3D space.",
            "metadata": {
                "module": "kinematics",
                "chapter": "forward-kinematics",
                "section": "basics"
            }
        },
        {
            "content": "Inverse kinematics is the reverse problem - finding the joint parameters required to place the end-effector at a desired position and orientation. This is more complex and may have multiple solutions or no solution.",
            "metadata": {
                "module": "kinematics",
                "chapter": "inverse-kinematics",
                "section": "introduction"
            }
        },
        {
            "content": "The Denavit-Hartenberg (DH) convention is a systematic method for assigning coordinate frames to robot links. It uses four parameters (a, α, d, θ) to describe the relationship between consecutive joint axes.",
            "metadata": {
                "module": "kinematics",
                "chapter": "dh-parameters",
                "section": "convention"
            }
        },
        {
            "content": "PID (Proportional-Integral-Derivative) control is a common feedback control loop used in robotics. The proportional term responds to present error, integral to accumulated error, and derivative to rate of change.",
            "metadata": {
                "module": "control",
                "chapter": "pid-control",
                "section": "theory"
            }
        },
        {
            "content": "Path planning involves finding a collision-free path from start to goal configuration. Algorithms include A*, RRT (Rapidly-exploring Random Trees), and probabilistic roadmaps.",
            "metadata": {
                "module": "planning",
                "chapter": "path-planning",
                "section": "algorithms"
            }
        },
        {
            "content": "ROS (Robot Operating System) is a flexible framework for writing robot software. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, and package management.",
            "metadata": {
                "module": "software",
                "chapter": "ros",
                "section": "introduction"
            }
        }
    ]

    # Convert to DocumentChunk objects
    chunks = []
    for i, item in enumerate(sample_content):
        chunk = DocumentChunk(
            content=item['content'],
            metadata={
                **item['metadata'],
                "chunk_id": f"sample_{i}",
                "chunk_index": i,
                "type": "sample_content"
            }
        )
        chunks.append(chunk)

    return chunks


if __name__ == "__main__":
    import sys

    # Check command line arguments
    if len(sys.argv) > 1 and sys.argv[1] == "--sample":
        # Generate sample content
        asyncio.run(generate_sample_content())
    else:
        # Populate from actual content
        asyncio.run(populate_vector_store())