"""
Add Test Data with Real Cohere Embeddings
Uses query embeddings (which work) instead of document embeddings
"""
import sys
import os
from pathlib import Path
import uuid
import time

sys.path.insert(0, str(Path(__file__).parent.parent))

from dotenv import load_dotenv
from app.config.settings import get_settings
from app.services.cohere_service import CohereService
from app.services.qdrant_service import QdrantService

load_dotenv()

def main():
    print("[INFO] Adding test data with real Cohere embeddings...")

    settings = get_settings()
    cohere_service = CohereService(settings.cohere_api_key, settings.cohere_embedding_model)
    qdrant_service = QdrantService(settings.qdrant_url, settings.qdrant_api_key, settings.qdrant_collection_name)

    # Test chunks - ROS2 content
    test_chunks = [
        {
            'content': """ROS2 (Robot Operating System 2) is an open-source middleware framework that serves as the communication backbone for modern robots. Think of it as the nervous system that allows different parts of a robot—sensors, motors, cameras, decision-making algorithms—to talk to each other efficiently and reliably. Unlike traditional software where all code runs in a single program, ROS2 enables distributed systems architecture.""",
            'metadata': {
                'module': '01-ros2-nervous-system',
                'lesson': '01-ros2-fundamentals',
                'lesson_title': 'ROS2 Fundamentals',
                'module_title': 'Module 1: ROS2 Nervous System',
                'section': 'What Is ROS2?',
                'tags': ['ros2', 'middleware', 'fundamentals'],
            }
        },
        {
            'content': """In ROS2, a node is an independent process responsible for a single, well-defined task within your robot system. Think of nodes as specialized workers in a factory: one worker handles packaging, another handles quality control, another manages inventory. Each node focuses on doing one thing well, and they coordinate through structured communication.""",
            'metadata': {
                'module': '01-ros2-nervous-system',
                'lesson': '02-nodes-topics-services',
                'lesson_title': 'Nodes, Topics, and Services',
                'module_title': 'Module 1: ROS2 Nervous System',
                'section': 'What Are Nodes?',
                'tags': ['ros2', 'nodes', 'architecture'],
            }
        },
        {
            'content': """rclpy (ROS Client Library for Python) is the official Python API for ROS2. It's the bridge that connects your Python programming skills to the powerful ROS2 middleware infrastructure. When you write robot code in Python, rclpy handles all the complex networking, message serialization, and inter-process communication behind the scenes.""",
            'metadata': {
                'module': '01-ros2-nervous-system',
                'lesson': '03-python-rclpy-bridge',
                'lesson_title': 'Python rclpy Bridge',
                'module_title': 'Module 1: ROS2 Nervous System',
                'section': 'What Is rclpy?',
                'tags': ['ros2', 'python', 'rclpy', 'api'],
            }
        },
    ]

    # Upload to Qdrant with real embeddings
    points = []
    for i, chunk in enumerate(test_chunks, 1):
        print(f"[INFO] Embedding chunk {i}/{len(test_chunks)}: {chunk['metadata']['section']}")

        # Use query embedding (works better than document embedding)
        embedding = cohere_service.embed_query(chunk['content'])

        chunk_id = str(uuid.uuid4())
        payload = {
            'chunk_id': chunk_id,
            'content': chunk['content'],
            **chunk['metadata']
        }

        points.append({
            'id': chunk_id,
            'vector': embedding,
            'payload': payload
        })

        # Small delay to avoid rate limits
        if i < len(test_chunks):
            time.sleep(1)

    print("[INFO] Uploading to Qdrant...")
    qdrant_service.client.upsert(
        collection_name=qdrant_service.collection_name,
        points=points
    )

    print(f"[SUCCESS] Added {len(points)} chunks with real Cohere embeddings")
    print(f"Collection: {settings.qdrant_collection_name}")
    print("\nTest chunks:")
    for i, chunk in enumerate(test_chunks, 1):
        print(f"  {i}. {chunk['metadata']['lesson_title']} - {chunk['metadata']['section']}")

if __name__ == '__main__':
    main()
