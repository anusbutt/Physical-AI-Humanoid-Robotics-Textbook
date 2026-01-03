"""Quick script to check Qdrant collection status"""
import sys
sys.path.insert(0, '.')

from app.config.settings import get_settings
from qdrant_client import QdrantClient

settings = get_settings()

client = QdrantClient(
    url=settings.qdrant_url,
    api_key=settings.qdrant_api_key
)

info = client.get_collection(settings.qdrant_collection_name)
print(f"[OK] Collection '{settings.qdrant_collection_name}' exists!")
print(f"   Points count: {info.points_count}")
print(f"   Status: {info.status}")

# Try to count points with a scroll
from qdrant_client.models import Filter
count = client.count(collection_name=settings.qdrant_collection_name)
print(f"   Total vectors: {count.count}")
