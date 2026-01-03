# Cross-Module Search & Lesson Filtering

## Overview

The RAG chatbot supports two search modes:

1. **Cross-Module Search** (default): Searches across ALL course modules
2. **Lesson-Specific Search**: Filters results to a specific lesson

## Cross-Module Search (Default Behavior)

When no `lesson_id` is provided, the chatbot searches across all embedded course content.

### API Usage

```json
POST /api/chat/query
{
  "query": "How do sensors work with ROS2?",
  "conversation_history": []
}
```

### Expected Behavior

- Searches all modules in Qdrant vector database
- Returns relevant chunks from any module
- Can synthesize information across multiple modules
- Example: Query about "VLAs and ROS2" retrieves from both Module 1 (ROS2) and Module 4 (VLA)

### Response

```json
{
  "response": "...",
  "sources": [
    {"module": "Module 1: ROS2 Nervous System", "lesson": "ROS2 Fundamentals", ...},
    {"module": "Module 2: Sensors & Perception", "lesson": "Sensor Types", ...}
  ],
  "retrieved_chunks": 5,
  "used_selected_text": false
}
```

## Lesson-Specific Filtering

Filter search results to a specific lesson by providing `lesson_id`.

###API Usage

```json
POST /api/chat/query
{
  "query": "What is ROS2?",
  "lesson_id": "module-01/lesson-01",
  "conversation_history": []
}
```

### Current Status

⚠️ **Lesson filtering requires Qdrant indexes to be created for the `lesson` field.**

When attempting to filter by lesson_id without proper indexes, you'll receive:

```json
{
  "error": "Invalid filter parameter",
  "detail": "The lesson filter is not available yet. Please search without lesson_id for now."
}
```

### Setting Up Lesson Filtering

To enable lesson filtering:

1. **Add lesson metadata to chunks** when embedding:
   ```python
   chunk_payload = {
       "content": "...",
       "module_title": "Module 1: ROS2 Nervous System",
       "lesson_title": "ROS2 Fundamentals",
       "lesson": "module-01/lesson-01",  # ← Required for filtering
       ...
   }
   ```

2. **Create Qdrant index** on the `lesson` field:
   ```python
   from qdrant_client import models

   qdrant_client.create_payload_index(
       collection_name="humanoid_robotics_book",
       field_name="lesson",
       field_schema=models.PayloadSchemaType.KEYWORD
   )
   ```

3. **Verify index created**:
   ```python
   collection_info = qdrant_client.get_collection("humanoid_robotics_book")
   print(collection_info.payload_schema)
   ```

Once indexes are created, lesson filtering will work automatically.

## Implementation Details

### RAG Service (`backend/app/services/rag_service.py`)

```python
# Step 3: Build filters if lesson_id provided
filters = None
if lesson_id:
    filters = {'lesson': lesson_id}
    logger.info(f"Filtering search to lesson: {lesson_id}")
else:
    logger.info("Cross-module search enabled (no lesson filter)")

# Step 5: Search Qdrant with or without filters
results = self.qdrant.search(
    query_vector=query_embedding,
    limit=adjusted_top_k,
    score_threshold=max(0.3, adjusted_threshold),
    filters=filters  # ← None for cross-module, {'lesson': 'xxx'} for filtered
)
```

### Qdrant Service (`backend/app/services/qdrant_service.py`)

```python
def search(
    self,
    query_vector: List[float],
    limit: int = 10,
    score_threshold: float = 0.7,
    filters: Optional[Dict[str, str]] = None  # ← Lesson filter
) -> List[Dict]:
    # Build filter if provided
    query_filter = None
    if filters:
        conditions = [
            FieldCondition(key=key, match=MatchValue(value=value))
            for key, value in filters.items()
        ]
        query_filter = Filter(must=conditions)

    # Perform search with optional filter
    results = self.client.query_points(
        collection_name=self.collection_name,
        query=query_vector,
        limit=limit,
        score_threshold=score_threshold,
        query_filter=query_filter  # ← Applied here
    )
```

## Testing

Run cross-module search tests:

```bash
cd backend
python scripts/test_cross_module_search.py
```

Tests include:
1. Default cross-module search (no filter)
2. Lesson-specific filtering (requires indexes)
3. Cross-module queries (ROS2 + Sensors, VLA + Perception, etc.)

## Benefits

### Cross-Module Search
- ✅ Students can ask broad questions spanning multiple topics
- ✅ Chatbot synthesizes information from different modules
- ✅ Better learning connections across course content
- ✅ Works out-of-the-box (default behavior)

### Lesson-Specific Filtering
- ✅ Focus answers on specific lesson context
- ✅ Avoid information overload from unrelated modules
- ✅ Better for lesson-specific questions
- ⏳ Requires Qdrant indexes to be set up

## Future Enhancements

1. **Module-level filtering**: Filter by entire module (e.g., "module-01")
2. **Tag-based filtering**: Filter by tags like "ros2", "sensors", "isaac"
3. **Skill-level filtering**: Filter by Bloom's taxonomy level
4. **Auto-detect lesson context**: Infer lesson_id from URL when in Docusaurus
5. **Hybrid search**: Combine semantic search with keyword filters

## Known Limitations

1. **Current test data**: Only Module 1 test chunks available (5 chunks)
2. **No indexes yet**: Lesson filtering not functional without Qdrant indexes
3. **Manual index creation**: Indexes must be created via Qdrant client
4. **No module filtering**: Only lesson-level filtering supported currently

## Conclusion

Cross-module search is **fully functional** and provides the best learning experience by default. Lesson-specific filtering is **implemented** but requires Qdrant indexes to be enabled in production.
