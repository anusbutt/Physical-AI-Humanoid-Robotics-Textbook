# RAG Chatbot Deployment Guide

## Architecture Overview

```
GitHub Pages (Frontend)          Hugging Face Spaces (Backend)
anusbutt.github.io/        -->  anusbutt-rag-chatbot.hf.space
  hackathon-phase-01/               |
  Docusaurus static site            FastAPI + Docker
                                    |
                              +-----+-----+
                              |     |     |
                           Qdrant Cohere Gemini
                           Cloud  API    API
```

## Backend Deployment (Hugging Face Spaces)

### Prerequisites
- Hugging Face account
- API keys: Gemini, Cohere, Qdrant Cloud, Neon Postgres

### Step 1: Create HF Space
1. Go to https://huggingface.co/spaces
2. Create new Space: `rag-chatbot`
3. Select **Docker** SDK
4. Set visibility to Public

### Step 2: Configure Secrets
In Space Settings > Repository Secrets, add:

| Secret | Description |
|--------|-------------|
| `GEMINI_API_KEY` | Google Gemini API key |
| `GEMINI_BASE_URL` | `https://generativelanguage.googleapis.com/v1beta/openai/` |
| `GEMINI_MODEL` | `gemini-2.5-flash` |
| `COHERE_API_KEY` | Cohere API key |
| `COHERE_EMBEDDING_MODEL` | `embed-english-v3.0` |
| `QDRANT_URL` | Qdrant Cloud cluster URL |
| `QDRANT_API_KEY` | Qdrant Cloud API key |
| `QDRANT_COLLECTION_NAME` | `humanoid_robotics_book` |
| `NEON_DATABASE_URL` | Neon Postgres connection string |
| `CORS_ORIGINS` | `https://anusbutt.github.io,https://anusbutt-rag-chatbot.hf.space` |
| `API_RATE_LIMIT` | `100` |

### Step 3: Deploy
Push the `backend/` directory to the HF Space repo. The Dockerfile auto-builds:
- Base: `python:3.11-slim`
- Exposes port 7860
- Runs: `uvicorn app.main:app --host 0.0.0.0 --port 7860`

### Step 4: Verify
```bash
curl https://anusbutt-rag-chatbot.hf.space/api/health
curl -X POST https://anusbutt-rag-chatbot.hf.space/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS2?"}'
```

## Embedding Pipeline (One-Time Setup)

Run from `backend/` directory:
```bash
python scripts/embed_content.py --module all
```

This embeds all 4 modules (~365 chunks) into the Qdrant `humanoid_robotics_book` collection.

## Frontend Deployment (GitHub Pages)

Frontend auto-deploys via GitHub Actions on push to `main`. The backend URL is configured in `book-source/src/services/chatApi.ts`.

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Cold starts (~30s) | Normal for free tier. First request after idle wakes the Space. |
| Port errors | HF Spaces requires port 7860. Configured in Dockerfile. |
| CORS blocked | Verify `CORS_ORIGINS` secret includes your frontend domain. |
| Rate limit 429 | Default 100 req/hour per session. Check `API_RATE_LIMIT`. |
| Qdrant 404 | Cluster may be paused. Reactivate at cloud.qdrant.io. |

## Free Tier Limits

| Service | Limit |
|---------|-------|
| Gemini | 1,500 req/day |
| Cohere | 10,000 embeddings/month |
| Qdrant Cloud | 1GB storage, auto-pause after inactivity |
| Neon Postgres | 0.5GB storage |
| HF Spaces | CPU basic, auto-sleep after 48h inactivity |
