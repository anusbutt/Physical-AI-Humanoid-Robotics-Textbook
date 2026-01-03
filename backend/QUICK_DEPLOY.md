# Quick Deploy to Railway (5 Minutes)

## Prerequisites
- GitHub account
- API keys ready (see `.env.example`)

## Step 1: Go to Railway (1 min)
1. Visit: https://railway.app/
2. Click "Start a New Project"
3. Sign in with GitHub

## Step 2: Deploy from GitHub (2 min)
1. Select "Deploy from GitHub repo"
2. Choose `anusbutt/hackathon-phase-01`
3. Railway detects Python app automatically
4. Go to Settings → Set Root Directory: `backend`

## Step 3: Add Environment Variables (2 min)
Go to Variables tab, add these 11 variables:

```bash
GEMINI_API_KEY=your_key_here
GEMINI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/
GEMINI_MODEL=gemini-2.5-flash

COHERE_API_KEY=your_key_here
COHERE_EMBEDDING_MODEL=embed-english-v3.0

QDRANT_URL=https://your-cluster.cloud.qdrant.io
QDRANT_API_KEY=your_key_here
QDRANT_COLLECTION_NAME=humanoid_robotics_book

NEON_DATABASE_URL=postgresql://user:pass@host/db?sslmode=require

CORS_ORIGINS=https://anusbutt.github.io
API_RATE_LIMIT=100
```

## Step 4: Deploy & Test (< 1 min)
Railway auto-deploys. Once done:

```bash
# Test health
curl https://your-app.railway.app/api/health

# Test chat
curl -X POST https://your-app.railway.app/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS2?"}'
```

## Done! 🎉
Your backend is live at: `https://your-app.railway.app`

Next: Update frontend to use this URL (Phase 9)

---

**Full Guide**: See [RAILWAY_DEPLOYMENT.md](./RAILWAY_DEPLOYMENT.md)
**Checklist**: See [DEPLOYMENT_CHECKLIST.md](./DEPLOYMENT_CHECKLIST.md)
