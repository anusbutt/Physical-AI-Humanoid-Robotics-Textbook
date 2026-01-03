# Railway Deployment Guide

## Prerequisites

- ✅ Backend code complete (Phases 1-7)
- ✅ GitHub repository: `anusbutt/hackathon-phase-01`
- ✅ Railway account (free tier available)
- ✅ API keys ready (Gemini, Cohere, Qdrant, Neon)

## Step 1: Create Railway Account

1. Go to https://railway.app/
2. Click "Start a New Project"
3. Sign in with GitHub
4. Authorize Railway to access your repositories

## Step 2: Create New Project

1. Click "New Project"
2. Select "Deploy from GitHub repo"
3. Choose `anusbutt/hackathon-phase-01`
4. Railway will detect the backend directory automatically

## Step 3: Configure Root Directory

Since the backend code is in `backend/` subdirectory:

1. Go to project **Settings**
2. Find "Root Directory" setting
3. Set to: `backend`
4. Click "Save"

## Step 4: Add Environment Variables

Go to project **Variables** tab and add all the following:

### Required Variables

```bash
# Google Gemini (LLM)
GEMINI_API_KEY=your_gemini_api_key_here
GEMINI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/
GEMINI_MODEL=gemini-2.5-flash

# Cohere (Embeddings)
COHERE_API_KEY=your_cohere_api_key_here
COHERE_EMBEDDING_MODEL=embed-english-v3.0

# Qdrant Cloud (Vector Database)
QDRANT_URL=your_qdrant_cluster_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=humanoid_robotics_book

# Neon Postgres (Database)
NEON_DATABASE_URL=your_neon_connection_string_here

# CORS Configuration
CORS_ORIGINS=https://anusbutt.github.io,http://localhost:3000

# API Configuration
API_RATE_LIMIT=100
```

### How to Add Variables

1. Click "New Variable"
2. Enter variable name (e.g., `GEMINI_API_KEY`)
3. Enter variable value
4. Click "Add"
5. Repeat for all variables above

### Get Your API Keys

**Gemini API Key**:
- Go to: https://ai.google.dev/
- Create project and enable Gemini API
- Generate API key
- Copy: `AIzaSy...`

**Cohere API Key**:
- Go to: https://dashboard.cohere.com/
- Sign up for free tier
- Go to API Keys
- Copy your API key

**Qdrant URL & API Key**:
- Already have cluster at: `https://c03f24dc-3829-45f8-8520-09acc4c0da6b.us-east4-0.gcp.cloud.qdrant.io`
- API key from Qdrant dashboard

**Neon Database URL**:
- Already have database
- Connection string format: `postgresql://user:password@host/dbname?sslmode=require`

## Step 5: Deploy

Railway will automatically:
1. Detect `railway.toml` configuration
2. Install dependencies from `requirements.txt`
3. Start the app with `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

### Monitor Deployment

1. Go to **Deployments** tab
2. Watch build logs in real-time
3. Look for:
   ```
   [SUCCESS] Chat services initialized successfully
   INFO: Application startup complete.
   ```

### Deployment Files

Railway uses these files:
- `railway.toml` - Railway-specific configuration
- `Procfile` - Process startup command (fallback)
- `runtime.txt` - Python version (3.11)
- `requirements.txt` - Python dependencies

## Step 6: Get Your Public URL

Once deployed:

1. Go to **Settings** tab
2. Find "Domains" section
3. Railway auto-generates a URL like:
   ```
   https://hackathon-phase-01-production.up.railway.app
   ```
4. Or click "Generate Domain" to create custom subdomain

## Step 7: Verify Deployment

### Test Health Endpoint

```bash
curl https://your-railway-url.railway.app/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "timestamp": "2026-01-02T12:00:00",
  "version": "1.0.0",
  "services": {
    "api": true
  }
}
```

### Test Chat Endpoint

```bash
curl -X POST https://your-railway-url.railway.app/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS2?", "conversation_history": []}'
```

Expected response:
```json
{
  "response": "ROS2 (Robot Operating System 2) is...",
  "sources": [...],
  "conversation_id": "uuid-here",
  "timestamp": "2026-01-02T12:00:00",
  "retrieved_chunks": 5,
  "used_selected_text": false
}
```

## Step 8: Update Frontend CORS

Once you have your Railway URL, update `CORS_ORIGINS`:

```bash
CORS_ORIGINS=https://anusbutt.github.io,https://your-railway-url.railway.app
```

Or update in code (`backend/app/main.py`):
```python
CORS_ORIGINS = [
    "https://anusbutt.github.io",
    "https://your-railway-url.railway.app",
    "http://localhost:3000"
]
```

## Troubleshooting

### Build Fails

**Problem**: Dependencies not installing
**Solution**: Check `requirements.txt` is in `backend/` directory

**Problem**: Python version mismatch
**Solution**: Verify `runtime.txt` specifies `python-3.11`

### App Crashes on Startup

**Problem**: Missing environment variables
**Solution**: Verify all required variables are set in Railway dashboard

**Problem**: Port binding error
**Solution**: Ensure using `--port $PORT` (Railway sets this automatically)

### API Errors

**Problem**: 503 errors on startup
**Solution**: Check logs for service initialization errors (Qdrant, Gemini, Cohere)

**Problem**: CORS errors from frontend
**Solution**: Update `CORS_ORIGINS` to include Railway URL

### Check Logs

1. Go to **Deployments** tab
2. Click on latest deployment
3. View "Deploy Logs" or "Runtime Logs"
4. Look for error messages

## Railway Free Tier Limits

- ✅ $5/month usage credit
- ✅ 500 MB RAM
- ✅ 1 GB storage
- ✅ Always-on (no sleep)
- ⚠️ Usage beyond $5/month requires payment

## Cost Optimization

1. **Use Free Tiers**: Gemini, Cohere, Qdrant, Neon all have free tiers
2. **Monitor Usage**: Check Railway dashboard for resource usage
3. **Optimize Queries**: Cache frequent queries, limit context window
4. **Scale Later**: Start with free tier, upgrade if needed

## Production Checklist

Before going live:

- [ ] All environment variables configured
- [ ] Health endpoint returns 200 OK
- [ ] Chat endpoint processes queries successfully
- [ ] CORS allows your frontend domain
- [ ] Error handling tested (out-of-scope queries, rate limits)
- [ ] Monitoring/logging set up
- [ ] Railway URL added to frontend config

## Next Steps

After successful deployment:

1. **Update Frontend** (Phase 9):
   - Add Railway URL to Docusaurus config
   - Update API client to use production URL

2. **Monitor Performance**:
   - Check Railway metrics dashboard
   - Watch for errors in logs

3. **Test in Production**:
   - Visit frontend: https://anusbutt.github.io/hackathon-phase-01/
   - Open chat interface
   - Test queries

## Deployment Commands (Alternative)

If you prefer Railway CLI:

```bash
# Install Railway CLI
npm install -g @railway/cli

# Login
railway login

# Link project
railway link

# Set environment variables
railway variables set GEMINI_API_KEY=your_key_here

# Deploy
railway up
```

## Support

- Railway Docs: https://docs.railway.app/
- Railway Discord: https://discord.gg/railway
- GitHub Issues: https://github.com/anusbutt/hackathon-phase-01/issues

---

## Summary

✅ **Easy Deployment**: Push to GitHub → Railway auto-deploys
✅ **Free Tier**: $5/month credit covers development
✅ **Auto HTTPS**: Railway provides SSL certificates
✅ **Zero Config**: Nixpacks detects Python automatically
✅ **Environment Variables**: Secure secret management
✅ **Logs**: Real-time deployment and runtime logs

**Your backend will be live at**: `https://your-app.up.railway.app`
