# Railway Deployment Checklist

## Pre-Deployment Checks

### ✅ Code Ready
- [x] All Phases 1-7 complete
- [x] Error handling implemented
- [x] CORS middleware configured
- [x] Health endpoint functional
- [x] Chat API tested locally

### ✅ Configuration Files
- [x] `railway.toml` - Railway configuration
- [x] `Procfile` - Process startup command
- [x] `runtime.txt` - Python 3.11 specified
- [x] `requirements.txt` - All dependencies listed
- [x] `.gitignore` - Secrets not committed
- [x] `.dockerignore` - Unnecessary files excluded
- [x] `.env.example` - Template for environment variables
- [x] `.env.production.example` - Production variables template

### ✅ API Keys Ready
- [ ] Gemini API key (`GEMINI_API_KEY`)
- [ ] Cohere API key (`COHERE_API_KEY`)
- [ ] Qdrant URL (`QDRANT_URL`)
- [ ] Qdrant API key (`QDRANT_API_KEY`)
- [ ] Neon database URL (`NEON_DATABASE_URL`)

### ✅ Database Setup
- [x] Qdrant collection created: `humanoid_robotics_book`
- [x] Test data uploaded (5 chunks minimum)
- [x] Postgres schema created (users, conversations, user_progress)

## Deployment Steps

### Step 1: Railway Account Setup
- [ ] Railway account created
- [ ] GitHub connected to Railway
- [ ] Repository access granted

### Step 2: Create Railway Project
- [ ] New project created
- [ ] Repository `anusbutt/hackathon-phase-01` selected
- [ ] Root directory set to `backend`

### Step 3: Environment Variables
Add to Railway dashboard (Settings > Variables):

```bash
GEMINI_API_KEY=____________
GEMINI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/
GEMINI_MODEL=gemini-2.5-flash

COHERE_API_KEY=____________
COHERE_EMBEDDING_MODEL=embed-english-v3.0

QDRANT_URL=____________
QDRANT_API_KEY=____________
QDRANT_COLLECTION_NAME=humanoid_robotics_book

NEON_DATABASE_URL=____________

CORS_ORIGINS=https://anusbutt.github.io,http://localhost:3000
API_RATE_LIMIT=100
```

- [ ] All 11 variables added
- [ ] Values verified (no typos)
- [ ] Sensitive keys secured

### Step 4: Deploy
- [ ] Push code to `main` branch
- [ ] Railway auto-deploy triggered
- [ ] Build logs monitored
- [ ] Deployment successful

### Step 5: Get Public URL
- [ ] Railway domain generated
- [ ] URL copied: `https://____________.railway.app`
- [ ] CORS_ORIGINS updated with Railway URL

## Post-Deployment Verification

### Health Check
```bash
curl https://your-app.railway.app/api/health
```

Expected:
```json
{
  "status": "healthy",
  "timestamp": "...",
  "version": "1.0.0",
  "services": {"api": true}
}
```

- [ ] Health endpoint returns 200 OK
- [ ] Response includes timestamp and version

### Chat API Test
```bash
curl -X POST https://your-app.railway.app/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS2?", "conversation_history": []}'
```

Expected:
```json
{
  "response": "ROS2...",
  "sources": [...],
  "conversation_id": "...",
  "retrieved_chunks": 5,
  "used_selected_text": false
}
```

- [ ] Chat endpoint returns 200 OK
- [ ] Response includes AI-generated answer
- [ ] Sources array populated
- [ ] Conversation ID present

### Error Handling Test
```bash
curl -X POST https://your-app.railway.app/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is the weather?", "conversation_history": []}'
```

Expected:
- Response suggests course topics (ROS2, Sensors, etc.)
- No sources (out-of-scope query)

- [ ] Out-of-scope queries handled gracefully
- [ ] Helpful suggestions provided

### CORS Test
From browser console at `https://anusbutt.github.io`:
```javascript
fetch('https://your-app.railway.app/api/health')
  .then(r => r.json())
  .then(console.log)
```

- [ ] No CORS errors
- [ ] Response received successfully

## Production Monitoring

### Railway Dashboard Checks
- [ ] CPU usage < 50%
- [ ] Memory usage < 400MB
- [ ] No crash loops
- [ ] Logs show successful requests

### Service Health
- [ ] Gemini API responding (check logs)
- [ ] Cohere API responding
- [ ] Qdrant connection stable
- [ ] Postgres connection stable

### Logging
- [ ] Application logs visible in Railway
- [ ] Errors logged with stack traces
- [ ] Request/response cycles logged
- [ ] Service initialization logged

## Rollback Plan

If deployment fails:

1. **Check Logs**: Railway > Deployments > View Logs
2. **Verify Variables**: Settings > Variables (all 11 present?)
3. **Rollback**: Deployments > Previous deployment > Redeploy
4. **Local Test**: Run `uvicorn app.main:app --port 8001` locally
5. **Fix & Redeploy**: Fix issue, push to main

## Cost Monitoring

- [ ] Railway usage dashboard checked
- [ ] Current month usage: $_____ / $5.00
- [ ] Estimated monthly cost within free tier
- [ ] Alerts set up for usage thresholds

## Next Steps After Deployment

1. **Update Frontend** (Phase 9):
   - [ ] Add Railway URL to Docusaurus config
   - [ ] Update API client baseURL
   - [ ] Test chat interface with production backend

2. **Documentation**:
   - [ ] Update README with live demo URL
   - [ ] Add deployment status badge
   - [ ] Document API endpoints

3. **Monitoring**:
   - [ ] Set up error tracking (Sentry, etc.)
   - [ ] Configure uptime monitoring
   - [ ] Set up usage alerts

## Troubleshooting Guide

### Build Fails
- Check `requirements.txt` in `backend/` directory
- Verify Python version in `runtime.txt`
- Check Railway build logs for errors

### App Crashes
- Verify all environment variables set
- Check logs for missing imports
- Ensure port binding uses `$PORT`

### API Errors
- Test each service individually (Gemini, Cohere, Qdrant, Postgres)
- Verify API keys are valid
- Check CORS configuration

### Deployment Successful But App Not Responding
- Check Railway logs for startup errors
- Verify Railway domain is correct
- Test health endpoint directly

## Success Criteria

✅ **Deployment Complete When**:
- Health endpoint returns 200 OK
- Chat API processes queries successfully
- No errors in Railway logs
- CORS allows frontend access
- All environment variables working
- Within free tier limits

---

**Current Status**: ✅ Deployed Successfully

**Railway URL**: `https://virtuous-creativity-production.up.railway.app`

**Deployed By**: anusbutt

**Deployment Date**: January 3, 2026
