<div align="center">

# Physical AI & Humanoid Robotics Textbook

### An AI-Powered Interactive Learning Platform

[![GitHub Pages](https://img.shields.io/badge/Live_Site-GitHub_Pages-2ea44f?style=for-the-badge&logo=github)](https://anusbutt.github.io/Physical-AI-Humanoid-Robotics-Textbook/)
[![Backend](https://img.shields.io/badge/API-Hugging_Face_Spaces-FFD21E?style=for-the-badge&logo=huggingface)](https://huggingface.co/spaces)
[![Built with Docusaurus](https://img.shields.io/badge/Built_with-Docusaurus_v3-3ECC5F?style=for-the-badge&logo=docusaurus)](https://docusaurus.io/)
[![FastAPI](https://img.shields.io/badge/Backend-FastAPI-009688?style=for-the-badge&logo=fastapi)](https://fastapi.tiangolo.com/)

<br />

> Master the complete technology stack for building intelligent humanoid robots — from ROS 2 fundamentals to Vision-Language-Action models — with an AI assistant that learns from the textbook itself.

<br />

[Explore the Textbook](https://anusbutt.github.io/Physical-AI-Humanoid-Robotics-Textbook/) · [Report Bug](https://github.com/anusbutt/hackathon-phase-01/issues) · [Request Feature](https://github.com/anusbutt/hackathon-phase-01/issues)

</div>

---

## What Is This?

This isn't just a textbook — it's a **full-stack educational platform** where students learn Physical AI & Humanoid Robotics through four progressive modules, guided by a **RAG-powered AI chatbot** that understands every page of the book.

Select any paragraph. Click **AI Assistant**. Ask _"explain this"_. Get an answer grounded in the textbook content, with source citations back to the exact lesson.

<br />

## Key Features

| Feature | Description |
|---------|-------------|
| **RAG Chatbot** | Ask questions in natural language — answers are retrieved from the textbook's vector-indexed content |
| **Highlight & Ask** | Select text on any page, click the floating AI button, and get context-aware explanations |
| **Source Citations** | Every answer references the module, lesson, and section it came from |
| **Conversation Memory** | Multi-turn conversations with up to 5 messages of context, persisted for 7 days |
| **4 Progressive Modules** | From ROS 2 basics to cutting-edge VLA models — a complete robotics curriculum |
| **Dark Mode** | Designed for comfortable reading during long study sessions |

<br />

## Curriculum

```
Module 1 ─ The Robotic Nervous System (ROS 2)
│  Nodes, topics, pub/sub, URDF, launch files
│
Module 2 ─ Sensors & Perception
│  Cameras, LiDAR, IMU, depth sensing, sensor fusion
│
Module 3 ─ The AI-Robot Brain (NVIDIA Isaac)
│  Isaac Sim, synthetic data, Nav2, autonomous navigation
│
Module 4 ─ Vision-Language-Action Models
   Foundation models, VLAs, end-to-end autonomous behavior
```

Each lesson follows a structured format: **Concept → Why It Matters → Key Principles → Hands-On Example → Summary → Next Steps** — with AI colearning prompts and practice exercises throughout.

<br />

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    GitHub Pages                          │
│  ┌───────────────────────────────────────────────────┐  │
│  │          Docusaurus v3 (React / TypeScript)       │  │
│  │                                                   │  │
│  │   ┌──────────┐  ┌──────────────┐  ┌───────────┐  │  │
│  │   │ MDX Docs │  │ AI Chat UI   │  │ Text      │  │  │
│  │   │ (4 mods) │  │ (ChatInterface│  │ Selection │  │  │
│  │   │          │  │  + hooks)    │  │ Hook      │  │  │
│  │   └──────────┘  └──────┬───────┘  └───────────┘  │  │
│  └─────────────────────────┼─────────────────────────┘  │
└────────────────────────────┼────────────────────────────┘
                             │ REST API
                             ▼
┌─────────────────────────────────────────────────────────┐
│               FastAPI Backend (Docker)                   │
│                                                          │
│   ┌──────────┐    ┌──────────┐    ┌──────────────────┐  │
│   │ Rate     │───▶│ RAG      │───▶│ Gemini 2.5 Flash │  │
│   │ Limiter  │    │ Service  │    │ (LLM Generation) │  │
│   └──────────┘    └────┬─────┘    └──────────────────┘  │
│                        │                                 │
│              ┌─────────┴─────────┐                       │
│              ▼                   ▼                       │
│   ┌──────────────────┐  ┌────────────────┐              │
│   │ Cohere Embeddings│  │  Qdrant Cloud  │              │
│   │ (v3.0 / 1024d)  │  │ (Vector Store) │              │
│   └──────────────────┘  └────────────────┘              │
└──────────────────────────────────────────────────────────┘
```

<br />

## Tech Stack

| Layer | Technology | Purpose |
|:------|:-----------|:--------|
| **Frontend** | Docusaurus v3 + React 19 | Static site generation, interactive UI |
| **Language** | TypeScript 5.6 / Python 3.11+ | Type-safe frontend & backend |
| **Backend** | FastAPI | Async REST API for RAG pipeline |
| **LLM** | Google Gemini 2.5 Flash | Response generation |
| **Embeddings** | Cohere embed-english-v3.0 | 1024-dim semantic vectors |
| **Vector DB** | Qdrant Cloud | Similarity search over textbook chunks |
| **Database** | Neon Serverless Postgres | Conversation storage |
| **Containerization** | Docker | Reproducible backend deploys |
| **CI/CD** | GitHub Actions | Auto-deploy frontend (Pages) & backend (HF Spaces) |

<br />

## Getting Started

### Prerequisites

- **Node.js** >= 20.0 and **npm**
- **Python** >= 3.11
- **Git**

### Run the Frontend

```bash
git clone https://github.com/anusbutt/hackathon-phase-01.git
cd hackathon-phase-01/book-source
npm install
npm start
```

Opens at [http://localhost:3000](http://localhost:3000).

### Run the Backend

```bash
cd backend

# Create .env with required keys (see .env.example)
# GEMINI_API_KEY, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL

pip install -r requirements.txt
uvicorn app.main:app --reload --port 7860
```

API docs at [http://localhost:7860/docs](http://localhost:7860/docs).

<br />

## API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| `POST` | `/api/chat/query` | Send a question through the RAG pipeline |
| `POST` | `/api/chat/feedback` | Submit feedback on a conversation |
| `GET` | `/api/health` | Health check |

<br />

## Project Structure

```
hackathon-phase-01/
├── book-source/                    # Docusaurus frontend
│   ├── docs/                       # MDX textbook content (4 modules)
│   ├── src/
│   │   ├── components/             # ChatInterface, ChatInput, ChatMessage
│   │   ├── hooks/                  # useChatState, useTextSelection
│   │   └── theme/                  # Root.tsx (global chat button)
│   ├── static/                     # Images & diagrams
│   └── docusaurus.config.ts
│
├── backend/                        # FastAPI RAG backend
│   ├── app/
│   │   ├── api/chat.py             # REST endpoints
│   │   ├── services/
│   │   │   ├── rag_service.py      # RAG orchestration
│   │   │   ├── cohere_service.py   # Embedding generation
│   │   │   └── qdrant_service.py   # Vector search
│   │   ├── middleware/             # Rate limiting, error handling
│   │   ├── models/schemas.py      # Pydantic data models
│   │   └── config/settings.py     # Environment configuration
│   ├── Dockerfile
│   └── requirements.txt
│
├── .github/workflows/              # CI/CD pipelines
│   ├── deploy.yml                  # Frontend → GitHub Pages
│   └── deploy-hf-space.yml        # Backend → Hugging Face Spaces
│
├── specs/                          # Feature specifications (SDD)
└── history/                        # Prompt History Records
```

<br />

## Development Philosophy

This project follows **Spec-Driven Development (SDD)**:

- Specification first, implementation second
- Testable acceptance criteria for every feature
- Architectural Decision Records for significant choices
- Small, incremental, testable changes

<br />

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

<br />

## Team

**Panaversity Hackathon Team**

<br />

## License

This project is part of the Panaversity Hackathon. See [LICENSE](LICENSE) for details.

---

<div align="center">

**[Explore the Textbook](https://anusbutt.github.io/Physical-AI-Humanoid-Robotics-Textbook/)**

Built with Docusaurus, FastAPI, Gemini, Cohere & Qdrant

</div>
