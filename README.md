# Physical AI & Humanoid Robotics - Hackathon Phase 01

> **Panaversity Hackathon Project**: Comprehensive educational platform teaching the complete technology stack for building intelligent humanoid robots

## 📚 Project Overview

This is a Docusaurus-based educational book designed for CS students learning Physical AI and Humanoid Robotics. The curriculum covers four progressive modules:

### Module Structure

1. **Module 1: The Robotic Nervous System (ROS 2)**
   - Introduction to ROS 2 architecture
   - Pub/Sub communication patterns
   - Robot description with URDF
   - Integration and best practices

2. **Module 2: Sensors & Perception Systems**
   - Camera systems and computer vision
   - Depth sensing (LiDAR, structured light, ToF)
   - IMU and proprioception
   - Sensor fusion and state estimation

3. **Module 3: The AI-Robot Brain (NVIDIA Isaac™)**
   - Isaac Sim for photorealistic simulation
   - Isaac ROS for hardware-accelerated perception
   - Nav2 path planning for bipedal movement
   - Autonomous navigation system integration

4. **Module 4: Vision-Language-Action Models**
   - Foundation models for robotics
   - Vision-language integration
   - Action planning with VLA models
   - End-to-end autonomous behavior

## 🛠️ Technology Stack

- **Platform**: Docusaurus v3.9.2 (TypeScript)
- **Content**: MDX (Markdown + JSX components)
- **Deployment**: GitHub Pages
- **Development**: Node.js >=20.0, npm
- **Conceptual Technologies**: ROS 2 Humble, Python 3.11+, NVIDIA Isaac, VLA models

## 🚀 Quick Start

### Prerequisites
- Node.js >= 20.0
- npm or yarn
- Git

### Installation

```bash
# Clone the repository
git clone https://github.com/anusbutt/hackathon-phase-01.git
cd hackathon-phase-01

# Navigate to book source
cd book-source

# Install dependencies
npm install

# Start development server
npm start
```

The site will open at `http://localhost:3000`

### Build for Production

```bash
npm run build
```

## 📖 Documentation Structure

Each module follows a consistent 7-section lesson structure:

1. **What Is [Concept]?** - Core explanation (400-500 words)
2. **Why [Concept] Matters** - Relevance to Physical AI (400-500 words)
3. **Key Principles** - 3-5 fundamental concepts
4. **Practical Example** - Hands-on code/configuration (300-400 words)
5. **Summary** - Key takeaways (200-300 words)
6. **Next Steps** - Preview of next lesson (100-150 words)

### Special Features

- 💬 **AI Colearning Prompts**: Explore concepts with Claude/ChatGPT
- 🎓 **Expert Insights**: Advanced perspectives and common pitfalls
- 🤝 **Practice Exercises**: Hands-on learning tasks
- **RAG-Ready Metadata**: Rich frontmatter for future chatbot integration

## 📁 Project Structure

```
hackathon-phase-01/
├── book-source/                  # Docusaurus project
│   ├── docs/                     # Content files
│   │   └── 13-Physical-AI-Humanoid-Robotics/
│   │       ├── README.md         # Chapter overview
│   │       ├── 01-ros2-nervous-system/
│   │       ├── 02-sensors-perception/
│   │       ├── 03-isaac-ai-brain/
│   │       └── 04-vision-language-action/
│   ├── static/                   # Images, diagrams
│   ├── src/                      # React components
│   ├── docusaurus.config.ts      # Site configuration
│   └── sidebars.ts               # Navigation structure
├── specs/                        # Feature specifications
├── .specify/                     # SpecKit Plus templates
├── CLAUDE.md                     # Development guidelines
└── README.md                     # This file
```

## 🎯 Development Philosophy

This project follows **Spec-Driven Development (SDD)** principles:

- ✅ Specification first, implementation second
- ✅ Testable acceptance criteria for all features
- ✅ Architectural Decision Records (ADRs) for significant choices
- ✅ Prompt History Records (PHRs) for development tracking
- ✅ Small, incremental, testable changes

## 📝 Contributing

This project uses a structured development workflow:

1. Each module has its own feature branch
2. Changes are reviewed via Pull Requests
3. All PRs merge into the `main` branch
4. GitHub Actions auto-deploys to GitHub Pages

### Branch Structure

- `main` - Production-ready content
- `module-01-ros2-nervous-system` - Module 1 development
- `module-02-sensors-perception` - Module 2 development
- `module-03-isaac-ai-brain` - Module 3 development
- `module-04-vision-language-action` - Module 4 development

## 📄 License

[Add your license here]

## 👥 Authors

**Panaversity Hackathon Team**

---

🤖 *Generated for Panaversity Hackathon Phase 01* | 📅 December 2025
