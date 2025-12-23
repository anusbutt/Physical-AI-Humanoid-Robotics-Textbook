# Docusaurus Development Environment - Setup Guide

**Status**: ✅ Implemented (T013)
**Project**: `book-source/` (Docusaurus Classic TypeScript)
**Purpose**: Local development and testing of lesson content

---

## Prerequisites

✅ **Node.js**: v18+ (check with `node --version`)
✅ **npm**: v8+ (check with `npm --version`)
✅ **Git**: Installed and configured

---

## Quick Start

### 1. Install Dependencies

```bash
cd book-source
npm install
```

**Output**:
```
up to date, audited 1279 packages in 7s
found 0 vulnerabilities
```

### 2. Start Development Server

```bash
npm start
```

**Output**:
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

### 3. Open in Browser

Navigate to: **http://localhost:3000/**

The site will auto-reload when you save changes to markdown files.

---

## Available Commands

### Development

```bash
npm start
```
- Starts local development server on http://localhost:3000
- Hot reload enabled (auto-refresh on file changes)
- Fast refresh for quick iterations

### Build (Production)

```bash
npm run build
```
- Creates optimized production build in `book-source/build/`
- Validates all links and references
- Minifies assets for deployment
- **Use this before deploying to GitHub Pages**

### Serve (Test Production Build)

```bash
npm run serve
```
- Serves the production build locally
- Test deployment before pushing to GitHub Pages
- Runs on http://localhost:3000 by default

### Deploy (GitHub Pages)

```bash
npm run deploy
```
- Builds and deploys to GitHub Pages
- Requires GitHub repository configured in `docusaurus.config.ts`
- **Note**: Only run after user approval

---

## Project Structure

```
book-source/
├── docs/                          # All documentation content
│   └── 13-Physical-AI-Humanoid-Robotics/
│       ├── README.md              # Chapter overview
│       └── 01-ros2-nervous-system/
│           ├── README.md          # Module overview
│           ├── 01-ros2-fundamentals.md
│           ├── 02-nodes-topics-services.md
│           ├── 03-python-rclpy-bridge.md
│           ├── 04-urdf-humanoid-basics.md
│           ├── 05-capstone-project.md
│           └── 06-quiz.md
├── static/
│   └── img/                       # Images and assets
│       └── 13-Physical-AI-Humanoid-Robotics/
│           └── 01-ros2-nervous-system/
├── src/                           # React components (if needed)
├── sidebars.ts                    # Sidebar navigation config
├── docusaurus.config.ts           # Main config
└── package.json                   # Dependencies
```

---

## Configuration Files

### docusaurus.config.ts

Main configuration for the Docusaurus site:

```typescript
export default {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Learn to build intelligent robots',
  url: 'https://yourusername.github.io',
  baseUrl: '/repo-name/',
  // ... more config
}
```

**Key Settings**:
- `title`: Site title (appears in browser tab)
- `url`: GitHub Pages URL
- `baseUrl`: Repository name (e.g., `/Hackathon_Project/`)
- `organizationName`: GitHub username or org
- `projectName`: Repository name

### sidebars.ts

Navigation structure:

```typescript
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: '13. Physical AI & Humanoid Robotics',
      items: [
        {
          type: 'category',
          label: '01. The Robotic Nervous System (ROS 2)',
          items: [
            '13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/01-ros2-fundamentals',
            '13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/02-nodes-topics-services',
            // ... more lessons
          ],
        },
      ],
    },
  ],
};
```

---

## Development Workflow

### 1. Create New Lesson

1. Create markdown file in `docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/`
2. Add frontmatter (use template from `specs/001-ros2-nervous-system/frontmatter-template.yaml`)
3. Write content following lesson template
4. Save file

### 2. Preview Changes

1. Start dev server: `npm start`
2. Navigate to lesson in browser
3. Edit markdown file
4. See changes auto-reload in browser

### 3. Validate

1. Run markdown validation:
   ```bash
   ./.specify/scripts/bash/validate-markdown.sh docs/.../01-ros2-fundamentals.md
   ```
2. Run Technical Reviewer Agent validation (see `.specify/docs/technical-reviewer-usage.md`)
3. Fix any errors

### 4. Build for Production

1. Test production build:
   ```bash
   npm run build
   ```
2. Check output for errors
3. Serve locally to verify:
   ```bash
   npm run serve
   ```

---

## Troubleshooting

### Issue: `npm start` fails with port already in use

**Fix**: Kill process on port 3000 or use different port:
```bash
# Kill process on port 3000 (Linux/Mac)
lsof -ti:3000 | xargs kill -9

# Or use different port
PORT=3001 npm start
```

### Issue: Build fails with broken links

**Fix**: Check for:
- Incorrect file paths in markdown links
- Missing files referenced in sidebar
- Typos in link URLs

### Issue: Hot reload not working

**Fix**:
1. Stop server (Ctrl+C)
2. Clear cache: `rm -rf .docusaurus .cache`
3. Restart: `npm start`

### Issue: Images not loading

**Fix**:
- Images must be in `static/img/` directory
- Reference as `/img/path/to/image.png` (absolute path from static)
- Example: `/img/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/diagram.png`

---

## Performance Notes

**Build Time**: ~30 seconds for Module 1 (4 lessons)
**Dev Server Startup**: ~5 seconds
**Hot Reload**: <1 second for markdown changes

**Optimization Tips**:
- Use WebP format for images (smaller file size)
- Minimize number of large images per lesson
- Keep code examples concise (10-30 lines)

---

## GitHub Pages Deployment

### Prerequisites

1. Repository must be public (or GitHub Pro for private)
2. GitHub Pages enabled in repository settings
3. `docusaurus.config.ts` configured with correct `url` and `baseUrl`

### Deploy Process

```bash
# 1. Configure (if not done)
# Edit docusaurus.config.ts:
#   url: 'https://yourusername.github.io'
#   baseUrl: '/Hackathon_Project/'
#   organizationName: 'yourusername'
#   projectName: 'Hackathon_Project'

# 2. Build and deploy
cd book-source
npm run deploy
```

**What happens**:
1. Builds production site
2. Pushes to `gh-pages` branch
3. GitHub Pages serves from that branch

**Access deployed site**:
https://yourusername.github.io/Hackathon_Project/

---

## Environment Variables (Optional)

Create `.env` in `book-source/` for environment-specific config:

```bash
# .env
PORT=3001
BROWSER=none  # Don't auto-open browser
```

**Note**: `.env` files are gitignored by default.

---

## Related Documentation

- **Docusaurus Docs**: https://docusaurus.io/docs
- **Markdown Features**: https://docusaurus.io/docs/markdown-features
- **Configuration**: https://docusaurus.io/docs/configuration
- **Deployment**: https://docusaurus.io/docs/deployment

---

## Status

✅ **T013 Complete**: Local Docusaurus development environment setup
✅ **Dependencies installed**: 1279 packages, 0 vulnerabilities
✅ **Commands available**: `npm start`, `npm run build`, `npm run serve`, `npm run deploy`

📍 **Phase 2 (Foundational) Complete**: All 7 tasks done! Ready to begin lesson content creation in Phase 3.

---

## Next Steps

Now that the development environment is ready, you can:

1. **Start Lesson 1 (Phase 3)**: Begin implementing T014-T029 (ROS2 Fundamentals)
2. **Test the setup**: Run `npm start` and verify http://localhost:3000 loads
3. **Create first lesson**: Follow the lesson template to build content

**Checkpoint Reached**: Foundation ready - lesson content creation can now begin! 🎯
