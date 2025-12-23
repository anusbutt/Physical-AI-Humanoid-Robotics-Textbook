# Quickstart: Module 1 Implementation

**Purpose**: Guide for implementing Module 1 (ROS2 Nervous System) following the 9-step agent pipeline
**Target**: Software engineer executing `/sp.tasks` and `/sp.implement`
**Prerequisites**: Constitution v1.0.0, spec.md approved, plan.md complete

## Overview

Module 1 creates 4 lessons about ROS2 fundamentals. Implementation follows a **manual → template → agent** progression (YAGNI principle).

**Timeline**: 4 lessons × ~6 hours each = ~24 hours total (manual for Lesson 1, semi-automated for 2-4)

## Implementation Phases

### Phase 1: Lesson 1 (Manual - Establish Pattern)

**Goal**: Create "golden example" following all templates manually

**Steps**:
1. **Create outline** (1 hr) - Follow 7-section template from research.md
2. **Write main content** (2 hrs) - Fill outline with prose, code examples, callouts
3. **Add case study** (30 min) - Real-world robotics example
4. **Write code examples** (30 min) - 2-3 Python rclpy snippets
5. **Technical review** (Manual, 45 min) - Validate ROS2 accuracy using Technical Reviewer checklist
6. **Style check** (30 min) - Heading hierarchy, frontmatter, summary file
7. **Generate frontmatter** (30 min) - Fill YAML schema
8. **Docusaurus integration** (15 min) - Test MDX rendering locally
9. **User review** (15 min) - Final approval

**Deliverables**:
- `01-ros2-fundamentals.md` (2000 words)
- `01-ros2-fundamentals.summary.md` (150 words)
- Frontmatter metadata (13 fields)
- Time log (document hours spent on each step)

### Phase 2: Lessons 2-3 (Template - Refine Pattern)

**Goal**: Copy-paste-modify Lesson 1, refine templates

**Efficiency Gains**: ~4 hours per lesson (vs 6 for Lesson 1)

**Process**:
1. Duplicate Lesson 1 files
2. Replace topic-specific content (ROS2 basics → Nodes/Topics → rclpy)
3. Update frontmatter (title, learning objectives, tags)
4. Run Technical Reviewer Agent (if implemented)
5. Manual review + approval

### Phase 3: Lesson 4 (Agent - Automate Pattern)

**Goal**: Build agents if pattern stabilized

**Decision Point**: If Lessons 1-3 follow consistent structure, implement:
- Outline Agent (generates 7-section skeleton)
- Frontmatter Agent (auto-fills YAML)
- Structure & Style Agent (validates headings, checks summary file)

**Efficiency Gains**: ~3 hours per lesson

### Phase 4: Capstone + Quiz

**Capstone** (2 hrs):
- Design conceptual exercise integrating all 4 lessons
- Evaluation rubric with 4-5 criteria

**Quiz** (1 hr):
- 15 questions (8 MC, 5 short answer, 2 code reading)
- Answer key + grading rubric

## File Checklist

**Per Lesson** (4 lessons × 2 files = 8 files):
- [ ] `0#-topic-name.md` (main content, 1500-2500 words)
- [ ] `0#-topic-name.summary.md` (summary, 100-200 words)

**Module Assets**:
- [ ] `README.md` (module overview, 300 words)
- [ ] `05-capstone-project.md` + `.summary.md`
- [ ] `06-quiz.md`

**Total**: 11 files

## Quality Gates

**Before Commit** (per lesson):
1. ✅ Word count: 1500-2500 (main), 100-200 (summary)
2. ✅ Frontmatter: All 13 fields populated
3. ✅ Code examples: 2-3 snippets, 10-30 lines each, type hints
4. ✅ Callouts: 2-3 per lesson (💬🎓🤝)
5. ✅ Technical Reviewer: PASS (or documented issues resolved)
6. ✅ Markdown linting: No syntax errors
7. ✅ File naming: Matches `##-topic.md` + `.summary.md` pattern

**Before Merge**:
1. ✅ All 4 lessons complete
2. ✅ Capstone + quiz complete
3. ✅ Module README exists
4. ✅ User approval (final review)

## Git Workflow

**Branch**: `001-ros2-nervous-system` (already created)

**Commit Strategy** (after each pipeline step):
```bash
# After outline
git add book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/01-ros2-fundamentals.md
git commit -m "docs(module-01): add outline for ROS2 fundamentals lesson"

# After content
git commit -m "docs(module-01): add main content for ROS2 fundamentals"

# After technical review
git commit -m "docs(module-01): fix technical review feedback for ROS2 fundamentals"

# After lesson complete
git commit -m "docs(module-01): complete lesson 1 (ROS2 fundamentals)"
```

**Merge to main**: After all 4 lessons + capstone + quiz complete and user approved

## Agent Pipeline (when automated)

```
1. Outline Agent → generates 7-section skeleton
2. Content Agent → fills prose (NOT implemented in Module 1)
3. Case Study Agent → adds real-world example (NOT implemented in Module 1)
4. Code Example Agent → generates rclpy snippets (NOT implemented in Module 1)
5. Technical Reviewer Agent → validates accuracy (IMPLEMENT ASAP)
6. Structure & Style Agent → checks formatting
7. Frontmatter Agent → auto-fills YAML
8. Docusaurus Agent → tests MDX rendering
9. User Review → final approval
```

**Module 1 Reality**: Steps 1-4 and 6-8 are manual or semi-automated. Only Technical Reviewer Agent is fully implemented.

## Local Testing

**Docusaurus Development Server**:
```bash
cd book-source
npm install
npm start
# Opens http://localhost:3000
# Navigate to: Physical AI > Module 1 > Lesson 1
```

**Validation**:
- Check frontmatter renders correctly
- Code blocks have syntax highlighting
- Navigation (prev/next) works
- Images load (if added)

## Success Criteria

**Lesson-Level**:
- Student can complete in <45 min (SC-001)
- 80% quiz success rate (SC-001)
- Technical Reviewer passes (SC-005)

**Module-Level**:
- All 4 lessons deployed to GitHub Pages (SC-006)
- Zero formatting errors (SC-006)
- Content supports RAG queries (SC-008)

## Troubleshooting

**Problem**: Technical Reviewer finds critical issues
**Solution**: Fix immediately, re-run validation, don't proceed to next step

**Problem**: Lesson too long (>2500 words)
**Solution**: Split into main content + "extension for advanced" in frontmatter

**Problem**: Code example doesn't parse
**Solution**: Run Python syntax check: `python -m py_compile example.py`

**Problem**: Frontmatter missing fields
**Solution**: Use data-model.md schema as checklist

## Next Steps

After Module 1 complete:
1. User reviews deployed GitHub Pages
2. Collect feedback (what worked, what didn't)
3. Update templates based on learnings
4. Create spec for Module 2 (`/sp.specify` again)
5. Iterate SDD cycle for Modules 2-4

**Phase 1 Complete**: All 16 lessons → transition to Constitution v2.0.0 (chatbot, personalization)
