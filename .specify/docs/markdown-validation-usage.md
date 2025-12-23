# Markdown Validation Script - Usage Guide

**Status**: ✅ Implemented (T012)
**Script**: `.specify/scripts/bash/validate-markdown.sh`
**Purpose**: Validate lesson markdown files for structural quality before committing

---

## Quick Start

### Basic Usage

```bash
./.specify/scripts/bash/validate-markdown.sh book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/01-ros2-fundamentals.md
```

### JSON Output

```bash
./.specify/scripts/bash/validate-markdown.sh book-source/docs/.../01-ros2-fundamentals.md --json
```

---

## What It Validates

### 1. Frontmatter Schema (13 Required Fields)

All lesson files MUST have these YAML frontmatter fields:

- `id` - Unique lesson identifier
- `title` - Lesson title
- `sidebar_position` - Navigation order (1-6)
- `description` - Brief summary (1-2 sentences)
- `tags` - Array of topic tags
- `skills` - Array of skills taught
- `learning_objectives` - Array of objectives
- `time_estimate` - Estimated completion time (e.g., "45 minutes")
- `difficulty_level` - "beginner", "intermediate", or "advanced"
- `cognitive_load` - "low", "medium", or "high"
- `prerequisites` - Array of prerequisite knowledge
- `related_lessons` - Array of related lesson IDs
- `assessment_method` - How learning is assessed

**Example Error**:
```
✗ Missing required frontmatter field: learning_objectives
✗ Missing required frontmatter field: time_estimate
```

### 2. Heading Hierarchy

**Rules**:
- Exactly **one H1** heading per file (the lesson title)
- No heading level skips (H2 → H4 is invalid, must go H2 → H3)
- Consistent heading structure for SEO and accessibility

**Example Error**:
```
✗ Multiple H1 headings found (2). Only one H1 allowed per lesson.
✗ Heading level skip at line 45: jumped from H2 to H4
```

### 3. Word Count (1500-2500 words)

**Calculation**:
- Excludes frontmatter YAML
- Excludes code blocks (```...```)
- Counts only prose content

**Targets**:
- **Minimum**: 1500 words (content too brief)
- **Maximum**: 2500 words (content too dense for beginners)

**Example Warning**:
```
⚠ Word count too low: 1200 words (minimum: 1500)
```

### 4. Callout Patterns (2-3 expected)

**Expected Callouts**:
- 💬 **AI Colearning Prompt**: "Ask Claude to explain..."
- 🎓 **Expert Insight**: Advanced concepts or best practices
- 🤝 **Practice Exercise**: Hands-on activity for students

**Example Warning**:
```
⚠ Few callouts found (1). Expected at least 2-3 callouts
```

### 5. Code Blocks (for technical lessons)

If the lesson mentions "rclpy", "Python", or "URDF", it should have code examples.

**Example Warning**:
```
⚠ No code blocks found in lesson that mentions rclpy/Python/URDF
```

---

## Output Formats

### Human-Readable Output

```
====================================
Markdown Validation Report
====================================
File: book-source/docs/.../01-ros2-fundamentals.md

INFO:
  ✓ Word count: 1850 words (target: 1500-2500)

WARNINGS:
  ⚠ Few callouts found (1). Expected at least 2-3 callouts

ERRORS:
  ✗ Missing required frontmatter field: time_estimate

Status: FAIL ✗
```

### JSON Output (for automation)

```json
{
  "file": "book-source/docs/.../01-ros2-fundamentals.md",
  "word_count": 1850,
  "h1_count": 1,
  "callout_count": 1,
  "errors": [
    "Missing required frontmatter field: time_estimate"
  ],
  "warnings": [
    "Few callouts found (1). Expected at least 2-3 callouts"
  ],
  "info": [
    "Word count: 1850 words (target: 1500-2500)"
  ],
  "status": "FAIL"
}
```

---

## Workflow Integration

### In Tasks (e.g., T028, T044, T063, T081)

Tasks reference validation like this:
```
T028 [US1] Run markdown validation (word count 1500-2500, heading hierarchy, frontmatter schema)
```

**How to execute**:
1. Complete lesson content
2. Run validation script
3. Fix any errors (critical - blocks progression)
4. Review warnings (fix if appropriate)
5. Mark task complete ✅

### Example: Validating Lesson 1

```bash
# After completing T014-T025 (Lesson 1 content)

# T028: Run validation
./.specify/scripts/bash/validate-markdown.sh \
  book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/01-ros2-fundamentals.md

# Review output
# Fix errors (if any)
# Re-run validation

# Mark complete
# T028 ✅
```

---

## Validation Severity

### Errors (MUST fix - blocks progression)
- Missing frontmatter fields
- Multiple H1 headings
- Heading level skips

### Warnings (SHOULD fix - quality issues)
- Word count outside target range
- Few callouts
- Missing code blocks in technical lessons

### Info (for reference)
- Word count within range
- Successful validation checks

---

## Exit Codes

- **0**: PASS (no errors, warnings allowed)
- **1**: FAIL (errors found)

---

## Common Issues & Fixes

### Issue: Missing frontmatter field

**Fix**: Add the field to the YAML frontmatter block:

```yaml
---
id: ros2-fundamentals
title: "ROS2 Fundamentals"
time_estimate: "45 minutes"  # ← Add missing field
---
```

### Issue: Word count too low

**Fix**: Expand sections with more detail:
- Add more conceptual explanations
- Include more examples
- Add analogies or comparisons

### Issue: Heading level skip

**Fix**: Add intermediate heading or adjust levels:

```markdown
## What Is ROS2?

### Core Components  # ← Add H3 before jumping to H4

#### Nodes  # Now H4 is valid
```

### Issue: Few callouts

**Fix**: Add interactive learning elements:

```markdown
💬 **AI Colearning Prompt**: Ask Claude to explain the pub/sub model using a real-world analogy.

🎓 **Expert Insight**: ROS2 uses DDS middleware, eliminating the single-point-of-failure master node from ROS1.

🤝 **Practice Exercise**: Diagram a simple robot system with 3 nodes and 2 topics.
```

---

## Best Practices

1. **Validate early**: Run validation as soon as content is complete
2. **Fix errors first**: Address all errors before reviewing warnings
3. **Re-validate**: Always re-run after fixes to confirm
4. **Use JSON for automation**: Parse JSON output in CI/CD pipelines
5. **Document exceptions**: If overriding a warning, add a comment explaining why

---

## Related Documentation

- **Frontmatter Template**: `specs/001-ros2-nervous-system/frontmatter-template.yaml`
- **Lesson Template**: `specs/001-ros2-nervous-system/lesson-content-template.md`
- **Data Model**: `specs/001-ros2-nervous-system/data-model.md`

---

## Status

✅ **T012 Complete**: Markdown validation script implemented
📍 **Next**: T013 (setup local Docusaurus development environment)
