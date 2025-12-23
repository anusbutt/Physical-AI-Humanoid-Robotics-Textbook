# Technical Reviewer Agent - Usage Guide

**Status**: ✅ Implemented (T011)
**Template**: `.specify/templates/technical-reviewer-prompt.md`
**Contract**: `specs/001-ros2-nervous-system/contracts/technical-reviewer-agent.md`

---

## Quick Start

### Step 1: Read the Lesson Content

```bash
# In Claude Code, read the lesson you want to validate
Read: book-source/docs/13-Physical-AI-Humanoid-Robotics/01-ros2-nervous-system/01-ros2-fundamentals.md
```

### Step 2: Prepare Validation Parameters

Determine which focus areas to validate:
- `ros2_concepts` - For Lessons 1, 2
- `rclpy_api` - For Lesson 3
- `urdf_syntax` - For Lesson 4
- `pedagogy` - For ALL lessons

### Step 3: Invoke Task Tool with Validation Prompt

```
Use the Task tool with subagent_type='general-purpose' and this prompt:

---
You are a Technical Reviewer Agent specializing in ROS2, rclpy, and URDF content validation.

Validate the technical accuracy of this lesson content against official ROS2 documentation and pedagogical best practices.

INPUT PARAMETERS:
```yaml
lesson_content: |
  [PASTE FULL LESSON CONTENT HERE - including frontmatter]

focus_areas: ["ros2_concepts", "pedagogy"]
ros2_version: "humble"
student_level: "beginner"
```

VALIDATION CHECKLIST:
[Copy from .specify/templates/technical-reviewer-prompt.md]

OUTPUT FORMAT:
Return JSON matching this schema:
{
  "validation_result": "PASS | FAIL | NEEDS_REVISION",
  "issues": [...],
  "validation_checks": {...},
  "summary": "..."
}

RULES:
1. Be specific: include exact location (section, line)
2. Be actionable: critical/major issues need concrete suggestions
3. Be accurate: validate against official ROS2 Humble docs
4. Be fair: consider beginner student level
5. Be thorough: check all requested focus_areas

AUTHORITATIVE REFERENCES:
- ROS2 Concepts: https://docs.ros.org/en/humble/Concepts.html
- rclpy API: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html
- URDF Spec: http://wiki.ros.org/urdf/XML

Return ONLY the JSON object.
---
```

### Step 4: Review Validation Results

The agent will return JSON output:

```json
{
  "validation_result": "PASS",
  "issues": [],
  "validation_checks": {
    "ros2_concepts_accurate": true,
    "explanations_appropriate": true,
    "terminology_consistent": true
  },
  "summary": "All validation checks passed. Content is technically accurate."
}
```

Or if issues found:

```json
{
  "validation_result": "NEEDS_REVISION",
  "issues": [
    {
      "severity": "critical",
      "category": "ros2_accuracy",
      "location": "Section: What Is ROS2?, Paragraph 2",
      "issue": "...",
      "suggestion": "...",
      "reference": "https://docs.ros.org/..."
    }
  ],
  "validation_checks": {...},
  "summary": "1 critical issue, 2 major issues found."
}
```

### Step 5: Fix Issues

- **Critical/Major issues**: MUST fix before proceeding
- **Minor issues**: Author decides whether to fix

### Step 6: Re-validate (if needed)

After fixing issues, repeat Steps 1-4 until validation_result is "PASS".

---

## Focus Areas by Lesson

| Lesson | Focus Areas |
|--------|-------------|
| Lesson 1: ROS2 Fundamentals | `["ros2_concepts", "pedagogy"]` |
| Lesson 2: Nodes/Topics/Services | `["ros2_concepts", "pedagogy"]` |
| Lesson 3: Python rclpy Bridge | `["ros2_concepts", "rclpy_api", "pedagogy"]` |
| Lesson 4: URDF for Humanoids | `["urdf_syntax", "pedagogy"]` |

---

## Severity Guidelines

### Critical (BLOCKS progression)
- Factually incorrect technical information
- API usage that won't work in ROS2 Humble
- Misleading explanations that teach wrong concepts

**Examples**:
- "ROS2 has a master node" (false - ROS2 is master-less)
- `Node.__init__(self, 'name')` (should use `super().__init__('name')`)
- "Topics are synchronous" (false - topics are asynchronous pub/sub)

### Major (BLOCKS progression)
- Incorrect code syntax that won't compile/run
- Misleading analogies that confuse core concepts
- Missing critical safety/best practice information

**Examples**:
- Missing `rclpy.init()` in example code
- Analogy that implies services are asynchronous
- Missing type hints in Python 3.11+ code

### Minor (logged, doesn't block)
- Terminology inconsistency (node vs Node)
- Unclear wording that could be clearer
- Missing helpful but non-essential details

**Examples**:
- "ROS2 nodes" vs "nodes" (inconsistent capitalization)
- Could add QoS explanation but not required for beginners
- Missing import statement in partial code example

---

## Workflow Integration

### In Tasks (e.g., T026, T042, T060)

Tasks reference validation like this:
```
T026 [US1] Run Technical Reviewer Agent validation with focus_areas: ["ros2_concepts", "pedagogy"]
T027 [US1] Fix any critical/major issues identified by Technical Reviewer Agent
```

**How to execute**:
1. Complete lesson content (T014-T025 for Lesson 1)
2. Read lesson file
3. Invoke Task tool with validation prompt (T026)
4. Review JSON output
5. Fix critical/major issues (T027)
6. Re-validate if needed
7. Mark T026, T027 as complete ✅

### Example: Validating Lesson 1

```
# After completing T014-T025 (Lesson 1 content)

# T026: Run validation
Read: book-source/docs/.../01-ros2-fundamentals.md
Task tool (general-purpose): [validation prompt with focus_areas: ["ros2_concepts", "pedagogy"]]

# T027: Fix issues
Edit: book-source/docs/.../01-ros2-fundamentals.md
  (Fix critical issue: replace "master node" with "distributed DDS discovery")

# Re-run T026 validation
Task tool (general-purpose): [validation prompt again]
  Result: "PASS" ✅

# Mark complete
T026 ✅
T027 ✅
```

---

## Troubleshooting

### Issue: Agent returns vague feedback

**Solution**: Re-run with emphasis on "be specific - include exact location and line numbers"

### Issue: False positive (agent flags correct content)

**Solution**: Human review override - document the dispute and proceed. Consider updating the prompt template with clarification.

### Issue: Agent misses actual error (false negative)

**Solution**: Post-deployment correction. Update validation checklist in template for future lessons.

### Issue: Validation takes too long

**Solution**: Reduce focus_areas to only what's relevant for that lesson. Don't validate "rclpy_api" if the lesson doesn't have rclpy code.

---

## Best Practices

1. **Validate early**: Run validation as soon as lesson content is complete, not at the end of all 4 lessons
2. **Fix incrementally**: Address critical issues first, then major, then decide on minor
3. **Keep validation output**: Save JSON to a file for documentation/reference
4. **Re-validate after fixes**: Always re-run validation to confirm issues are resolved
5. **Document disputes**: If you override a false positive, add a note explaining why

---

## Template Location

Full prompt template with all checklists:
`.specify/templates/technical-reviewer-prompt.md`

Contract specification:
`specs/001-ros2-nervous-system/contracts/technical-reviewer-agent.md`

---

## Status

✅ **T011 Complete**: Technical Reviewer Agent validation workflow implemented
📍 **Next**: T012 (markdown validation script), T013 (local Docusaurus setup)
