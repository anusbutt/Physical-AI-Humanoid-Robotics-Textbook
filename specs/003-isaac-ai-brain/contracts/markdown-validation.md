# Markdown Validation for Module 3 - Isaac Platform Content

**Purpose**: Ensure consistent formatting, structure, and quality for Isaac platform lessons
**Based on**: Lesson structure template and frontmatter schema
**Target**: All Module 3 markdown files before publication
**Usage**: Validate each lesson file against these standards

---

## Validation Criteria

### File Structure
- [ ] Frontmatter YAML block exists and is properly formatted (lines 1-30 approximately)
- [ ] Title follows Isaac platform focus with clear student-facing language
- [ ] Sidebar position is unique within module (1-6)
- [ ] All 13 frontmatter fields present and properly filled
- [ ] Main content begins after closing `---` of frontmatter
- [ ] File ends with proper closing if needed

### Heading Hierarchy
- [ ] H1 (single `#`) used only for main lesson title
- [ ] H2 (double `##`) used for main sections: What Is, Why Matters, Practical Example, Summary, Next Steps
- [ ] H3 (triple `###`) used for subsections: Key Principles, callouts (AI Colearning, Expert Insight, Practice Exercise)
- [ ] Heading structure follows logical progression without gaps (H1 → H2 → H3 only)
- [ ] All required sections present: What Is, Why Matters, Key Principles, Practical Example, Summary, Next Steps

### Content Requirements
- [ ] Word count: 1500-2500 words for main lesson
- [ ] Summary section: 100-200 words with 3-5 key takeaways
- [ ] 2-3 callouts distributed throughout (💬 AI Colearning Prompt, 🎓 Expert Insight, 🤝 Practice Exercise)
- [ ] Isaac-specific terminology used appropriately
- [ ] Content builds on Modules 1-2 knowledge without requiring hardware access

### Isaac Platform Specific Validation
- [ ] References to Isaac Sim, Isaac ROS, or Nav2 are accurate and current
- [ ] Hardware requirements acknowledged without discouraging conceptual learning
- [ ] Integration between Isaac components described accurately
- [ ] Performance comparisons are realistic and properly contextualized
- [ ] Real-world examples specific to Isaac platform

### Formatting Standards
- [ ] Code blocks properly formatted with ``` and language specified
- [ ] Callouts consistently formatted with emojis and proper structure
- [ ] Links to Isaac documentation or resources are valid
- [ ] Images referenced with proper paths (static/img/isaac-ai-brain/)
- [ ] Internal cross-references to other lessons follow proper format

### Quality Gates
- [ ] Technical accuracy verified through Technical Reviewer Agent
- [ ] Content flows logically from section to section
- [ ] Learning objectives from frontmatter addressed in content
- [ ] Practice exercises are actionable and Isaac-focused
- [ ] Next Steps connect to subsequent lesson appropriately

---

## Validation Script Concept

This would be implemented as a Node.js script in the book-source directory:

```javascript
// validate-module3-lessons.js

const fs = require('fs');
const path = require('path');
const matter = require('gray-matter');

function validateLesson(filePath) {
  const content = fs.readFileSync(filePath, 'utf8');
  const parsed = matter(content);

  // Validate frontmatter
  const frontmatter = parsed.data;
  const requiredFields = ['title', 'sidebar_position', 'skills', 'learning_objectives',
                         'cognitive_load', 'differentiation', 'tags', 'generated_by',
                         'created', 'last_modified', 'ros2_version'];

  // Validate content structure
  const body = parsed.content;
  const headings = body.match(/^##\s.*/gm) || [];
  const requiredHeadings = ['What Is', 'Why Matters', 'Practical Example', 'Summary', 'Next Steps'];

  // Additional validation logic...

  return {
    isValid: true/false,
    errors: [],
    warnings: [],
    stats: { wordCount, headingCount, etc. }
  };
}

// Usage: node validate-module3-lessons.js book-source/docs/13-Physical-AI-Humanoid-Robotics/03-isaac-ai-brain/
```

---

## Common Issues to Check

### Frontmatter Issues
- Missing required fields
- Incorrect data types (e.g., string instead of array)
- Placeholder text not replaced
- Date format incorrect (should be YYYY-MM-DD)
- Sidebar position conflicts

### Content Issues
- Heading hierarchy violations
- Missing required sections
- Incorrect word count (too short or too long)
- Callouts missing or incorrectly formatted
- Isaac terminology inconsistent

### Isaac Platform Issues
- Outdated Isaac platform information
- Hardware requirements not properly addressed
- Integration scenarios technically inaccurate
- Performance claims unrealistic
- Missing acknowledgment of conceptual vs practical learning

---

## Acceptance Criteria

A Module 3 lesson passes validation when:
1. All structural requirements are met (✓)
2. Isaac platform content is accurate and current (✓)
3. Content is appropriate for target audience (✓)
4. Formatting follows established standards (✓)
5. Integration with Isaac ecosystem accurately described (✓)
6. All validation checks return no critical or major errors (✓)

---

## Notes
- Validation should occur before Technical Reviewer Agent
- Focus on Isaac platform specificity in all validation criteria
- Ensure lessons remain accessible to students without hardware
- Maintain consistency with Modules 1-2 structural patterns
- Verify proper referencing of Isaac platform documentation