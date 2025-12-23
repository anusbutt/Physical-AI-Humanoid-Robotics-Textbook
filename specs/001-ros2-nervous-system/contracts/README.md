# Agent Contracts - Module 1

This directory contains contracts for the 8 agents in the content creation pipeline.

**Fully Specified**:
1. [outline-agent.md](outline-agent.md) - Generates 7-section lesson outlines
2. [technical-reviewer-agent.md](technical-reviewer-agent.md) - Validates ROS2/rclpy/URDF accuracy

**Placeholders** (to be detailed in future iterations):
3. content-agent.md - Fills outline with full prose content
4. case-study-agent.md - Generates real-world robotics examples
5. code-example-agent.md - Creates Python rclpy snippets
6. structure-style-agent.md - Validates heading hierarchy and formatting
7. frontmatter-agent.md - Auto-generates YAML metadata
8. docusaurus-agent.md - Tests MDX rendering and navigation

**Implementation Strategy**: Build agents iteratively as patterns stabilize (Constitution Principle VII - YAGNI).

**Priority**: Technical Reviewer Agent is critical for all lessons. Other agents optional for Module 1.
