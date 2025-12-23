#!/usr/bin/env bash
# validate-markdown.sh
# Purpose: Validate markdown lesson files for heading hierarchy, frontmatter schema, and word count
# Usage: ./validate-markdown.sh <file-path> [--json]

set -euo pipefail

# Configuration
MIN_WORD_COUNT=1500
MAX_WORD_COUNT=2500
REQUIRED_FRONTMATTER_FIELDS=(
    "id"
    "title"
    "sidebar_position"
    "description"
    "tags"
    "skills"
    "learning_objectives"
    "time_estimate"
    "difficulty_level"
    "cognitive_load"
    "prerequisites"
    "related_lessons"
    "assessment_method"
)

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Parse arguments
FILE_PATH="${1:-}"
JSON_OUTPUT=false

if [[ "${2:-}" == "--json" ]]; then
    JSON_OUTPUT=true
fi

# Validate file exists
if [[ -z "$FILE_PATH" ]]; then
    echo "Usage: $0 <file-path> [--json]"
    exit 1
fi

if [[ ! -f "$FILE_PATH" ]]; then
    echo "Error: File not found: $FILE_PATH"
    exit 1
fi

# Initialize validation results
ERRORS=()
WARNINGS=()
INFO=()

# 1. Extract frontmatter
if grep -q "^---$" "$FILE_PATH"; then
    FRONTMATTER=$(sed -n '/^---$/,/^---$/p' "$FILE_PATH" | sed '1d;$d')

    # Check required frontmatter fields
    for field in "${REQUIRED_FRONTMATTER_FIELDS[@]}"; do
        if ! echo "$FRONTMATTER" | grep -q "^${field}:"; then
            ERRORS+=("Missing required frontmatter field: $field")
        fi
    done
else
    ERRORS+=("No frontmatter block found (must start and end with '---')")
fi

# 2. Validate heading hierarchy
HEADINGS=$(grep -E "^#{1,6} " "$FILE_PATH" || true)

# Check for H1 (only one allowed)
H1_COUNT=$(echo "$HEADINGS" | grep -c "^# " || true)
if [[ $H1_COUNT -eq 0 ]]; then
    WARNINGS+=("No H1 heading found")
elif [[ $H1_COUNT -gt 1 ]]; then
    ERRORS+=("Multiple H1 headings found ($H1_COUNT). Only one H1 allowed per lesson.")
fi

# Check heading sequence (no skipping levels)
PREV_LEVEL=0
LINE_NUM=0
while IFS= read -r line; do
    LINE_NUM=$((LINE_NUM + 1))
    if [[ $line =~ ^(#{1,6})\ .+ ]]; then
        CURRENT_LEVEL=${#BASH_REMATCH[1]}

        if [[ $PREV_LEVEL -gt 0 ]] && [[ $CURRENT_LEVEL -gt $((PREV_LEVEL + 1)) ]]; then
            ERRORS+=("Heading level skip at line $LINE_NUM: jumped from H$PREV_LEVEL to H$CURRENT_LEVEL")
        fi

        PREV_LEVEL=$CURRENT_LEVEL
    fi
done < "$FILE_PATH"

# 3. Word count validation (exclude frontmatter and code blocks)
# Remove frontmatter
CONTENT=$(sed '/^---$/,/^---$/d' "$FILE_PATH")

# Remove code blocks
CONTENT=$(echo "$CONTENT" | sed '/```/,/```/d')

# Count words
WORD_COUNT=$(echo "$CONTENT" | wc -w | tr -d ' ')

if [[ $WORD_COUNT -lt $MIN_WORD_COUNT ]]; then
    WARNINGS+=("Word count too low: $WORD_COUNT words (minimum: $MIN_WORD_COUNT)")
elif [[ $WORD_COUNT -gt $MAX_WORD_COUNT ]]; then
    WARNINGS+=("Word count too high: $WORD_COUNT words (maximum: $MAX_WORD_COUNT)")
else
    INFO+=("Word count: $WORD_COUNT words (target: $MIN_WORD_COUNT-$MAX_WORD_COUNT)")
fi

# 4. Check for common patterns
# Check for callouts
CALLOUT_COUNT=$(grep -c "💬\|🎓\|🤝" "$FILE_PATH" || true)
if [[ $CALLOUT_COUNT -lt 2 ]]; then
    WARNINGS+=("Few callouts found ($CALLOUT_COUNT). Expected at least 2-3 callouts (💬 AI Colearning, 🎓 Expert Insight, 🤝 Practice Exercise)")
fi

# Check for code blocks (if rclpy or URDF lesson)
if grep -qi "rclpy\|python\|urdf" "$FILE_PATH"; then
    CODE_BLOCK_COUNT=$(grep -c '```' "$FILE_PATH" || true)
    CODE_BLOCK_COUNT=$((CODE_BLOCK_COUNT / 2)) # Each block has opening and closing

    if [[ $CODE_BLOCK_COUNT -eq 0 ]]; then
        WARNINGS+=("No code blocks found in lesson that mentions rclpy/Python/URDF")
    fi
fi

# Generate output
if [[ "$JSON_OUTPUT" == true ]]; then
    # JSON output
    cat <<EOF
{
  "file": "$FILE_PATH",
  "word_count": $WORD_COUNT,
  "h1_count": $H1_COUNT,
  "callout_count": $CALLOUT_COUNT,
  "errors": [
$(printf '    "%s"' "${ERRORS[@]}" | paste -sd ',' -)
  ],
  "warnings": [
$(printf '    "%s"' "${WARNINGS[@]}" | paste -sd ',' -)
  ],
  "info": [
$(printf '    "%s"' "${INFO[@]}" | paste -sd ',' -)
  ],
  "status": "$(if [[ ${#ERRORS[@]} -eq 0 ]]; then echo "PASS"; else echo "FAIL"; fi)"
}
EOF
else
    # Human-readable output
    echo "===================================="
    echo "Markdown Validation Report"
    echo "===================================="
    echo "File: $FILE_PATH"
    echo ""

    # Info
    if [[ ${#INFO[@]} -gt 0 ]]; then
        echo -e "${GREEN}INFO:${NC}"
        for item in "${INFO[@]}"; do
            echo "  ✓ $item"
        done
        echo ""
    fi

    # Warnings
    if [[ ${#WARNINGS[@]} -gt 0 ]]; then
        echo -e "${YELLOW}WARNINGS:${NC}"
        for item in "${WARNINGS[@]}"; do
            echo "  ⚠ $item"
        done
        echo ""
    fi

    # Errors
    if [[ ${#ERRORS[@]} -gt 0 ]]; then
        echo -e "${RED}ERRORS:${NC}"
        for item in "${ERRORS[@]}"; do
            echo "  ✗ $item"
        done
        echo ""
    fi

    # Final status
    if [[ ${#ERRORS[@]} -eq 0 ]]; then
        echo -e "${GREEN}Status: PASS ✓${NC}"
        exit 0
    else
        echo -e "${RED}Status: FAIL ✗${NC}"
        exit 1
    fi
fi
