#!/bin/bash
# ============================================================================
# MDX Textbook Chapters - Complete Validation Pipeline
# ============================================================================
# This script runs all validation steps for the MDX textbook chapters:
#   1. Extract code examples from MDX files
#   2. Validate code examples (Python execution, C++ syntax)
#   3. Validate readability (Flesch-Kincaid grade level)
#
# Usage:
#   ./run-all-validations.sh [docs_directory]
#
# Arguments:
#   docs_directory  Directory containing MDX files (default: docs/)
#
# Exit Codes:
#   0 - All validations passed
#   1 - Extraction failed
#   2 - Code validation failed
#   3 - Readability validation failed
# ============================================================================

set -e  # Exit on first error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory (where this script and Python scripts are located)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default docs directory
DOCS_DIR="${1:-docs}"

# Results tracking
EXTRACTION_RESULT=0
CODE_VALIDATION_RESULT=0
READABILITY_RESULT=0

# Print header
echo ""
echo "============================================================"
echo -e "${BLUE}MDX Textbook Validation Pipeline${NC}"
echo "============================================================"
echo "Docs Directory: $DOCS_DIR"
echo "Script Directory: $SCRIPT_DIR"
echo "============================================================"
echo ""

# Check that docs directory exists
if [ ! -d "$DOCS_DIR" ]; then
    echo -e "${RED}Error: Docs directory '$DOCS_DIR' does not exist${NC}"
    exit 1
fi

# Step 1: Extract code examples
echo ""
echo "============================================================"
echo -e "${YELLOW}Step 1/3: Extracting Code Examples${NC}"
echo "============================================================"

if python "$SCRIPT_DIR/extract-code-examples.py" "$DOCS_DIR"; then
    echo -e "${GREEN}Code extraction completed successfully${NC}"
else
    EXTRACTION_RESULT=$?
    echo -e "${RED}Code extraction failed with exit code $EXTRACTION_RESULT${NC}"
fi

# Step 2: Validate code examples
echo ""
echo "============================================================"
echo -e "${YELLOW}Step 2/3: Validating Code Examples${NC}"
echo "============================================================"

if python "$SCRIPT_DIR/validate-code-examples.py"; then
    echo -e "${GREEN}Code validation completed successfully${NC}"
else
    CODE_VALIDATION_RESULT=$?
    echo -e "${RED}Code validation failed with exit code $CODE_VALIDATION_RESULT${NC}"
fi

# Step 3: Validate readability
echo ""
echo "============================================================"
echo -e "${YELLOW}Step 3/3: Validating Readability${NC}"
echo "============================================================"

if python "$SCRIPT_DIR/validate-readability.py" "$DOCS_DIR"; then
    echo -e "${GREEN}Readability validation completed successfully${NC}"
else
    READABILITY_RESULT=$?
    echo -e "${RED}Readability validation failed with exit code $READABILITY_RESULT${NC}"
fi

# Final Summary
echo ""
echo "============================================================"
echo -e "${BLUE}Validation Pipeline Summary${NC}"
echo "============================================================"

FINAL_EXIT_CODE=0

if [ $EXTRACTION_RESULT -eq 0 ]; then
    echo -e "  Code Extraction:    ${GREEN}PASSED${NC}"
else
    echo -e "  Code Extraction:    ${RED}FAILED${NC}"
    FINAL_EXIT_CODE=1
fi

if [ $CODE_VALIDATION_RESULT -eq 0 ]; then
    echo -e "  Code Validation:    ${GREEN}PASSED${NC}"
else
    echo -e "  Code Validation:    ${RED}FAILED${NC}"
    if [ $FINAL_EXIT_CODE -eq 0 ]; then
        FINAL_EXIT_CODE=2
    fi
fi

if [ $READABILITY_RESULT -eq 0 ]; then
    echo -e "  Readability:        ${GREEN}PASSED${NC}"
else
    echo -e "  Readability:        ${RED}FAILED${NC}"
    if [ $FINAL_EXIT_CODE -eq 0 ]; then
        FINAL_EXIT_CODE=3
    fi
fi

echo "============================================================"

if [ $FINAL_EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}All validations passed!${NC}"
else
    echo -e "${RED}Some validations failed. Check output above for details.${NC}"
fi

echo ""
echo "Results saved to:"
echo "  - tests/code-validation-results.json"
echo "  - tests/readability-results.json"
echo ""

exit $FINAL_EXIT_CODE
