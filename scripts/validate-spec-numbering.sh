#!/bin/bash

# Validation script to check spec numbering consistency
# Ensures that spec folders follow the phase-based numbering from the constitution

set -e

# Phase to number mapping based on constitution
# Note: Phase 4 has sub-phases (4A=004, 4B=005), so Phase 5=006, Phase 6=007
declare -A PHASE_NUMBERS=(
    ["Book Infrastructure"]="001"
    ["Content Creation"]="002"
    ["RAG Chatbot Core"]="003"
    ["Authentication"]="004"
    ["Personalization"]="005"
    ["Translation"]="006"
    ["Integration & Deployment"]="007"
)

# Phase number to spec prefix mapping (handles sub-phases)
# Note: "4B" maps to 005, "4A" or "4" maps to 004
declare -A PHASE_TO_PREFIX=(
    ["1"]="001"
    ["2"]="002"
    ["3"]="003"
    ["4"]="004"   # Phase 4 or 4A (Authentication)
    ["4A"]="004"  # Phase 4A (Authentication)
    ["4B"]="005"  # Phase 4B (Personalization)
    ["5"]="006"   # Phase 5 (Translation) - offset due to Phase 4A/4B split
    ["6"]="007"   # Phase 6 (Integration)
)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

errors=0
warnings=0

echo "🔍 Validating spec numbering consistency..."
echo "=========================================="

# Check all spec directories
for spec_dir in specs/*/; do
    if [ -d "$spec_dir" ]; then
        dir_name=$(basename "$spec_dir")

        # Extract number from directory name
        if [[ $dir_name =~ ^([0-9]+)-(.+)$ ]]; then
            spec_number="${BASH_REMATCH[1]}"
            spec_name="${BASH_REMATCH[2]}"

            # Check if spec has spec.md file
            spec_file="${spec_dir}spec.md"
            if [ -f "$spec_file" ]; then
                # Extract phase from spec content (look for "Phase X", "Phase XA", "Phase XB")
                phase_line=$(grep -i "phase [0-9][AB]*" "$spec_file" | head -1 || echo "")
                # Extract "Phase X" or "Phase XA/XB" from patterns
                phase=$(echo "$phase_line" | sed -E 's/.*(phase [0-9]+[AB]?).*/\1/i' | tr '[:lower:]' '[:upper:]' || echo "")

                if [ ! -z "$phase" ]; then
                    # Extract phase identifier (e.g., "4B", "5", "1")
                    phase_id=$(echo "$phase" | grep -oE '[0-9]+[AB]?' | head -1)

                    # Use the phase-to-prefix mapping to handle sub-phases
                    expected_prefix="${PHASE_TO_PREFIX[$phase_id]}"
                    if [ -z "$expected_prefix" ]; then
                        # Try without letter suffix (e.g., "4B" -> "4")
                        phase_num_only=$(echo "$phase_id" | grep -o '[0-9]')
                        expected_prefix="${PHASE_TO_PREFIX[$phase_num_only]}"
                    fi
                    if [ -z "$expected_prefix" ]; then
                        # Fallback to simple padding if not in mapping
                        phase_num_only=$(echo "$phase_id" | grep -o '[0-9]')
                        expected_prefix=$(printf "%03d" $phase_num_only)
                    fi

                    # Check if spec number matches expected prefix
                    if [ "$spec_number" != "$expected_prefix" ]; then
                        echo -e "${RED}❌ MISMATCH: $dir_name"
                        echo "   Spec mentions $phase but uses prefix $spec_number"
                        echo "   Expected: ${expected_prefix}-${spec_name}${NC}"
                        errors=$((errors + 1))
                    else
                        echo -e "${GREEN}✅ $dir_name (Phase $phase_id)${NC}"
                    fi
                else
                    echo -e "${YELLOW}⚠️  $dir_name: No phase found in spec.md${NC}"
                    warnings=$((warnings + 1))
                fi
            else
                echo -e "${RED}❌ $dir_name: Missing spec.md file${NC}"
                errors=$((errors + 1))
            fi
        else
            echo -e "${YELLOW}⚠️  $dir_name: Does not follow NNN-name pattern${NC}"
            warnings=$((warnings + 1))
        fi
    fi
done

# Check for duplicate numbers
echo ""
echo "🔍 Checking for duplicate spec numbers..."
numbers=$(ls specs/ | grep -o '^[0-9]\+' | sort | uniq -d)
if [ ! -z "$numbers" ]; then
    echo -e "${RED}❌ Duplicate spec numbers found:${NC}"
    echo "$numbers" | while read num; do
        ls specs/ | grep "^${num}-" | sed 's/^/   /'
    done
    errors=$((errors + 1))
else
    echo -e "${GREEN}✅ No duplicate spec numbers${NC}"
fi

# Summary
echo ""
echo "=========================================="
if [ $errors -gt 0 ]; then
    echo -e "${RED}❌ Validation failed with $errors error(s)${NC}"
    exit 1
elif [ $warnings -gt 0 ]; then
    echo -e "${YELLOW}⚠️  Validation passed with $warnings warning(s)${NC}"
    exit 0
else
    echo -e "${GREEN}✅ All specs follow correct phase-based numbering!${NC}"
    exit 0
fi