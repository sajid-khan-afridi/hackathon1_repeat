#!/bin/bash

# Validation script to check spec numbering consistency
# Ensures that spec folders follow the phase-based numbering from the constitution

set -e

# Phase to number mapping based on constitution
declare -A PHASE_NUMBERS=(
    ["Book Infrastructure"]="001"
    ["Content Creation"]="002"
    ["RAG Chatbot Core"]="003"
    ["Authentication"]="004"
    ["Personalization"]="005"
    ["Translation"]="006"
    ["Integration & Deployment"]="007"
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
                # Extract phase from spec content (look for "Phase X" or "Phase X of Y")
                phase_line=$(grep -i "phase [0-9]" "$spec_file" | head -1 || echo "")
                # Extract just "Phase X" from patterns like "Phase 1" or "Phase 1 of 6"
                phase=$(echo "$phase_line" | sed -E 's/.*(phase [0-9]+).*/\1/i' | tr '[:lower:]' '[:upper:]' || echo "")

                if [ ! -z "$phase" ]; then
                    phase_num=$(echo $phase | grep -o '[0-9]')
                    padded_phase_num=$(printf "%03d" $phase_num)

                    # Check if spec number matches phase number
                    if [ "$spec_number" != "$padded_phase_num" ]; then
                        echo -e "${RED}❌ MISMATCH: $dir_name"
                        echo "   Spec mentions $phase but uses prefix $spec_number"
                        echo "   Expected: ${padded_phase_num}-${spec_name}${NC}"
                        errors=$((errors + 1))
                    else
                        echo -e "${GREEN}✅ $dir_name (Phase $phase_num)${NC}"
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