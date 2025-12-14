@echo off
REM ============================================================================
REM MDX Textbook Chapters - Complete Validation Pipeline (Windows)
REM ============================================================================
REM This script runs all validation steps for the MDX textbook chapters:
REM   1. Extract code examples from MDX files
REM   2. Validate code examples (Python execution, C++ syntax)
REM   3. Validate readability (Flesch-Kincaid grade level)
REM
REM Usage:
REM   run-all-validations.bat [docs_directory]
REM
REM Arguments:
REM   docs_directory  Directory containing MDX files (default: docs/)
REM ============================================================================

setlocal enabledelayedexpansion

REM Script directory
set "SCRIPT_DIR=%~dp0"

REM Default docs directory
if "%~1"=="" (
    set "DOCS_DIR=docs"
) else (
    set "DOCS_DIR=%~1"
)

REM Results tracking
set EXTRACTION_RESULT=0
set CODE_VALIDATION_RESULT=0
set READABILITY_RESULT=0

echo.
echo ============================================================
echo MDX Textbook Validation Pipeline
echo ============================================================
echo Docs Directory: %DOCS_DIR%
echo Script Directory: %SCRIPT_DIR%
echo ============================================================
echo.

REM Check that docs directory exists
if not exist "%DOCS_DIR%" (
    echo Error: Docs directory '%DOCS_DIR%' does not exist
    exit /b 1
)

REM Step 1: Extract code examples
echo.
echo ============================================================
echo Step 1/3: Extracting Code Examples
echo ============================================================

python "%SCRIPT_DIR%extract-code-examples.py" "%DOCS_DIR%"
if !errorlevel! neq 0 (
    set EXTRACTION_RESULT=!errorlevel!
    echo Code extraction failed with exit code !EXTRACTION_RESULT!
) else (
    echo Code extraction completed successfully
)

REM Step 2: Validate code examples
echo.
echo ============================================================
echo Step 2/3: Validating Code Examples
echo ============================================================

python "%SCRIPT_DIR%validate-code-examples.py"
if !errorlevel! neq 0 (
    set CODE_VALIDATION_RESULT=!errorlevel!
    echo Code validation failed with exit code !CODE_VALIDATION_RESULT!
) else (
    echo Code validation completed successfully
)

REM Step 3: Validate readability
echo.
echo ============================================================
echo Step 3/3: Validating Readability
echo ============================================================

python "%SCRIPT_DIR%validate-readability.py" "%DOCS_DIR%"
if !errorlevel! neq 0 (
    set READABILITY_RESULT=!errorlevel!
    echo Readability validation failed with exit code !READABILITY_RESULT!
) else (
    echo Readability validation completed successfully
)

REM Final Summary
echo.
echo ============================================================
echo Validation Pipeline Summary
echo ============================================================

set FINAL_EXIT_CODE=0

if !EXTRACTION_RESULT! equ 0 (
    echo   Code Extraction:    PASSED
) else (
    echo   Code Extraction:    FAILED
    set FINAL_EXIT_CODE=1
)

if !CODE_VALIDATION_RESULT! equ 0 (
    echo   Code Validation:    PASSED
) else (
    echo   Code Validation:    FAILED
    if !FINAL_EXIT_CODE! equ 0 set FINAL_EXIT_CODE=2
)

if !READABILITY_RESULT! equ 0 (
    echo   Readability:        PASSED
) else (
    echo   Readability:        FAILED
    if !FINAL_EXIT_CODE! equ 0 set FINAL_EXIT_CODE=3
)

echo ============================================================

if !FINAL_EXIT_CODE! equ 0 (
    echo All validations passed!
) else (
    echo Some validations failed. Check output above for details.
)

echo.
echo Results saved to:
echo   - tests\code-validation-results.json
echo   - tests\readability-results.json
echo.

exit /b !FINAL_EXIT_CODE!
