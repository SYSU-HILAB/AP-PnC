#!/bin/bash
# test_bin - Run test executables from the build directory
# Usage: ./test_bin.sh <test_name> [args...]

TEST_NAME="$1"
shift

# Find the test executable in build directory
TEST_PATH=$(find ./build -name "$TEST_NAME" -type f -executable 2>/dev/null | head -1)

if [ -z "$TEST_PATH" ]; then
    echo "Error: Test executable '$TEST_NAME' not found in ./build"
    echo ""
    echo "Available tests:"
    find ./build -name "test_*" -type f -executable 2>/dev/null | sed 's|./build/||' | sed 's|/||g' | sort
    exit 1
fi

echo "Running: $TEST_PATH"
echo "======================================"
"$TEST_PATH" "$@"
RESULT=$?
echo "======================================"
echo "Exit code: $RESULT"

exit $RESULT
