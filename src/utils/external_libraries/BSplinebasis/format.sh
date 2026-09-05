#!/bin/bash

FILES=`git diff --cached --name-only | grep -i -E "\.(h|cpp)$"`
for FILE in $FILES
do
    echo "Formatting altered file \"$FILE\""
    clang-format -i --style=google "$FILE"
done
