#!/bin/bash

# Go to the source directory.
[ -n "$1" ] && cd "$1" || exit 1

# Build a version string with git.
version=$(git describe --tags --always --dirty 2> /dev/null)

# If this is not a git repository, fallback to the compilation date.
[ -n "$version" ] || version=$(date -I)

# Save this in git-version.h.
echo "#define GIT_VERSION \"$version\"" > $2/sketch/git-version.h
