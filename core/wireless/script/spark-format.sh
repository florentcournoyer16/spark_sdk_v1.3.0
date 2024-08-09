#!/bin/bash

# Default values
source_branch=""
fix_formatting=false
help=false

script_dir=$(dirname "$0")
git_clang_format="$script_dir/git-clang-format"

# Parse arguments
options=$(getopt -o hf --long help,fix -n "$0" -- "$@")
eval set -- "$options"

while true; do
    case "$1" in
        -h|--help)
            help=true
            shift
            ;;
        -f|--fix)
            fix_formatting=true
            shift
            ;;
        --)
            shift
            break
            ;;
        *)
            echo "Invalid option: $1" >&2
            echo "Usage: $0 <source branch> [-h | --help] [-f | --fix]" >&2
            exit 1
            ;;
    esac
done

# Check for missing source branch argument
if [ -z "$1" ]; then
    echo "Missing source branch argument" >&2
    echo "Usage: $0 <source branch> [-h | --help] [-f | --fix]" >&2
    exit 1
fi

source_branch_name="$1"
source_branch=$(git merge-base --fork-point  $source_branch_name)

# Show help message if requested
if $help; then
    echo "Usage: $0 <source branch> [-h | --help] [-f | --fix]"
    echo ""
    echo "Run clang-format on all lines that differ between the current commit and the <source branch>"
    echo ""
    echo "Arguments:"
    echo "  <source branch>: The name of the source branch for the merge request"
    echo ""
    echo "Options:"
    echo "  -h, --help: Display this help message"
    echo "  -f, --fix:  Fix formatting errors"
    exit 0
fi

# Check if git and clang-format are installed
if [ ! -x "$(command -v git)" ] || [ ! -x "$(command -v clang-format)" ]; then
    echo "Error : git and clang-format must be installed to run this script" >&2
    exit 1
fi

num_files=$($git_clang_format $source_branch --diff --extensions h,c,cpp | cat | grep -c "diff")

if [ $num_files -eq 0 ]; then
    echo "Congratulations! No formatting changes needed"
    exit 0
fi

echo ""
if [[ $num_files -eq 0 || $num_files -eq 1 ]]; then
  echo "Found $num_files file with formatting errors. Format changes:"
else
  echo "Found $num_files files with formatting errors. Format changes:"
fi

echo ""
$git_clang_format $source_branch --diff --extensions h,c,cpp | cat | sed 's/^-/\x1b[1;31m-/;s/^+/\x1b[1;32m+/;s/^@/\x1b[1;36m@/;s/$/\x1b[0m/'
echo ""

if [ "$fix_formatting" = false ]; then
    echo "To fix formatting, run:"
    echo ""
    echo "  spark-format.sh $source_branch_name --fix"
    exit 1
fi

echo "Fixing formatting errors..."
$git_clang_format $source_branch --extensions h,c,cpp

if [ $? -ne 1 ]; then
    echo ""
    echo "Failed to format files"
    exit 1
fi

echo ""
if [[ $num_files -eq 0 || $num_files -eq 1 ]]; then
  echo "Successfully formatted $num_files file"
else
  echo "Successfully formatted $num_files files"
fi

exit 0

