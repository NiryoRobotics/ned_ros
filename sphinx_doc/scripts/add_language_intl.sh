#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Please supply argument(s) according to language(s) you want to add"
  exit 1
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

lang="$1"

# Set Build folder
if [ $# -eq 1 ]; then # If no build folder given, use default
  build_dir="_build"
else
  build_dir=$2
fi
cd $SCRIPT_DIR/..;
make gettext BUILDDIR=$build_dir

# Loop in arguments to find which arguments provided
while test $# -gt 0; do
  sphinx-intl update -p $build_dir/gettext -l $lang
  echo "Update language $lang"
  shift
done
cd -;