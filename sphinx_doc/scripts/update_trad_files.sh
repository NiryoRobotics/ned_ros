#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Set Build folder
if [ $# -eq 0 ]; then # If no build folder given, use default
  build_dir="_build"
else
  build_dir=$1
fi

cd $SCRIPT_DIR/..;
make gettext BUILDDIR=$build_dir

sphinx-intl update -p $build_dir/gettext;
cd -;