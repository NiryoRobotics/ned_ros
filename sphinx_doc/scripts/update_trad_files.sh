#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

build_output="_build"

additional_parameters=()
while [[ $# -gt 0 ]]; do
  key="$1"
  case $key in
    -o|--output)
      build_output="$2"
      shift # past argument
      shift # past value
      ;;
    *)    # unknown option
      additional_parameters+=("$1") # save it in an array for later
      shift # past argument
      ;;
  esac
done

cd $SCRIPT_DIR/..;
make gettext BUILDDIR=$build_output

sphinx-intl update -p $build_output/gettext;
cd -;