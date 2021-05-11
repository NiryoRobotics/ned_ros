#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Please supply argument(s) according to language(s) you want to add"
  exit 1
fi

make gettext

# Loop in arguments to find which arguments provided
while test $# -gt 0; do
  sphinx-intl update -p _build/gettext -l "$1"
  echo "Update language $1"
  shift
done
