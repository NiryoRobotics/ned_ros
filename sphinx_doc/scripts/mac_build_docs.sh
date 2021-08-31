#!/bin/bash

# Folder in which paragraphs are translated
lang_path="locale"

# Set Build folder
if [ $# -eq 0 ]; then # If no build folder given, use default
  build_path="_build"
else
  build_path=$1
fi

# -- Iterate over all languages and build their respective documentation
for dir in "$lang_path"/*/; do
  # Obtain language
  dir=${dir%*/}         # remove the trailing "/"
  language="${dir##*/}" # remove everything before the last "/"

  # Build documentation
  python3 -m sphinx -D language="$language" -b html . "${build_path}"/"$language"
done
