#!/bin/bash

# Folder in which paragraphs are translated
lang_path="locale"

# Set Build folder
if [ $# -eq 0 ]; then # If no build folder given, use default
  build_path="_build"
else
  build_path=$1
fi

# Change current directory
script_path=$(realpath "$0") # Get script path
documentation_path=$(dirname "$(dirname "$script_path")")
cd "$documentation_path" || exit

# -- Iterate over all languages and build their respective documentation
for dir in "$lang_path"/*/; do
  # Obtain language
  dir=${dir%*/}         # remove the trailing "/"
  language="${dir##*/}" # remove everything before the last "/"

  # Build documentation
  python -m sphinx -D language="$language" -b html . "${build_path}"/"$language"
done

# - Display docs paths
echo ""
echo "--- Path to Docs ---"
echo ""

for dir in "$lang_path"/*/; do
  # Obtain language
  dir=${dir%*/}         # remove the trailing "/"
  language="${dir##*/}" # remove everything before the last "/"
  echo "** ${language} **"
  echo $(realpath "${build_path}/${language}/index.html")
done
