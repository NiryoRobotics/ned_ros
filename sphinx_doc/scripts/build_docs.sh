#!/bin/bash

# Folder in which paragraphs are translated
lang_path="locale"
build_output="_build"
build_type="html"

additional_parameters=()
while [[ $# -gt 0 ]]; do
  key="$1"
  case $key in
    -o|--output)
      build_output="$2"
      shift # past argument
      shift # past value
      ;;
    -b|--build-type)
      build_type="$2"
      shift # past argument
      shift # past value
      ;;
    *)    # unknown option
      additional_parameters+=("$1") # save it in an array for later
      shift # past argument
      ;;
  esac
done

# Change current directory
script_path=$(realpath "$0") # Get script path
documentation_path=$(dirname "$(dirname "$script_path")")
cd "$documentation_path" || exit

# -- Iterate over all languages and build their respective documentation
for dir in "$lang_path"/*/; do
  # Obtain language
  dir=${dir%*/}         # remove the trailing "/"
  language="${dir##*/}" # remove everything before the last "/"

  sphinx-build -D language="$language" -b ${build_type} . "${build_output}"/"$language" "${additional_parameters[@]}"
done
