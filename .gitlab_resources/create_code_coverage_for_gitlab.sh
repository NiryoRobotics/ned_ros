#! /bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
OUTPUT_FILENAME="cobertura.xml"
TMP_MERGED_INFO_FILENAME="merged_file.info"

function help()
{
    echo -e \
    "Usage: create_code_coverage_for_gitlab.sh -p|--path BUILD_PATH [-o|--output_file]"
}

POSITIONAL=()
while [[ $# -gt 0 ]]; do
  key="$1"

  case $key in
    -p|--path)
      BUILD_PATH="$2"
      shift # past argument
      shift # past value
      ;;
    -o|--output_file)
      OUTPUT_FILENAME="$2"
      shift # past argument
      shift # past value
      ;;
    *)    # unknown option
      POSITIONAL+=("$1") # save it in an array for later
      shift # past argument
      ;;
  esac
done

if [ -z ${BUILD_PATH+x} ]; then
    echo "You need to provide the following argument: -p / --path BUILD_PATH"
    exit 1
fi

lcov_merge_file_arguments_string=""
for file_path in $(find $BUILD_PATH -name '*.info'); do
    lcov --summary "$file_path" 2&> /dev/null
    if [ $? -eq 0 ]; then
        lcov_merge_file_arguments_string="$lcov_merge_file_arguments_string -a $file_path"
    fi
done

echo $lcov_merge_file_arguments_string
lcov $lcov_merge_file_arguments_string -o "$TMP_MERGED_INFO_FILENAME"

python $SCRIPT_DIR/lcov_to_cobertura.py $TMP_MERGED_INFO_FILENAME --output $OUTPUT_FILENAME
rm $TMP_MERGED_INFO_FILENAME
echo "Cobertura XML code coverage file generated at: $OUTPUT_FILENAME"

FILESIZE=$(stat -c%s "$OUTPUT_FILENAME")
MAX_SIZE=10000000
if [ $FILESIZE -gt $MAX_SIZE ]; then
    echo "WARNING: Generated code coverage file is bigger than the maximum authorized size ($MAX_SIZE bytes), it MAY fails the gitlab pipeline."
fi