#! /bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

OUTPUT_FOLDER=generated_pdfs
SINGLE_HTML_BUILD_NAME=_singlehtml

additional_parameters=()
while [[ $# -gt 0 ]]; do
  key="$1"
  case $key in
    -o|--output)
      OUTPUT_FOLDER="$2"
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
$SCRIPT_DIR/build_docs.sh -b singlehtml -o $SINGLE_HTML_BUILD_NAME -D todo_include_todos=0

mkdir -p $OUTPUT_FOLDER
for directory_path in _singlehtml/*; do
    directory=$(basename $directory_path)
    echo ./$OUTPUT_FOLDER/pdf_$directory.pdf
    wkhtmltopdf ./$SINGLE_HTML_BUILD_NAME/$directory/index.html ./$OUTPUT_FOLDER/pdf_$directory.pdf
    done
cd -;
rm -r $SINGLE_HTML_BUILD_NAME