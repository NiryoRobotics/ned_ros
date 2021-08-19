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

cd $SCRIPT_DIR/.. > /dev/null;
$SCRIPT_DIR/build_docs.sh -b singlehtml -o $SINGLE_HTML_BUILD_NAME -D todo_include_todos=0
header_html='header_html="""<!DOCTYPE html>
<meta charset="UTF-8">
<html>

<style>
    @import url("https://use.typekit.net/tsj7mzy.css");

    .niryoheader {{
        text-align: center;
        font-family: {};
        color: #{};
        font-size: {}px;
        font-weight: bold;
    }}
</style>
<header>
  <p class="niryoheader">{}</p>
</header>
</html>""".format(conf.shared_conf.pdf_header_font_name,
conf.shared_conf.pdf_header_font_color,
conf.shared_conf.pdf_header_font_size,
"" + conf.project + " (" + conf.release + ")")'

footer_html='footer_html="""<!DOCTYPE html>
<meta charset="UTF-8">
<html>

<style>
    @import url("https://use.typekit.net/tsj7mzy.css");

    .niryofooter {{
        text-align:center;
        font-family: {};
        color: #{};
        font-size: {}px;
    }}
</style>
<footer>
  <p class="niryofooter">{}</p>
</footer>
</html>""".format(conf.shared_conf.pdf_footer_font_name,
conf.shared_conf.pdf_footer_font_color,
conf.shared_conf.pdf_footer_font_size,
conf.shared_conf.copyright)'

python_script='
import sys
import subprocess

import conf

argc = len(sys.argv)
if argc != 3:
    print("Wrong number of arguments: {} rather than 3".format(argc))
    sys.exit(0)
'${header_html}'
'${footer_html}'

header_file_name = "header.html"
footer_file_name = "footer.html"

f = open(header_file_name, "w")
f.write(header_html)
f.close()

f = open(footer_file_name, "w")
f.write(footer_html)
f.close()

process = subprocess.Popen(["wkhtmltopdf", "--print-media-type", "--enable-local-file-access", \
"--header-spacing", "{}".format(conf.shared_conf.pdf_header_spacing), "--header-line", "--header-html", "header.html",\
"--footer-spacing", "{}".format(conf.shared_conf.pdf_footer_spacing), "--footer-line", "--footer-html", "footer.html",\
"--javascript-delay", "2000",\
sys.argv[1], sys.argv[2]], stdout=subprocess.PIPE)
process.communicate()[0]

import os

if os.path.exists(header_file_name):
  os.remove(header_file_name)
if os.path.exists(footer_file_name):
  os.remove(footer_file_name)
'

# Create output folder
mkdir -p $OUTPUT_FOLDER
# Loop for every language
for directory_path in _singlehtml/*; do
    # Get language name
    directory=$(basename $directory_path)
    echo ./$OUTPUT_FOLDER/pdf_$directory.pdf
    # Execute script from string
    python3 -c "$python_script" ./$SINGLE_HTML_BUILD_NAME/$directory/index.html ./$OUTPUT_FOLDER/pdf_$directory.pdf;
    done
cd - > /dev/null;
# Remove tmp single html build
rm -rf $SINGLE_HTML_BUILD_NAME