#!/bin/bash

######## Functions
usage()
{
    echo -e "Usage:"
    echo -e "\tgenerateDocRst.sh [options] [folder...]\t\tCreate the core of doc file .rst for ros package"
    echo -e ""
    echo -e "Options:"
    echo -e "\t-h\t\t shows this help message"
    echo -e "\t-p\t\t specify path of doc file where you want to save for package from root doc (sphin_doc/source/)"
}

no_args="true"
#getopts
while getopts 'hp:' flag; do
	case $flag in
		h)
			usage
			exit 0
			;;
		p)
            GENERATE_FILE="${OPTARG%.*}"
			;;	
		?)
            echo -e "Options not found!"
			exit 1;
			;;
	esac
    no_args="false"
done

[[ "$no_args" == "true" ]] && { usage; exit 1; }

# function parse yaml file and add to doc with rst format
# \* - Name of Parameters
# \* - | Description
# \*   | Default: value
function parse_yaml {
    local FILE_OUTPUT=$2
    local prefix=$3
    local s='[[:space:]]*' w='[a-zA-Z0-9_]*' fs=$(echo @|tr @ '\034')
    sed -ne "s|^\($s\):|\1|" \
        -e "s|^\($s\)\($w\)$s:$s[\"']\(.*\)[\"']$s\$|\1$fs\2$fs\3|p" \
        -e "s|^\($s\)\($w\)$s:$s\(.*\)$s\$|\1$fs\2$fs\3|p"  $1 |
    awk -F$fs '{
        indent = length($1)/2;
        vname[indent] = $2;
        for (i in vname) {if (i > indent) {delete vname[i]}}
        if (length($3) > 0) {
            vn=""; for (i=0; i<indent; i++) {vn=(vn)(vname[i])("_")}
            printf("   *  -  ``%s%s%s``\n", "'$prefix'", vn, $2);
            printf("      -  | Description.\n");
            printf("         | Default: %s\n", $3);
        }
    }' >> $FILE_OUTPUT
}

# get script path
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# create doc file
RST_DIR="$SCRIPT_DIR/../source"
FILE="$RST_DIR/$GENERATE_FILE.rst"
touch -a $FILE

# go to file path
cd $( dirname $FILE)

# Get package name
PACKAGE_NAME="$(basename -- $RST_DIR/$GENERATE_FILE)"

echo "Beginning to write in to .rst file"
 > $FILE
# uppercase char after _ in package name
HEADER_UPPERCASE="$(sed -r 's/[_]\s*./\U&\E/g' <<<$PACKAGE_NAME)"
HEADER="$(sed -r 's/[_]+/ /g' <<<${HEADER_UPPERCASE^})"
# uppercase first char + remove "_"
echo "$HEADER" >> $FILE
echo  "=====================================" >> $FILE
echo "" >> $FILE

echo "| Add Description of package here". >> $FILE
echo "" >> $FILE

echo "Name of Important Element in Package" >> $FILE
echo "--------------------------" >> $FILE
echo "[This Element] is used to:" >> $FILE
echo " - Some usages." >> $FILE
echo "" >> $FILE

#########################################
############# PARAMETERS ################
#########################################
echo "Getting parameters in file config ..."

echo "Parameters - $HEADER" >> $FILE
echo "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" >> $FILE
echo "" >> $FILE
echo "\
.. list-table:: $PACKAGE_NAME's Parameters
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center
    
   *  -  Name
      -  Description" >> $FILE

# Get package dir in ros stack
PACKAGE_DIR="$(find $SCRIPT_DIR/../../ -maxdepth 2 -type d -name $PACKAGE_NAME -print -quit)"

# Processing common config file
if [ -d $PACKAGE_DIR/config ]; then
    COMMON_CONFIG="$PACKAGE_DIR/config/*.yaml"
    for f in $COMMON_CONFIG
    do
        echo "processing $(basename -- $f) file..."
        parse_yaml $f $FILE
    done

    if [ -d $PACKAGE_DIR/config/ned2/ ]; then
        # Processing specific config file
        SPECIFIC_CONFIG="$PACKAGE_DIR/config/ned2/*.yaml"
        for f in $SPECIFIC_CONFIG
        do
            echo "processing $(basename -- $f) file ..."
            parse_yaml $f $FILE
        done
    fi
    if [ -d $PACKAGE_DIR/config/ned/ ]; then
        # Processing specific config file
        SPECIFIC_CONFIG="$PACKAGE_DIR/config/ned2/*.yaml"
        for f in $SPECIFIC_CONFIG
        do
            echo "processing $(basename -- $f) file ..."
            parse_yaml $f $FILE
        done
    fi
    if [ -d $PACKAGE_DIR/config/one/ ]; then
        # Processing specific config file
        SPECIFIC_CONFIG="$PACKAGE_DIR/config/ned2/*.yaml"
        for f in $SPECIFIC_CONFIG
        do
            echo "processing $(basename -- $f) file ..."
            parse_yaml $f $FILE
        done
    fi
fi

echo "Get Parameters in file config done"

####################################
########### SERVICE ################
####################################

echo "Getting services ..."
# You have to check if there are others services used in code or not where theses services do not have service file .srv
# This script help you generate only service which use a service file .srv
echo "" >> $FILE
echo "Services - $HEADER" >> $FILE
echo "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" >> $FILE
echo "" >> $FILE

echo "\
.. list-table:: $PACKAGE_NAME Package Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center
   
   *  -  Name
      -  Service Type
      -  Description" >> $FILE

SERVICEDIR="$PACKAGE_DIR/srv/*.srv"
# add service
if [ -d $PACKAGE_DIR/srv ]; then
    echo "Processing package's service ..."
    for f in $SERVICEDIR
    do
        SERVICE_FILE="$(basename -- $f)" 
        SERVICE_FILENAME="${SERVICE_FILE%.*}"
        echo "   *  -  ``[service name]``" >> $FILE
        PACKAGENAME_BASE_ROOTDOC="source/$GENERATE_FILE"
        echo "      -  :ref:\`$SERVICE_FILENAME<$PACKAGENAME_BASE_ROOTDOC:$SERVICE_FILENAME (Service)>\`" >> $FILE
        echo "      -  Description of service." >> $FILE
    done
fi
echo "" >> $FILE
echo "Getting services done"

##########################################
########### Published Topics #############
##########################################

echo "Getting topics ..."
# You have to check if there are others topics used in code or not where theses topics do not have message file .msg
# This script help you generate only message which use a message file .msg
echo "" >> $FILE
echo "Published topics - $HEADER" >> $FILE
echo "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" >> $FILE
echo "" >> $FILE

echo "\
.. list-table:: $PACKAGE_NAME Package Topics
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center
   
   *  -  Name
      -  Message Type
      -  Description" >> $FILE

MESSAGEDIR="$PACKAGE_DIR/msg/*.msg"
# add topics
if [ -d $PACKAGE_DIR/msg ]; then
    echo "Processing package's message ..."
    for f in $MESSAGEDIR
    do
        MESSAGE_FILE="$(basename -- $f)" 
        MESSAGE_FILENAME="${MESSAGE_FILE%.*}"
        echo "   *  -  ``[topics name]``" >> $FILE
        PACKAGENAME_BASE_ROOTDOC="source/$GENERATE_FILE"
        echo "      -  :ref:\`$MESSAGE_FILENAME<$PACKAGENAME_BASE_ROOTDOC:$MESSAGE_FILENAME (Message)>\`" >> $FILE
        echo "      -  Description of topics." >> $FILE
    done
fi
echo "" >> $FILE
echo "Getting topics done"

#########################################
########### DEPENDENCIES ################
#########################################

echo "Dependencies - $HEADER" >> $FILE
echo "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" >> $FILE
echo ""  >> $FILE

#example
#- :wiki_ros:`dynamixel_sdk` 
echo "Add dependencies of package" >> $FILE
echo ""  >> $FILE

echo "Services & Messages files - $HEADER" >> $FILE
echo "--------------------------------------------------" >> $FILE
echo "" >> $FILE

#################################################
#### SERVICES AND MESSAGES FILE IN ROS STACK ####
#################################################
# services
echo "Getting services files in ros stack ..."

if [ -d $PACKAGE_DIR/srv ]; then
    for f in $SERVICEDIR
    do
        SERVICE_FILE="$(basename -- $f)" 
        SERVICE_FILENAME="${SERVICE_FILE%.*}"
        echo "$SERVICE_FILENAME (Service)" >> $FILE
        echo "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" >> $FILE
        echo "" >> $FILE
        DIR_FILE="$(dirname $FILE)"
        SEARCHROOT=""
        while [ "$(realpath $DIR_FILE)" != "$(realpath $SCRIPT_DIR/../../)" ]
        do
            TEMP="$(dirname $(realpath $DIR_FILE))"
            DIR_FILE=$TEMP
            SEARCHROOT+="../"
        done

        RELATIVE_SERVICE_PACKAGE_DIR_FROM_RST_FILE="$(find $SEARCHROOT -name $SERVICE_FILE)"
        echo ".. literalinclude:: $RELATIVE_SERVICE_PACKAGE_DIR_FROM_RST_FILE" >> $FILE
        echo "   :language: rostype" >> $FILE
        echo "" >> $FILE
    done
fi
echo "Getting services files in ros stack done"

# messages
echo "Getting messages files in ros stack ..."

if [ -d $PACKAGE_DIR/msg ]; then
    for f in $MESSAGEDIR
    do
        MESSAGE_FILE="$(basename -- $f)" 
        MESSAGE_FILENAME="${MESSAGE_FILE%.*}"
        echo "$MESSAGE_FILENAME (Message)" >> $FILE
        echo "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" >> $FILE
        echo "" >> $FILE
        DIR_FILE="$(dirname $FILE)"
        SEARCHROOT=""
        while [ "$(realpath $DIR_FILE)" != "$(realpath $SCRIPT_DIR/../../)" ]
        do
            TEMP="$(dirname $(realpath $DIR_FILE))"
            DIR_FILE=$TEMP
            SEARCHROOT+="../"
        done

        RELATIVE_MESSAGE_PACKAGE_DIR_FROM_RST_FILE="$(find $SEARCHROOT -name $MESSAGE_FILE)"
        echo ".. literalinclude:: $RELATIVE_MESSAGE_PACKAGE_DIR_FROM_RST_FILE" >> $FILE
        echo "   :language: rostype" >> $FILE
        echo "" >> $FILE
    done
fi

echo "Getting messages files in ros stack done"
echo "Write to .rst file done successfully"
