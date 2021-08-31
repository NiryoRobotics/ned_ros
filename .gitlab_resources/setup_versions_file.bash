#! /bin/bash

if [ $# -lt 2 ]; then
    echo "Not enough parameters provided (FILE_URL and NEW_VERSION are needed)";
    exit 1;
elif [ $# -gt 2 ]; then
    echo "Too much parameters provided, ignoring them"
fi

VERSION_FILE_NAME=versions.json
VERSION_TEMPORARY_FILE_LOCAL_NAME=versions_tmp.json
VERSION_FILE_URL=$1
NEW_VERSION=$2

wget -O $VERSION_FILE_NAME $VERSION_FILE_URL/$VERSION_FILE_NAME;
VERSION_FILE_ALREADY_EXISTS=$?
# Create file if it wasn't retrieved (doesn't exist server side)
if [[ VERSION_FILE_ALREADY_EXISTS -ne 0 ]]; then
    echo "Version file doesn't exist, creating it"
    echo \
"\
{
    \"versions\":
    [
    ]
}" > $VERSION_FILE_NAME
fi

# Check if versions already exists
jq -e ".versions | index(\"$NEW_VERSION\")" $VERSION_FILE_NAME > /dev/null;
VERSION_ALREADY_EXISTS=$?
# If versions exists => We do not add version but only update an already existing version
# If version do not exist => We add the new version
if [ $VERSION_ALREADY_EXISTS -ne 0 ]; then
    echo "New versions: adding version to versions.json file in progress..."
    # Append new versions to versions file
    jq ".versions += [\"$NEW_VERSION\"]" $VERSION_FILE_NAME > $VERSION_TEMPORARY_FILE_LOCAL_NAME
    VERSION_FILE_UPDATED_RESULT=$?
    if [ $VERSION_FILE_UPDATED_RESULT -eq 0 ]; then
        mv $VERSION_TEMPORARY_FILE_LOCAL_NAME $VERSION_FILE_NAME
        echo "Version file properly updated"
        exit 0;
    else
        echo "Modifying version file failed"
        rm -f $VERSION_TEMPORARY_FILE_LOCAL_NAME $VERSION_FILE_NAME;
        exit 1;
    fi
else
    echo "Version already exists: nothing to do with versions.json file"
fi