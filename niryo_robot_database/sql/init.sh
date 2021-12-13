#!/usr/bin/env bash

sqlite3 -v > /dev/null 2>&1

if [ $? -ne 1 ] ; then
    echo "sqlite3 is not installed"
    exit 1
fi

DB_PATH="$HOME/.config/niryo/ned.db"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

if [[ ! -f  $DB_PATH ]]; then
    touch $DB_PATH
fi

sqlite3 "$DB_PATH" < "$SCRIPT_DIR/create.sql"
sqlite3 "$DB_PATH" < "$SCRIPT_DIR/populate.sql"