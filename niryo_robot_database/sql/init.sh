#!/usr/bin/env bash

DB_PATH="$HOME/.config/niryo/ned.db"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

if [[ ! -f  $DB_PATH ]]; then
    touch $DB_PATH
fi

sqlite3 "$DB_PATH" < "$SCRIPT_DIR/create.sql"
sqlite3 "$DB_PATH" < "$SCRIPT_DIR/populate.sql"