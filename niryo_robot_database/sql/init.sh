DB_PATH="$HOME/.config/niryo/ned.db"

if [[ ! -f  $DB_PATH ]]; then
    touch $DB_PATH
fi

sqlite3 $DB_PATH < create.sql
sqlite3 $DB_PATH < populate.sql