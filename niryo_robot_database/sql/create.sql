DROP TABLE IF EXISTS settings;
CREATE TABLE IF NOT EXISTS settings(
    id          TEXT   NOT NULL   PRIMARY KEY,
    name        TEXT   NOT NULL,
    value       TEXT,
    type        TEXT   NOT NULL
);

DROP TABLE IF EXISTS metric;
CREATE TABLE IF NOT EXISTS metric(
    id          TEXT   NOT NULL   PRIMARY KEY,
    name        TEXT   NOT NULL,
    value       TEXT,
    update_date TEXT   NOT NULL
);

DROP TABLE IF EXISTS log;
CREATE TABLE IF NOT EXISTS log(
    id            TEXT      NOT NULL   PRIMARY KEY,
    date          TEXT      NOT NULL,
    severity      TEXT      NOT NULL,
    origin        TEXT      NOT NULL,
    message       TEXT      NOT NULL
);

CREATE TRIGGER log_row_count
BEFORE INSERT ON log
WHEN (SELECT COUNT(*) FROM log) >= 1000
BEGIN
    DELETE FROM log WHERE id in (
        SELECT id
        FROM log
        ORDER BY date
        LIMIT (SELECT COUNT(*) FROM log) - 1000
    );
END;