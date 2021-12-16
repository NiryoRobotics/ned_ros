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

DROP TABLE IF EXISTS file_path;
CREATE TABLE IF NOT EXISTS file_path(
    id   TEXT NOT NULL PRIMARY KEY,
    type TEXT NOT NULL,
    name TEXT NOT NULL,
    date TEXT NOT NULL,
    path TEXT NOT NULL
);
