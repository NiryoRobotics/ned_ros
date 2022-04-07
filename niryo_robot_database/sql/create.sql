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

DROP TABLE IF EXISTS robot_version;
CREATE TABLE IF NOT EXISTS robot_version(
    id          TEXT   NOT NULL   PRIMARY KEY,
    name        TEXT   NOT NULL,
    value       TEXT,
    update_date TEXT   NOT NULL
);

DROP TABLE IF EXISTS joints;
CREATE TABLE IF NOT EXISTS joints(
    joint_name          TEXT   NOT NULL   PRIMARY KEY,
    motor               TEXT   NOT NULL,
    firmware_version    TEXT   NOT NULL,
    error_code          TEXT,
    update_date         TEXT   NOT NULL
);