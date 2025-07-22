CREATE TABLE metrics (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    sucesso REAL,
    perda REAL,
    layer INTEGER,
    nos INTEGER,
    rtt INTEGER,
    trocas_pai INTEGER,
    ult_troca INTEGER,
    retransmissoes INTEGER
);

CREATE TABLE signal (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    mac TEXT,
    rssi INTEGER,
    mac_pai TEXT
);

CREATE TABLE mesh_sensor_data (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    mac TEXT,
    temperatura FLOAT,
    umidade FLOAT,
    latencia INTEGER, 
    hops INTEGER
);
