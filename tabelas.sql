CREATE TABLE metrics (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    hops INTEGER,
    sucesso REAL,
    perda REAL,
    rtt INTEGER,
    nos INTEGER,
    trocas_pai INTEGER,
    ult_troca INTEGER,
    retransmissoes INTEGER
);

CREATE TABLE sinal (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    mac TEXT,
    rssi INTEGER
);

CREATE TABLE mesh_sensor (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    mac TEXT,
    temperatura FLOAT,
    umidade FLOAT,
    latencia INTEGER
);
