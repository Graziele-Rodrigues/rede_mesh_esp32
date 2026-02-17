# ESP32 Wi-Fi Mesh com Coleta de Métricas, Sensores e MQTT

Este projeto implementa uma rede **Wi-Fi Mesh utilizando ESP32**, com coleta de dados de sensores, métricas de rede e sistema, sincronização de tempo via NTP e publicação em um broker MQTT.

O sistema permite monitorar em tempo real o desempenho da rede mesh, incluindo latência, RSSI, taxa de sucesso, perda de pacotes, uso de CPU e memória.

## 📌 Funcionalidades

* Criação de rede Wi-Fi Mesh com ESP32
* Comunicação entre nós utilizando protocolo personalizado
* Publicação de dados via MQTT
* Coleta de métricas da rede:

  * RSSI
  * Latência
  * Número de saltos (hops)
  * Taxa de entrega e perda de pacotes
  * Número de filhos
  * Trocas de nó pai
* Monitoramento do sistema:

  * Temperatura interna do ESP32
  * Memória heap livre
  * Uso de CPU
* Sincronização de tempo via SNTP/NTP
* Sincronização de horário entre os nós da malha
* CRC-16 para integridade dos dados
* Tabela de roteamento da mesh


## 🧱 Arquitetura da Rede

A rede é composta por três tipos de nós:

* **Root node**

  * Conectado ao Wi-Fi e MQTT broker
  * Recebe dados dos demais nós
  * Publica dados no MQTT
  * Sincroniza o horário

* **Intermediate nodes**

  * Roteiam dados
  * Enviam métricas e sensores

* **Leaf nodes**

  * Enviam dados de sensores
  * Não possuem filhos

---

## 📡 Estrutura do Pacote

O protocolo utiliza a seguinte estrutura:

```c
typedef struct {
    uint8_t mac[6];
    uint8_t id;
    uint16_t num_bytes;
    uint8_t payload[1024];
    uint16_t crc;
} mesh_packet_t;
```

Tipos de comando:

| ID              | Descrição              |
| --------------- | ---------------------- |
| CMD_SENSOR      | Dados de sensor        |
| CMD_METRICS     | Métricas RSSI e pai    |
| CMD_ROUTE_TABLE | Tabela de roteamento   |
| CMD_SYNC_TIME   | Sincronização de tempo |
| CMD_RTT         | Medição de RTT         |

---

## 📊 Dados Publicados no MQTT

### Sensor

Tópico:

```
/topic/mesh/sensor
```

Exemplo:

```json
{
  "mac":"24:6f:28:aa:bb:cc",
  "temp":25.3,
  "hum":60.1,
  "latency_ms":45,
  "hops":2,
  "temp_interna":36.2,
  "free_heap":198432,
  "cpu":23.5,
  "cpu_idle":76.5
}
```

---

### RSSI

```
/topic/mesh/rssi
```

---

### Métricas da rede

```
/topic/mesh/metricas
```

Exemplo:

```json
{
  "layer":3,
  "success_rate":98.2,
  "packet_loss_rate":1.8,
  "children_count":4,
  "parent_changes":2,
  "last_parent_change_ms":1500
}
```


### Sistema Root

```
/topic/mesh/sistema_root
```


## ⏱ Frequência de envio

| Tipo                   | Intervalo |
| ---------------------- | --------- |
| Sensor                 | 5 minutos |
| Métricas               | 5 minutos |
| Sincronização de tempo | 1 minuto  |


## 🧮 Métricas calculadas

* Taxa de sucesso:

```
success_rate = total_received / total_sent
```

* Perda de pacotes:

```
packet_loss = 100 − success_rate
```

* Latência:

```
latency = timestamp_recebido − timestamp_enviado
```


## 🕒 Sincronização de Tempo

O nó root sincroniza com NTP:

```
pool.ntp.org
```

E distribui o horário para todos os nós da malha.


## 🔐 Integridade dos dados

Utiliza CRC-16 (MODBUS):

```c
uint16_t calculate_crc16(...)
```

---

## 🧵 Tasks FreeRTOS utilizadas

| Task              | Função               |
| ----------------- | -------------------- |
| sensor_send_task  | envia dados sensores |
| metrics_send_task | envia métricas       |
| sync_time_task    | sincroniza horário   |
| ping_send_task    | mede RTT             |

---

## ⚙️ Requisitos

### Hardware

* ESP32 (qualquer versão compatível com ESP-MESH)
* Sensor interno de temperatura (built-in ESP32)

### Software

* ESP-IDF v5.x ou superior
* MQTT broker (ex: Mosquitto)
* Wi-Fi


## 📦 Dependências ESP-IDF

Bibliotecas utilizadas:

```
esp_wifi
esp_mesh
esp_event
esp_netif
esp_sntp
mqtt
freertos
nvs_flash
driver
```


## 🚀 Como usar

### 1. Configurar ESP-IDF

```bash
idf.py menuconfig
```

Configure:

* WiFi SSID
* WiFi Password
* MQTT broker


### 2. Compilar

```bash
idf.py build
```

### 3. Gravar no ESP32

```bash
idf.py flash monitor
```


## 📈 Aplicações

* Monitoramento de redes IoT
* Redes industriais (IIoT)
* Sistemas distribuídos
* Redes resilientes
* Pesquisa acadêmica em redes mesh

