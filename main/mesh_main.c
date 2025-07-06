#include <string.h>
#include <inttypes.h>
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "nvs_flash.h"
#include "mesh_netif.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"

#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "time.h"
#include "lwip/ip_addr.h"

#include <sys/time.h>
#include <inttypes.h> // Para lidar com uint64_t em logs
#include <time.h>

/*******************************************************
 *                Macros
 *******************************************************/

#define PAYLOAD_SIZE 200
#define PACKET_HEADER_SIZE 8 + 1 + 2  // MAC (6) + ID (1) + NUM_BYTES (2)
#define CMD_ROUTE_TABLE 0x01
#define CMD_METRICS     0x02
#define CMD_SENSOR      0x03
#define CMD_RTT 0x05
#define CMD_SYNC_TIME 0xAA

typedef struct {
    uint8_t mac[6];
    uint8_t id;
    uint16_t num_bytes;
    uint8_t payload[PAYLOAD_SIZE];
    uint16_t crc;
} mesh_packet_t;

typedef struct {
    uint8_t layer;
    float success_rate;
    float packet_loss_rate;
    uint32_t rtt_ms;
    uint8_t children_count;
    uint16_t subtree_size;
    uint16_t parent_changes;
    uint64_t last_parent_change_ms;
    uint8_t retransmission_count;
} mesh_metrics_t;

mesh_metrics_t metrics;

/*******************************************************
 *                Constants
 *******************************************************/
static const char *MESH_TAG = "mesh_main";
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x76};

/*******************************************************
 *                Variable Definitions
 *******************************************************/
static mesh_addr_t mesh_parent_addr;
static int mesh_layer = -1;
static esp_ip4_addr_t s_current_ip;
static mesh_addr_t s_route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
static int s_route_table_size = 0;
static SemaphoreHandle_t s_route_table_lock = NULL;

// Variaveis para as metricas de avaliação

int8_t parent_rssi = 0;
volatile uint64_t timestamp_send = 0;
volatile uint64_t timestamp_recv;
volatile uint16_t packets_sent = 0;
volatile uint16_t packets_received = 0;
uint64_t reconnection_time = 0;
uint64_t timestamp_last_parent_change = 0;
int parent_changes = 0;
int parent_children_count = 0;
int child_count = 0;
int children_count = 0;
int64_t time_offset = 0;  
static bool ntp_synced = false;  
volatile bool time_synced = false;

typedef struct {
    uint8_t mac[6];
    uint16_t last_sent;
} node_info_t;

#define MAX_NODES 10
node_info_t nodes[MAX_NODES];
int node_count = 0;
float packet_loss = 0.0;  // Taxa de perda de pacotes
float success_rate = 0.0; // Taxa de sucesso de pacotes
uint32_t total_sent = 0;
uint32_t total_received = 0;

/*******************************************************
 *                Function Declarations
 *******************************************************/
// interaction with public mqtt broker
void mqtt_app_start(void);
void mqtt_app_publish(char* topic, char *publish_string);

/*******************************************************
 *                Function Definitions
 *******************************************************/

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI("NTP", "Time synchronized");
}

bool obtain_time(void)
{
    ESP_LOGI("NTP", "Inicializando SNTP...");

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");

    config.start = false; // iniciaremos manualmente
    config.sync_cb = time_sync_notification_cb;

#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    config.smooth_sync = true;
#endif

    esp_netif_sntp_init(&config);
    esp_netif_sntp_start();

    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 20;

    while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count) {
        ESP_LOGI("NTP", "Aguardando sincronização SNTP... (%d/%d)", retry, retry_count);
    }

    time(&now);
    localtime_r(&now, &timeinfo);

    if (timeinfo.tm_year < (2020 - 1900)) {
        ESP_LOGE("NTP", "Falha ao sincronizar NTP");
        return false;
    }

    ESP_LOGI("NTP", "Tempo sincronizado: %s", asctime(&timeinfo));
    return true;
}

 // funcao para obter o MAC
 void get_mac_str(char *mac_str, size_t len) {
    uint8_t mac[6];
    esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, mac);
    if (err == ESP_OK) {
        snprintf(mac_str, len, "%02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else {
        strcpy(mac_str, "UNKNOWN_MAC");
        ESP_LOGE(MESH_TAG, "Failed to get MAC address");
    }
}


// Mede o tempo de troca de nó pai
void update_parent_change_time() {

    // Timestamp de recebimento
    struct timeval tv_recv;
    gettimeofday(&tv_recv, NULL);
    uint64_t timestamp_recv = (uint64_t)tv_recv.tv_sec * 1000 + tv_recv.tv_usec / 1000;
    uint64_t timestamp_now = timestamp_recv + time_offset;

    if (timestamp_last_parent_change > 0) {
        uint64_t reconnection_time = timestamp_now - timestamp_last_parent_change;
        ESP_LOGI(MESH_TAG, "Tempo desde última troca de pai: %" PRIu64 " ms", reconnection_time);
    } else {
        ESP_LOGI(MESH_TAG, "Primeira conexão com pai registrada.");
    }

    timestamp_last_parent_change = timestamp_now;
    parent_changes++;
}

void calculate_rates() {
    if (total_sent == 0) {
        success_rate = 0.0;
        packet_loss = 0.0;
        return;
    }

    success_rate = ((float)total_received / total_sent) * 100.0;
    packet_loss = 100.0 - success_rate;

    ESP_LOGI(MESH_TAG, "Taxa de sucesso: %.2f%%, perda: %.2f%%", success_rate, packet_loss);
}

void update_packet_counters(uint8_t *mac, uint16_t current_sent) {
    bool node_found = false;
    
    for (int i = 0; i < node_count; i++) {
        if (memcmp(mac, nodes[i].mac, 6) == 0) {
            // Para nós existentes, incrementa apenas 1 no total_sent (o pacote atual)
            total_sent += 1;
            nodes[i].last_sent = current_sent;
            total_received += 1;
            node_found = true;
            break;
        }
    }

    // Novo nó
    if (!node_found && node_count < MAX_NODES) {
        memcpy(nodes[node_count].mac, mac, 6);
        nodes[node_count].last_sent = current_sent;
        node_count++;
        // Para novo nó, conta apenas o pacote atual
        total_sent += 1;
        total_received += 1;
    }

    // Chama calculate_rates apenas se necessário (opcional)
    static uint32_t last_calc = 0;
    if (total_sent - last_calc >= 10) { // A cada 10 pacotes, por exemplo
        calculate_rates();
        last_calc = total_sent;
    }
}


// Calcula o tempo de ida e volta (RTT) entre o envio e recebimento de pacotes
uint32_t calculate_rtt() {
    if (timestamp_send == 0 || timestamp_recv == 0) return 0;

    // Calcula diferença primeiro, depois converte
    return (uint32_t)(timestamp_recv - timestamp_send);
}


// Função para calcular CRC-16
uint16_t calculate_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;  // Polinômio CRC-16 MODBUS
            else
                crc = crc >> 1;
        }
    }
    return crc;
}

// Funcao montagem de pacotes
void build_packet(mesh_packet_t *packet, uint8_t id, uint8_t *data, uint16_t len) {
    memcpy(packet->mac, mesh_netif_get_station_mac(), 6);
    packet->id = id;
    packet->num_bytes = len;
    memcpy(packet->payload, data, len);
    packet->crc = calculate_crc16((uint8_t *)packet, 6 + 1 + 2 + len); // Simples CRC-16
}

// funcao processamento recepcao dados
void static recv_cb(mesh_addr_t *from, mesh_data_t *data)
{
   
    mesh_packet_t *packet = (mesh_packet_t *)data->data;

    // Verifica CRC
    uint16_t computed_crc = calculate_crc16((uint8_t *)packet, 6 + 1 + 2 + packet->num_bytes);
    if (computed_crc != packet->crc) {
        ESP_LOGW(MESH_TAG, "CRC inválido! Esperado: 0x%04X, Recebido: 0x%04X", computed_crc, packet->crc);
        return;
    }

    // Converte MAC para string
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), MACSTR, MAC2STR(packet->mac));

    // Processa com base no ID do pacote
    switch (packet->id) {
        case CMD_SENSOR: {
            if (esp_mesh_is_root()) {
                packets_received++;

                struct timeval tv_start;
                gettimeofday(&tv_start, NULL);
                uint64_t processing_start = (uint64_t)tv_start.tv_sec * 1000 + tv_start.tv_usec / 1000;

                uint64_t timestamp;
                float temp, hum;
                uint16_t sent;
                memcpy(&timestamp, packet->payload, sizeof(uint64_t));
                memcpy(&temp, packet->payload + 8, sizeof(float));
                memcpy(&hum, packet->payload + 12, sizeof(float));
                memcpy(&sent, packet->payload + 16, sizeof(uint16_t));

                // Timestamp de recebimento
                struct timeval tv_recv;
                gettimeofday(&tv_recv, NULL);
                uint64_t timestamp_recv = (uint64_t)tv_recv.tv_sec * 1000 + tv_recv.tv_usec / 1000;

                // Tempo de processamento local
                uint64_t processing_time = timestamp_recv - processing_start;

                // Latência = tempo de recebimento - timestamp enviado - tempo de processamento
                uint64_t latency_ms = timestamp_recv - timestamp - processing_time;

                char msg[150];
                snprintf(msg, sizeof(msg),
                        "{\"mac\":\"%s\",\"temp\":%.2f,\"hum\":%.2f,\"latency_ms\":%" PRIu64 "}",
                        mac_str, temp, hum, latency_ms);
                mqtt_app_publish("/topic/mesh/sensor", msg);
                ESP_LOGI(MESH_TAG, "MQTT publicado em /topic/mesh/sensor: %s", msg);

                update_packet_counters(from->addr, sent);
            }
            break;
        }

        case CMD_METRICS: {
            if (esp_mesh_is_root()) {
                if (packet->num_bytes < sizeof(int8_t)) {
                    ESP_LOGW(MESH_TAG, "Payload de métrica inválido");
                        return;
                }
                packets_received++;

                int8_t rssi;
                uint16_t sent;

                memcpy(&rssi, packet->payload, sizeof(int8_t));
                memcpy(&sent, packet->payload + 1, sizeof(uint16_t));

                char msg[100];
                snprintf(msg, sizeof(msg),
                         "{\"mac\":\"%s\",\"rssi\":%d}", mac_str, rssi);
                mqtt_app_publish("/topic/mesh/rssi", msg);
                ESP_LOGI(MESH_TAG, "MQTT publicado em /topic/mesh/rssi: %s", msg);

                update_packet_counters(packet->mac, sent);
                calculate_rates();
            }
            break;
        }

        case CMD_RTT: {
            if (esp_mesh_is_root()) {
                // Responde o pacote de volta para quem enviou
                mesh_packet_t response;
                build_packet(&response, CMD_RTT, NULL, 0);

                mesh_data_t data = {
                    .data = (uint8_t *)&response,
                    .size = sizeof(mesh_packet_t), 
                    .proto = MESH_PROTO_BIN,
                    .tos = MESH_TOS_P2P
                };
                esp_mesh_send(from, &data, MESH_DATA_P2P, NULL, 0);
                ESP_LOGI(MESH_TAG, "Pong enviado");
            } else {
                // Se sou um nó e recebi o pong, calculo o RTT
                struct timeval tv_recv;
                gettimeofday(&tv_recv, NULL);
                uint64_t timestamp_recv = (uint64_t)tv_recv.tv_sec * 1000 + tv_recv.tv_usec / 1000;
                metrics.rtt_ms = calculate_rtt();
            }
            break;
        }

        case CMD_SYNC_TIME: {
            if (packet->num_bytes != sizeof(uint64_t)) break;

            uint64_t time_from_root;
            memcpy(&time_from_root, packet->payload, sizeof(uint64_t));

            struct timeval now;
            gettimeofday(&now, NULL);  // Pega o tempo absoluto (RTC/NTP)
            uint64_t local_time = (uint64_t)now.tv_sec * 1000 + now.tv_usec / 1000; // em ms

            time_offset = (int64_t)time_from_root - (int64_t)local_time;
            time_synced = true;

            ESP_LOGI(MESH_TAG, "Delta de sincronização (ms): %" PRId64, time_offset);
            break;
        }

        case CMD_ROUTE_TABLE: {
            int size = packet->num_bytes;
            if (s_route_table_lock == NULL || size % 6 != 0) {
                ESP_LOGE(MESH_TAG, "Erro na tabela de roteamento recebida");
                return;
            }

            xSemaphoreTake(s_route_table_lock, portMAX_DELAY);
            s_route_table_size = size / 6;
            memcpy(&s_route_table, packet->payload, size);
            xSemaphoreGive(s_route_table_lock);

            for (int i = 0; i < s_route_table_size; ++i) {
                ESP_LOGI(MESH_TAG, "Recebido MAC[%d]: " MACSTR, i, MAC2STR(s_route_table[i].addr));
            }
            break;
        }

        default:
            ESP_LOGW(MESH_TAG, "Comando desconhecido: %d", packet->id);
            break;
    }
}


// funcao tabela roteamento
void send_routing_table() {
    esp_err_t err;
    char *route_table_json = malloc(256);
    char *ptr = route_table_json;

    // 1. Obtem a tabela de roteamento
    esp_mesh_get_routing_table((mesh_addr_t *)&s_route_table,
                               CONFIG_MESH_ROUTE_TABLE_SIZE * 6,
                               &s_route_table_size);

    // 2. Publica a tabela como JSON no MQTT 
    ptr += sprintf(ptr, "{\"route_table\":[");
    for (int i = 0; i < s_route_table_size; i++) {
        if (i > 0) ptr += sprintf(ptr, ",");
        ptr += sprintf(ptr, "\"%02x:%02x:%02x:%02x:%02x:%02x\"",
                       s_route_table[i].addr[0], s_route_table[i].addr[1],
                       s_route_table[i].addr[2], s_route_table[i].addr[3],
                       s_route_table[i].addr[4], s_route_table[i].addr[5]);
    }
    sprintf(ptr, "]}");

    free(route_table_json);

    // Monta o payload com os MACs dos nós da tabela
    uint16_t routing_data_len = s_route_table_size * sizeof(mesh_addr_t);
    uint8_t *routing_payload = malloc(routing_data_len);
    memcpy(routing_payload, s_route_table, routing_data_len);

    // Constrói o pacote usando seu protocolo
    mesh_packet_t packet;
    build_packet(&packet, CMD_ROUTE_TABLE, routing_payload, routing_data_len);

    // Prepara para envio via malha
    mesh_data_t data;
    data.data = (uint8_t *)&packet;
    data.size = sizeof(mesh_packet_t);  // tamanho total do pacote
    data.proto = MESH_PROTO_BIN;
    data.tos = MESH_TOS_P2P;

    // Envia o pacote para todos os nós da tabela
    for (int i = 0; i < s_route_table_size; i++) {
        err = esp_mesh_send(&s_route_table[i], &data, MESH_DATA_P2P, NULL, 0);
        ESP_LOGI(MESH_TAG, "Sending routing table to [%d] " MACSTR " | err: %d",
                 i, MAC2STR(s_route_table[i].addr), err);
    }

    free(routing_payload);  // libera o buffer
}

// Envio dados sensores a cada 2 minutos
static void sensor_send_task(void *args) {
    while (true) {
        if (!esp_mesh_is_root()) {
             if (!time_synced) {
                ESP_LOGW(MESH_TAG, "Aguardando sincronização de horário...");
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                continue;
            }
            packets_sent++;
            float temp =  18.0 + (rand() % 1300) / 100.0;
            float hum  = 60.0 + (rand() % 3100) / 100.0;
            uint16_t sent = packets_sent;

            struct timeval tv_recv;
            gettimeofday(&tv_recv, NULL);
            uint64_t local_ts = (uint64_t)tv_recv.tv_sec * 1000 + tv_recv.tv_usec / 1000;

            uint64_t adjusted_ts = local_ts + time_offset; // ms, timestamp ajustado
            
            uint8_t buffer[20];

            memcpy(buffer, &adjusted_ts, sizeof(adjusted_ts));
            memcpy(buffer + 8, &temp, sizeof(temp));
            memcpy(buffer + 12, &hum, sizeof(hum));
            memcpy(buffer + 16, &sent, sizeof(sent)); 

            mesh_packet_t packet;
            build_packet(&packet, CMD_SENSOR, buffer, sizeof(buffer));

            mesh_data_t data = {
                .data  = (uint8_t *)&packet,
                .size  = sizeof(mesh_packet_t),
                .proto = MESH_PROTO_BIN,
                .tos   = MESH_TOS_P2P
            };

            mesh_addr_t parent_addr;
            esp_err_t err = esp_mesh_get_parent_bssid(&parent_addr);    
            if (err == ESP_OK) {
                err = esp_mesh_send(&parent_addr, &data, MESH_DATA_P2P, NULL, 0);
                if (err == ESP_OK) {
                    ESP_LOGI(MESH_TAG, "Dados de sensor enviados ao pai " MACSTR, MAC2STR(parent_addr.addr));
                } else {
                    ESP_LOGW(MESH_TAG, "Falha ao enviar dados ao pai: %d", err);
                }
            } else {
                ESP_LOGW(MESH_TAG, "Não foi possível obter endereço do nó pai");
            }
        }

        vTaskDelay(2 * 60 * 1000 / portTICK_PERIOD_MS);  // a cada 2 minutos
    }
}


// Métricas Gerais
void metricas() {
    // Conectividade
    metrics.layer = esp_mesh_get_layer();
    metrics.success_rate = success_rate;
    metrics.packet_loss_rate = packet_loss;

    // Topologia
    metrics.children_count = children_count;
    metrics.parent_changes = parent_changes;

    // Tempo última troca de pai
    metrics.last_parent_change_ms = reconnection_time;

    // Inicializar outras métricas (implementar conforme necessário)
    metrics.subtree_size = 0;
    metrics.retransmission_count = 0;

    char payload[512];
    snprintf(payload, sizeof(payload),
        "{"
        "\"layer\":%d,"
        "\"success_rate\":%.2f,"
        "\"packet_loss_rate\":%.2f,"
        "\"rtt_ms\":%" PRIu32 ","
        "\"children_count\":%d,"
        "\"parent_changes\":%d,"
        "\"last_parent_change_ms\":%" PRIu64 ","
        "\"retransmission_count\":%d"
        "}",
        metrics.layer,
        metrics.success_rate,
        metrics.packet_loss_rate,
        metrics.rtt_ms,
        metrics.children_count,
        metrics.parent_changes,
        metrics.last_parent_change_ms,
        metrics.retransmission_count
    );

    if (esp_mesh_is_root()) {
        mqtt_app_publish("/topic/mesh/metricas", payload);
        ESP_LOGI(MESH_TAG, "Métricas publicadas: %s", payload);
        int8_t rssi = -127;
        wifi_ap_record_t ap_info;

        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            rssi = ap_info.rssi;
        } else {
            ESP_LOGW(MESH_TAG, "Falha ao obter RSSI do root em relação ao AP");
        }

        char mac_str[18];
        get_mac_str(mac_str, sizeof(mac_str));  // Usa sua função existente

        char msg[100];
        snprintf(msg, sizeof(msg),
                "{\"mac\":\"%s\",\"rssi\":%d}", mac_str, rssi);
        mqtt_app_publish("/topic/mesh/rssi", msg);
        ESP_LOGI(MESH_TAG, "Root publicou seu RSSI: %s", msg);

    }

}

// Envio de metricas
static void metrics_send_task(void *args) {
    while (true) {
        if (!esp_mesh_is_root()) {
            packets_sent++;

            int8_t rssi;
            uint16_t sent = packets_sent;
            // Obtém o RSSI do nó pai
            esp_wifi_sta_get_rssi((int *)&rssi);
            

            uint8_t buffer[10] = {0};
            memcpy(buffer, &rssi, sizeof(rssi));
            memcpy(buffer + 1, &sent, sizeof(sent)); 

            mesh_packet_t packet;
            build_packet(&packet, CMD_METRICS, buffer, sizeof(rssi));

            mesh_data_t data = {
                .data  = (uint8_t *)&packet,
                .size  = sizeof(mesh_packet_t),
                .proto = MESH_PROTO_BIN,
                .tos   = MESH_TOS_P2P
            };

            mesh_addr_t parent_addr;
            esp_err_t err = esp_mesh_get_parent_bssid(&parent_addr); 
            if (err == ESP_OK) {
                err = esp_mesh_send(&parent_addr, &data, MESH_DATA_P2P, NULL, 0);
                if (err == ESP_OK) {
                    ESP_LOGI(MESH_TAG, "Métrica enviada ao pai " MACSTR ": RSSI=%d", MAC2STR(parent_addr.addr), rssi);
                } else {
                    ESP_LOGW(MESH_TAG, "Falha ao enviar métrica ao pai: %d", err);
                }
            } else {
                ESP_LOGW(MESH_TAG, "Não foi possível obter endereço do nó pai");
            }
        }
        if(esp_mesh_is_root()) {
            metricas(); // Atualiza as métricas
        }
        vTaskDelay(2 * 60 * 1000 / portTICK_PERIOD_MS);  // a cada 2 minutos
    }
}

// ping_send_task - para medir RTT
static void ping_send_task(void *args) {
    while (true) {
        if (!esp_mesh_is_root()) {
            // Timestamp de recebimento
            struct timeval tv_recv;
            gettimeofday(&tv_recv, NULL);
            uint64_t timestamp_send = (uint64_t)tv_recv.tv_sec * 1000 + tv_recv.tv_usec / 1000;

            mesh_packet_t packet;
            build_packet(&packet, CMD_RTT, NULL, 0);

            mesh_data_t data = {
                .data = (uint8_t *)&packet,
                .size = sizeof(mesh_packet_t), 
                .proto = MESH_PROTO_BIN,
                .tos = MESH_TOS_P2P
            };

            mesh_addr_t parent_addr;
            esp_err_t err = esp_mesh_get_parent_bssid(&parent_addr);
            if (err == ESP_OK) {
                esp_mesh_send(&parent_addr, &data, MESH_DATA_P2P, NULL, 0);
                packets_sent++;
                ESP_LOGI(MESH_TAG, "Ping enviado para o pai");
            }
        }

        vTaskDelay(2 * 60 * 1000 / portTICK_PERIOD_MS);  // a cada 2 minutos
    }
}

void send_time_to_children() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t unix_now_ms = (uint64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;

    mesh_packet_t packet;
    build_packet(&packet, CMD_SYNC_TIME, (uint8_t*)&unix_now_ms, sizeof(unix_now_ms));

    mesh_data_t data = {
        .data = (uint8_t*)&packet,
        .size = sizeof(mesh_packet_t),
        .proto = MESH_PROTO_BIN,
        .tos = MESH_TOS_P2P
    };

    send_routing_table(); // Atualiza a tabela de roteamento antes de enviar

    for (int i = 0; i < s_route_table_size; i++) {
        esp_mesh_send(&s_route_table[i], &data, MESH_DATA_P2P, NULL, 0);
    }

    ESP_LOGI(MESH_TAG, "Horário sincronizado enviado para filhos: %" PRIu64 " ms", unix_now_ms);
}

// task para sincronizar o horário com os filhos
static void sync_time_task(void *args) {
    while (true) {
        if (esp_mesh_is_root()) {
            send_time_to_children();
        }
        vTaskDelay(1 * 60 * 1000 / portTICK_PERIOD_MS);  // a cada 1 minuto
    }
}

esp_err_t esp_mesh_comm_mqtt_task_start(void)
{
    static bool is_comm_mqtt_task_started = false;

    s_route_table_lock = xSemaphoreCreateMutex();

    if (!is_comm_mqtt_task_started) {
        xTaskCreate(sensor_send_task, "sensor_send_task", 4096, NULL, 8, NULL);
        xTaskCreate(metrics_send_task, "metrics_send_task", 4096, NULL, 5, NULL);
        xTaskCreate(ping_send_task, "ping_send_task", 4096, NULL, 7, NULL);
        mqtt_app_start();
        is_comm_mqtt_task_started = true;
    }
    return ESP_OK;
}

void mesh_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    mesh_addr_t id = {0,};
    static uint8_t last_layer = 0;

    switch (event_id) {
    case MESH_EVENT_STARTED: {
        esp_mesh_get_id(&id);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_MESH_STARTED>ID:"MACSTR"", MAC2STR(id.addr));
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOPPED>");
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_CHILD_CONNECTED: {
        mesh_event_child_connected_t *child_connected = (mesh_event_child_connected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_CONNECTED>aid:%d, "MACSTR"",
                 child_connected->aid,
                 MAC2STR(child_connected->mac));
        children_count = esp_mesh_get_total_node_num();
    }
    break;
    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *child_disconnected = (mesh_event_child_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_DISCONNECTED>aid:%d, "MACSTR"",
                 child_disconnected->aid,
                 MAC2STR(child_disconnected->mac));
        children_count = esp_mesh_get_total_node_num();
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_ADD: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_ADD>add %d, new:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new);
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_REMOVE: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_REMOVE>remove %d, new:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new);
    }
    break;
    case MESH_EVENT_NO_PARENT_FOUND: {
        mesh_event_no_parent_found_t *no_parent = (mesh_event_no_parent_found_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NO_PARENT_FOUND>scan times:%d",
                 no_parent->scan_times);
    }
    /* TODO handler for the failure */
    break;
    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *connected = (mesh_event_connected_t *)event_data;
        esp_mesh_get_id(&id);
        mesh_layer = connected->self_layer;
        memcpy(&mesh_parent_addr.addr, connected->connected.bssid, 6);
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_CONNECTED>layer:%d-->%d, parent:"MACSTR"%s, ID:"MACSTR"",
                 last_layer, mesh_layer, MAC2STR(mesh_parent_addr.addr),
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "", MAC2STR(id.addr));
        last_layer = mesh_layer;
        ESP_LOGI(MESH_TAG, "Nó mudou de pai.");
        update_parent_change_time();  // Atualiza o tempo de troca do nó pai
        mesh_netifs_start(esp_mesh_is_root());
    }
    break;
    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *disconnected = (mesh_event_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_DISCONNECTED>reason:%d",
                 disconnected->reason);
        mesh_layer = esp_mesh_get_layer();
        mesh_netifs_stop();
    }
    break;
    case MESH_EVENT_LAYER_CHANGE: {
        mesh_event_layer_change_t *layer_change = (mesh_event_layer_change_t *)event_data;
        mesh_layer = layer_change->new_layer;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_LAYER_CHANGE>layer:%d-->%d%s",
                 last_layer, mesh_layer,
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "");
        last_layer = mesh_layer;
    }
    break;
    case MESH_EVENT_ROOT_ADDRESS: {
        mesh_event_root_address_t *root_addr = (mesh_event_root_address_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_ADDRESS>root address:"MACSTR"",
                 MAC2STR(root_addr->addr));
    }
    break;
    case MESH_EVENT_VOTE_STARTED: {
        mesh_event_vote_started_t *vote_started = (mesh_event_vote_started_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_VOTE_STARTED>attempts:%d, reason:%d, rc_addr:"MACSTR"",
                 vote_started->attempts,
                 vote_started->reason,
                 MAC2STR(vote_started->rc_addr.addr));
    }
    break;
    case MESH_EVENT_VOTE_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_VOTE_STOPPED>");
        break;
    }
    case MESH_EVENT_ROOT_SWITCH_REQ: {
        mesh_event_root_switch_req_t *switch_req = (mesh_event_root_switch_req_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_ROOT_SWITCH_REQ>reason:%d, rc_addr:"MACSTR"",
                 switch_req->reason,
                 MAC2STR( switch_req->rc_addr.addr));
    }
    break;
    case MESH_EVENT_ROOT_SWITCH_ACK: {
        /* new root */
        mesh_layer = esp_mesh_get_layer();
        esp_mesh_get_parent_bssid(&mesh_parent_addr);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_SWITCH_ACK>layer:%d, parent:"MACSTR"", mesh_layer, MAC2STR(mesh_parent_addr.addr));
    }
    break;
    case MESH_EVENT_TODS_STATE: {
        mesh_event_toDS_state_t *toDs_state = (mesh_event_toDS_state_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_TODS_REACHABLE>state:%d", *toDs_state);
    }
    break;
    case MESH_EVENT_ROOT_FIXED: {
        mesh_event_root_fixed_t *root_fixed = (mesh_event_root_fixed_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_FIXED>%s",
                 root_fixed->is_fixed ? "fixed" : "not fixed");
    }
    break;
    case MESH_EVENT_ROOT_ASKED_YIELD: {
        mesh_event_root_conflict_t *root_conflict = (mesh_event_root_conflict_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_ROOT_ASKED_YIELD>"MACSTR", rssi:%d, capacity:%d",
                 MAC2STR(root_conflict->addr),
                 root_conflict->rssi,
                 root_conflict->capacity);
    }
    break;
    case MESH_EVENT_CHANNEL_SWITCH: {
        mesh_event_channel_switch_t *channel_switch = (mesh_event_channel_switch_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHANNEL_SWITCH>new channel:%d", channel_switch->channel);
    }
    break;
    case MESH_EVENT_SCAN_DONE: {
        mesh_event_scan_done_t *scan_done = (mesh_event_scan_done_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_SCAN_DONE>number:%d",
                 scan_done->number);
    }
    break;
    case MESH_EVENT_NETWORK_STATE: {
        mesh_event_network_state_t *network_state = (mesh_event_network_state_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NETWORK_STATE>is_rootless:%d",
                 network_state->is_rootless);
    }
    break;
    case MESH_EVENT_STOP_RECONNECTION: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOP_RECONNECTION>");
    }
    break;
    case MESH_EVENT_FIND_NETWORK: {
        mesh_event_find_network_t *find_network = (mesh_event_find_network_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_FIND_NETWORK>new channel:%d, router BSSID:"MACSTR"",
                 find_network->channel, MAC2STR(find_network->router_bssid));
    }
    break;
    case MESH_EVENT_ROUTER_SWITCH: {
        mesh_event_router_switch_t *router_switch = (mesh_event_router_switch_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROUTER_SWITCH>new router:%s, channel:%d, "MACSTR"",
                 router_switch->ssid, router_switch->channel, MAC2STR(router_switch->bssid));
    }
    break;
    default:
        ESP_LOGI(MESH_TAG, "unknown id:%" PRId32 "", event_id);
        break;
    }
}

void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    ESP_LOGI(MESH_TAG, "<IP_EVENT_STA_GOT_IP>IP:" IPSTR, IP2STR(&event->ip_info.ip));
    s_current_ip.addr = event->ip_info.ip.addr;
#if !CONFIG_MESH_USE_GLOBAL_DNS_IP
    esp_netif_t *netif = event->esp_netif;
    esp_netif_dns_info_t dns;
    ESP_ERROR_CHECK(esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns));
    mesh_netif_start_root_ap(esp_mesh_is_root(), dns.ip.u_addr.ip4.addr); 
#endif

  if (esp_mesh_is_root() && !ntp_synced) {
        ESP_LOGI(MESH_TAG, "Root com IP, iniciando sincronização NTP...");
        if (obtain_time()) {
            ntp_synced = true;
            ESP_LOGI(MESH_TAG, "NTP sincronizado com sucesso.");
            xTaskCreate(sync_time_task, "sync_time_task", 4096, NULL, 10, NULL);
        } else {
            ESP_LOGE(MESH_TAG, "Falha ao sincronizar NTP.");
        }
    }
    esp_mesh_comm_mqtt_task_start();  
}


void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    /*  tcpip initialization */
    ESP_ERROR_CHECK(esp_netif_init());
    /*  event initialization */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /*  crete network interfaces for mesh (only station instance saved for further manipulation, soft AP instance ignored */
    ESP_ERROR_CHECK(mesh_netifs_init(recv_cb));

    /*  wifi initialization */
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    /*  mesh initialization */
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(CONFIG_MESH_MAX_LAYER));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    /* mesh ID */
    memcpy((uint8_t *) &cfg.mesh_id, MESH_ID, 6);
    /* router */
    cfg.channel = CONFIG_MESH_CHANNEL;
    cfg.router.ssid_len = strlen(CONFIG_MESH_ROUTER_SSID);
    memcpy((uint8_t *) &cfg.router.ssid, CONFIG_MESH_ROUTER_SSID, cfg.router.ssid_len);
    memcpy((uint8_t *) &cfg.router.password, CONFIG_MESH_ROUTER_PASSWD,
           strlen(CONFIG_MESH_ROUTER_PASSWD));
    /* mesh softAP */
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE));
    cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
    cfg.mesh_ap.nonmesh_max_connection = CONFIG_MESH_NON_MESH_AP_CONNECTIONS;
    memcpy((uint8_t *) &cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD,
           strlen(CONFIG_MESH_AP_PASSWD));
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));
    /* mesh start */
    ESP_ERROR_CHECK(esp_mesh_start());
    ESP_LOGI(MESH_TAG, "mesh starts successfully, heap:%" PRId32 ", %s\n",  esp_get_free_heap_size(),
             esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed");

}
