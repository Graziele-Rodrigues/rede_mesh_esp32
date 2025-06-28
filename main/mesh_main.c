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
#include "esp_timer.h" 
#include "esp_sntp.h"

#include <sys/time.h>
#include <inttypes.h> // Para lidar com uint64_t em logs

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
volatile uint32_t packets_sent = 0;
volatile uint32_t packets_received = 0;
uint64_t timestamp_last_parent_change = 0;
int parent_changes = 0;
int parent_children_count = 0;
int child_count = 0;
int children_count = 0;
int64_t time_offset_us = 0;  

/*******************************************************
 *                Function Declarations
 *******************************************************/
// interaction with public mqtt broker
void mqtt_app_start(void);
void mqtt_app_publish(char* topic, char *publish_string);

/*******************************************************
 *                Function Definitions
 *******************************************************/

// Inicializa o SNTP para sincronização de tempo
 void initialize_sntp(void) {
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
    ESP_LOGI(MESH_TAG, "SNTP inicializado");
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
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t timestamp_now = (uint64_t)tv.tv_sec * 1000 + (tv.tv_usec / 1000); // Convertendo para ms
    
    if (timestamp_last_parent_change > 0) {
        uint64_t reconnection_time = timestamp_now - timestamp_last_parent_change;
        char reconnection_str[20];
        snprintf(reconnection_str, sizeof(reconnection_str), "%" PRIu64 " ms", reconnection_time); // Alterado para ms
        mqtt_app_publish("/topic/mesh/reconnection_time", reconnection_str);

        ESP_LOGI(MESH_TAG, "Tempo de reconexão: %" PRIu64 " ms", reconnection_time); // Alterado para ms
    }

    timestamp_last_parent_change = timestamp_now;
    parent_changes++;
}

// Calcula a taxa de sucesso de pacotes (já existe, mantida)
// A taxa de sucesso é calculada como (pacotes recebidos / pacotes enviados) * 100
float get_success_rate() {
    if (packets_sent == 0) {
        ESP_LOGI(MESH_TAG, "Success Rate: 0.00%% (No packets sent)");
        return 0.0;
    }
    float success_rate = ((float)packets_received / packets_sent) * 100.0;
    ESP_LOGI(MESH_TAG, "Success Rate: %.2f%% (Received: %lu, Sent: %lu)", success_rate, packets_received, packets_sent);
    return success_rate;
}

// Calcula a taxa de perda de pacotes (já existe, mantida)
// A taxa de perda é calculada como (pacotes enviados - pacotes recebidos) / pacotes enviados * 100
float get_packet_loss_rate() {
    if (packets_sent == 0) {
        ESP_LOGI(MESH_TAG, "Packet Loss Rate: 0.00%% (No packets sent)");
        return 0.0;
    }
    int lost = packets_sent - packets_received;
    float packet_loss_rate = lost < 0 ? 0.0 : ((float)lost / packets_sent) * 100.0;
    ESP_LOGI(MESH_TAG, "Packet Loss Rate: %.2f%% (Lost: %d, Sent: %lu)", packet_loss_rate, lost, packets_sent);
    return packet_loss_rate;
}


// Calcula o tempo de ida e volta (RTT) entre o envio e recebimento de pacotes
uint32_t calculate_rtt() {
    if (timestamp_send == 0 || timestamp_recv == 0) return 0;
    
    // Converter para milissegundos
    uint64_t send_ms = timestamp_send / 1000;
    uint64_t recv_ms = timestamp_recv / 1000;
    
    return (uint32_t)(recv_ms - send_ms);
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

    packets_received++;

    // Converte MAC para string
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), MACSTR, MAC2STR(packet->mac));

    // Processa com base no ID do pacote
    switch (packet->id) {
        case CMD_SENSOR: {
            if (packet->num_bytes < 16) {
                ESP_LOGW(MESH_TAG, "Payload de sensor muito pequeno");
                return;
            }

            uint64_t timestamp;
            float temp, hum;
            memcpy(&timestamp, packet->payload, sizeof(uint64_t));
            memcpy(&temp, packet->payload + 8, sizeof(float));
            memcpy(&hum, packet->payload + 12, sizeof(float));

            uint64_t now = esp_timer_get_time();
            uint64_t latency_us = now - timestamp;
            uint64_t latency_ms = latency_us / 1000;

            if (esp_mesh_is_root()) {
                char msg[150];
                snprintf(msg, sizeof(msg),
                         "{\"mac\":\"%s\",\"temp\":%.2f,\"hum\":%.2f,\"latency_ms\":%" PRIu64 "}",
                         mac_str, temp, hum, latency_ms);
                mqtt_app_publish("/topic/mesh/sensor", msg);
                ESP_LOGI(MESH_TAG, "MQTT publicado em /topic/mesh/sensor: %s", msg);

            }
            break;
        }

        case CMD_METRICS: {
            if (packet->num_bytes < sizeof(int8_t)) {
                ESP_LOGW(MESH_TAG, "Payload de métrica inválido");
                return;
            }

            int8_t rssi;
            memcpy(&rssi, packet->payload, sizeof(int8_t));


            if (esp_mesh_is_root()) {
                char msg[100];
                snprintf(msg, sizeof(msg),
                         "{\"mac\":\"%s\",\"rssi\":%d}", mac_str, rssi);
                mqtt_app_publish("/topic/mesh/rssi", msg);
                ESP_LOGI(MESH_TAG, "MQTT publicado em /topic/mesh/rssi: %s", msg);
            }
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
                timestamp_recv = esp_timer_get_time();
                metrics.rtt_ms = calculate_rtt();
            }
            break;
        }

        case CMD_SYNC_TIME: {
            if (packet->num_bytes != sizeof(uint64_t)) break;

            uint64_t time_from_root;
            memcpy(&time_from_root, packet->payload, sizeof(uint64_t));

            uint64_t local_time = esp_timer_get_time();
            time_offset_us = (int64_t)time_from_root - (int64_t)local_time;

            ESP_LOGI(MESH_TAG, "Delta de sincronização (us): %" PRId64, time_offset_us);
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

    mqtt_app_publish("/topic/route_table", route_table_json);
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
            float temp =  18.0 + (rand() % 1300) / 100.0;
            float hum  = 60.0 + (rand() % 3100) / 100.0;

            uint64_t local_ts = esp_timer_get_time();
            uint64_t adjusted_ts = local_ts + time_offset_us;  // Ajusta com base no root
            
            uint8_t buffer[20];

            memcpy(buffer, &adjusted_ts, sizeof(adjusted_ts));
            memcpy(buffer + 8, &temp, sizeof(temp));
            memcpy(buffer + 12, &hum, sizeof(hum));

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
                    packets_sent++;
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
    metrics.success_rate = get_success_rate();
    metrics.packet_loss_rate = get_packet_loss_rate();

    // Topologia
    metrics.children_count = children_count;
    metrics.parent_changes = parent_changes;

    // Tempo desde última troca de pai
    metrics.last_parent_change_ms = timestamp_last_parent_change;

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

    packets_sent = 0;
    packets_received = 0;
}

// Envio de metricas
static void metrics_send_task(void *args) {
    while (true) {
        if (!esp_mesh_is_root()) {
            int8_t rssi;
            esp_wifi_sta_get_rssi((int *)&rssi);

            uint8_t buffer[10] = {0};
            memcpy(buffer, &rssi, sizeof(rssi));

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
                    packets_sent++;
                    ESP_LOGI(MESH_TAG, "Métrica enviada ao pai " MACSTR ": RSSI=%d", MAC2STR(parent_addr.addr), rssi);
                } else {
                    ESP_LOGW(MESH_TAG, "Falha ao enviar métrica ao pai: %d", err);
                }
            } else {
                ESP_LOGW(MESH_TAG, "Não foi possível obter endereço do nó pai");
            }
        }

        metricas(); // Atualiza as métricas
        vTaskDelay(5 * 60 * 1000 / portTICK_PERIOD_MS);  // a cada 5 minutos
    }
}

// ping_send_task - para medir RTT
static void ping_send_task(void *args) {
    while (true) {
        if (!esp_mesh_is_root()) {
            timestamp_send = esp_timer_get_time(); // em us

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

        vTaskDelay(4 * 60 * 1000 / portTICK_PERIOD_MS);  // a cada 4 minutos
    }
}

void send_time_to_children() {
    uint64_t current_time = esp_timer_get_time(); // tempo em us

    mesh_packet_t packet;
    build_packet(&packet, CMD_SYNC_TIME, (uint8_t*)&current_time, sizeof(current_time));

    mesh_data_t data = {
        .data = (uint8_t*)&packet,
        .size = sizeof(mesh_packet_t),
        .proto = MESH_PROTO_BIN,
        .tos = MESH_TOS_P2P
    };

    for (int i = 0; i < s_route_table_size; i++) {
        esp_mesh_send(&s_route_table[i], &data, MESH_DATA_P2P, NULL, 0);
    }

    ESP_LOGI(MESH_TAG, "Horário sincronizado enviado para filhos");
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
        xTaskCreate(sensor_send_task, "sensor_send_task", 4096, NULL, 5, NULL);
        xTaskCreate(metrics_send_task, "metrics_send_task", 4096, NULL, 5, NULL);
        xTaskCreate(ping_send_task, "ping_send_task", 4096, NULL, 5, NULL);
        xTaskCreate(sync_time_task, "sync_time_task", 4096, NULL, 5, NULL);
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
