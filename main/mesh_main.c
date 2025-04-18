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

#include <sys/time.h>
#include <inttypes.h> // Para lidar com uint64_t em logs

/*******************************************************
 *                Macros
 *******************************************************/
#define EXAMPLE_BUTTON_GPIO     21

// commands for internal mesh communication:
// <CMD> <PAYLOAD>, where CMD is one character, payload is variable dep. on command
#define CMD_DATA 0x55
// CMD_DATA: payload is always 6 bytes identifying address of node sending keypress event
#define CMD_ROUTE_TABLE 0x56
// CMD_ROUTE_TABLE: payload is a multiple of 6 listing addresses in a routing table
/*******************************************************
 *                Constants
 *******************************************************/
static const char *MESH_TAG = "mesh_main";

static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x76};

/*******************************************************
 *                Variable Definitions
 *******************************************************/
static bool is_running = true;
static mesh_addr_t mesh_parent_addr;
static int mesh_layer = -1;
static esp_ip4_addr_t s_current_ip;
static mesh_addr_t s_route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
static int s_route_table_size = 0;
static SemaphoreHandle_t s_route_table_lock = NULL;
static uint8_t s_mesh_tx_payload[CONFIG_MESH_ROUTE_TABLE_SIZE*6+1];

// Variaveis para as metricas de avaliação

int8_t parent_rssi = 0;
uint64_t timestamp_send = 0;
int packets_sent = 0;
int packets_received = 0;
uint64_t timestamp_last_parent_change = 0;
int parent_changes = 0;
int parent_children_count = 0;
int child_count = 0;
int children_count = 0;
uint64_t timestamp_recv;

// Estrutura para armazenar RSSI de cada nó
typedef struct {
    mesh_addr_t addr; // Endereço MAC do nó
    int8_t rssi;      // RSSI do nó
} node_rssi_t;

static node_rssi_t node_rssi_table[CONFIG_MESH_ROUTE_TABLE_SIZE];


/*******************************************************
 *                Function Declarations
 *******************************************************/
// interaction with public mqtt broker
void mqtt_app_start(void);
void mqtt_app_publish(char* topic, char *publish_string);

/*******************************************************
 *                Function Definitions
 *******************************************************/

 // funcao mac
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

// funcao rssi no pai 
void update_rssi() {
    int rssi;
    esp_wifi_sta_get_rssi(&rssi); // Corrigido para usar int
    parent_rssi = (int8_t)rssi;   // Converte para int8_t
    char rssi_str[10];
    snprintf(rssi_str, sizeof(rssi_str), "%d", parent_rssi);
    mqtt_app_publish("/topic/mesh/rssi_pai", rssi_str);
    ESP_LOGI(MESH_TAG, "RSSI do nó pai atualizado: %d dBm", parent_rssi);
}

// RSSI geral
void update_parent_rssi() {
    int rssi;
    esp_wifi_sta_get_rssi(&rssi);
    parent_rssi = (int8_t)rssi;

    char mac_str[18];
    get_mac_str(mac_str, sizeof(mac_str));

    char rssi_str[50];
    snprintf(rssi_str, sizeof(rssi_str), "%s - RSSI: %d", mac_str, parent_rssi);
    mqtt_app_publish("/topic/mesh/rssi", rssi_str);
}

// Calcula a taxa de sucesso de pacotes
// A taxa de sucesso é calculada como (pacotes recebidos / pacotes enviados) * 100
void calculate_success_rate() {
    if (packets_sent == 0) return;

    float success_rate = packets_received > packets_sent ? 100.0 :
                         ((float)packets_received / packets_sent) * 100.0;

    char success_rate_str[10];
    snprintf(success_rate_str, sizeof(success_rate_str), "%.2f", success_rate);
    mqtt_app_publish("/topic/mesh/success_rate", success_rate_str);
    ESP_LOGI(MESH_TAG, "Taxa de sucesso de pacotes: %.2f%%", success_rate);
}

// Calcula a taxa de perda de pacotes
// A taxa de perda é calculada como (pacotes enviados - pacotes recebidos) / pacotes enviados * 100
void calculate_packet_loss() {
    if (packets_sent == 0) return;

    int lost = packets_sent - packets_received;
    float packet_loss = lost < 0 ? 0.0 : ((float)lost / packets_sent) * 100.0;

    char packet_loss_str[20];
    snprintf(packet_loss_str, sizeof(packet_loss_str), "%.2f", packet_loss);
    mqtt_app_publish("/topic/mesh/packet_loss", packet_loss_str);
    ESP_LOGI(MESH_TAG, "Taxa de perda de pacotes: %.2f%%", packet_loss);
}


// Publica número de saltos até o nó raiz
void publish_hops() {
    char mac_str[18];
    get_mac_str(mac_str, sizeof(mac_str));

    char hops_str[50];
    snprintf(hops_str, sizeof(hops_str), "%s - Hops: %d", mac_str, mesh_layer);
    mqtt_app_publish("/topic/mesh/hops", hops_str);
    ESP_LOGI(MESH_TAG, "Número de saltos até o nó raiz: %d", mesh_layer);

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

// Publica número de trocas de nó pai
void publish_parent_changes() {
    char parent_changes_str[10];
    snprintf(parent_changes_str, sizeof(parent_changes_str), "%d", parent_changes);
    mqtt_app_publish("/topic/mesh/parent_changes", parent_changes_str);

    ESP_LOGI(MESH_TAG, "Número de trocas de nó pai: %d", parent_changes);
}

// Conta o número de filhos do nó pai
void update_parent_children_count(int children_count) {
    parent_children_count = children_count;

    char children_str[10];
    snprintf(children_str, sizeof(children_str), "%d", parent_children_count);
    mqtt_app_publish("/topic/mesh/parent_children_count", children_str);

    ESP_LOGI(MESH_TAG, "Número de filhos do nó pai: %d", parent_children_count);
}

// funcao tabela roteamento
void send_routing_table() {
    esp_err_t err;
    char *print;
    mesh_data_t data;
    // Obtém a tabela de roteamento
    esp_mesh_get_routing_table((mesh_addr_t *) &s_route_table,
            CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &s_route_table_size);

    // Publica a tabela de roteamento por MQTT (formato JSON)
    char *route_table_json = malloc(256); // Ajuste o tamanho conforme necessário
    char *ptr = route_table_json;
    ptr += sprintf(ptr, "{\"route_table\":[");

    for (int i = 0; i < s_route_table_size; i++) {
        if (i > 0) {
            ptr += sprintf(ptr, ",");
        }
        ptr += sprintf(ptr, "\"%02x:%02x:%02x:%02x:%02x:%02x\"",
            s_route_table[i].addr[0], s_route_table[i].addr[1],
            s_route_table[i].addr[2], s_route_table[i].addr[3],
            s_route_table[i].addr[4], s_route_table[i].addr[5]);
    }
    sprintf(ptr, "]}");

    ESP_LOGI(MESH_TAG, "Publishing routing table: %s", route_table_json);
    mqtt_app_publish("/topic/route_table", route_table_json);
    free(route_table_json);

    // Prepara os dados para envio via malha
    data.size = s_route_table_size * 6 + 1;
    data.proto = MESH_PROTO_BIN;
    data.tos = MESH_TOS_P2P;
    s_mesh_tx_payload[0] = CMD_ROUTE_TABLE;
    memcpy(s_mesh_tx_payload + 1, s_route_table, s_route_table_size * 6);
    data.data = s_mesh_tx_payload;

    // Envia a tabela de roteamento para cada nó via malha
    for (int i = 0; i < s_route_table_size; i++) {
        err = esp_mesh_send(&s_route_table[i], &data, MESH_DATA_P2P, NULL, 0); //funcao envio
        ESP_LOGI(MESH_TAG, "Sending routing table to [%d] "
                MACSTR ": sent with err code: %d",
        i, MAC2STR(s_route_table[i].addr), err);
    }
}

// funcao processamento recepcao dados
void static recv_cb(mesh_addr_t *from, mesh_data_t *data)
{
    
    if (data->data[0] == CMD_ROUTE_TABLE) {
        int size = data->size - 1;
        if (s_route_table_lock == NULL || size % 6 != 0) {
            ESP_LOGE(MESH_TAG, "Error in receiving route table: Unexpected size");
            return;
        }

        xSemaphoreTake(s_route_table_lock, portMAX_DELAY);
        s_route_table_size = size / 6;
        for (int i = 0; i < s_route_table_size; ++i) {
            ESP_LOGI(MESH_TAG, "Received Routing table [%d] " MACSTR, i, MAC2STR(data->data + 6 * i + 1));
        }
        memcpy(&s_route_table, data->data + 1, size);
        xSemaphoreGive(s_route_table_lock);

    } else if (data->data[0] == CMD_DATA) {
        packets_received++; // Contador de pacotes recebidos
        // Timestamp de início do processamento (alta precisão)
        uint64_t processing_start = esp_timer_get_time();
        packets_received++;
        
        // Extrai dados do payload
        uint8_t* sender_mac = data->data + 1;
        uint64_t received_timestamp_send;
        float temp, hum;
    
        // Copia os dados do payload
        memcpy(&received_timestamp_send, data->data + 7, sizeof(uint64_t));
        memcpy(&temp, data->data + 15, sizeof(float));
        memcpy(&hum, data->data + 19, sizeof(float));
    
        // Formata MAC para impressão
        char mac_str[18];
        snprintf(mac_str, sizeof(mac_str), MACSTR, MAC2STR(sender_mac));
    
        // Timestamp atual (alta precisão)
        uint64_t timestamp_recv = esp_timer_get_time();
        
        // Calcula tempo de processamento até este ponto
        uint64_t processing_time = esp_timer_get_time() - processing_start;
        
        // Cálculo preciso da latência
        uint64_t latency_us = timestamp_recv - received_timestamp_send - processing_time;
        uint64_t latency_ms = latency_us / 1000;
        
        // Verificação de validade (evita valores absurdos)
        if (latency_us > 10000000) { // > 10s = inválido
            ESP_LOGW(MESH_TAG, "Latência inválida detectada: %" PRIu64 " µs - Possível dessincronização de tempo", latency_us);
            return;
        }
        
        // Logs detalhados
        ESP_LOGI(MESH_TAG, "Dados recebidos de %s", mac_str);
        ESP_LOGI(MESH_TAG, "Temperatura: %.2f°C | Umidade: %.2f%%", temp, hum);
        ESP_LOGI(MESH_TAG, "Tempo de processamento: %" PRIu64 " µs", processing_time);
        ESP_LOGI(MESH_TAG, "Latência total: %" PRIu64 " µs (%" PRIu64 " ms)", 
                 latency_us, latency_ms);

    
        // Publica no MQTT (formato simplificado)
        char mqtt_msg[150];
        snprintf(mqtt_msg, sizeof(mqtt_msg), 
                 "{\"mac\":\"%s\",\"temp\":%.2f,\"hum\":%.2f,\"latency_ms\":%" PRIu64 "}", 
                 mac_str, temp, hum, latency_ms);
        
        mqtt_app_publish("/topic/mesh/sensor_data", mqtt_msg);

        // Atualiza estatísticas
        calculate_success_rate();
        calculate_packet_loss();
    } else {
        ESP_LOGE(MESH_TAG, "Comando desconhecido: %02X", data->data[0]);
    }
}



// Leitura sensor a cada 1 min 
static void periodic_send_task(void* args)
{
    while (true) {
        if (s_route_table_size && !esp_mesh_is_root()) { // Se está na tabela de roteamento e não é o nó raiz
            mesh_data_t data;
            uint8_t *my_mac = mesh_netif_get_station_mac();
            float temp = 18.0 + (rand() % 1300) / 100.0;  // 18.0 a 30.99
            float hum = 60.0 + (rand() % 3100) / 100.0;   // 60.0 a 90.99

            // Registra o timestamp de envio ANTES de usá-lo
            timestamp_send = esp_timer_get_time();  


            // Monta o payload com timestamp + sensor + MAC
            uint8_t data_to_send[23] = { CMD_DATA, };
            memcpy(data_to_send + 1, my_mac, 6);                      // MAC (6 bytes)
            memcpy(data_to_send + 7, &timestamp_send, sizeof(uint64_t)); // Timestamp (8 bytes)
            memcpy(data_to_send + 15, &temp, sizeof(float));         // Temp (4 bytes)
            memcpy(data_to_send + 19, &hum, sizeof(float));          // Humidade (4 bytes)

            data.size = sizeof(data_to_send);
            data.proto = MESH_PROTO_BIN;
            data.tos = MESH_TOS_P2P;
            data.data = data_to_send;

            xSemaphoreTake(s_route_table_lock, portMAX_DELAY);
            for (int i = 0; i < s_route_table_size; i++) {
                if (MAC_ADDR_EQUAL(s_route_table[i].addr, my_mac)) {
                    continue;
                }
                packets_sent++; // Incrementa o contador de pacotes enviados
                esp_err_t err = esp_mesh_send(&s_route_table[i], &data, MESH_DATA_P2P, NULL, 0);
                ESP_LOGI(MESH_TAG, "Sent to [%d] " MACSTR " with err: %d", 
                         i, MAC2STR(s_route_table[i].addr), err);
            }
            xSemaphoreGive(s_route_table_lock);
        }
        vTaskDelay(1 * 60 * 1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}



// Chamada a cada 1 min para publicar métricas via MQTT
void esp_mesh_mqtt_task(void *arg) {
    is_running = true;
    char *print;
    mesh_data_t data;
    esp_err_t err;

    // Inicializa o MQTT
    mqtt_app_start();

    while (is_running) {
        // Publica o layer e IP atual
        asprintf(&print, "layer:%d IP:" IPSTR, esp_mesh_get_layer(), IP2STR(&s_current_ip));
        ESP_LOGI(MESH_TAG, "Tried to publish %s", print);
        mqtt_app_publish("/topic/ip_mesh", print);
        free(print);

        // Apenas o root executa as funções abaixo
        if (esp_mesh_is_root()) {    
            update_rssi();             // O root pode medir o RSSI do seu pai
            send_routing_table();      // Somente o root pode enviar a tabela de roteamento  
            publish_parent_changes();  // O root pode monitorar mudanças globais de conexão  
        }
        update_parent_rssi();      // Cada nó pode medir o RSSI do seu pai
        publish_hops();            // Cada nó pode publicar sua própria camada na rede

        // Aguarda antes de repetir
        vTaskDelay(1 * 60 * 1000 / portTICK_PERIOD_MS);
    }
    // Finaliza a tarefa
    vTaskDelete(NULL);
}


esp_err_t esp_mesh_comm_mqtt_task_start(void)
{
    static bool is_comm_mqtt_task_started = false;

    s_route_table_lock = xSemaphoreCreateMutex();

    if (!is_comm_mqtt_task_started) {
        xTaskCreate(esp_mesh_mqtt_task, "mqtt task", 4096, NULL, 5, NULL);
        xTaskCreate(periodic_send_task, "periodic_send_task", 4096, NULL, 5, NULL);
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
        update_parent_children_count(children_count);
    }
    break;
    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *child_disconnected = (mesh_event_child_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_DISCONNECTED>aid:%d, "MACSTR"",
                 child_disconnected->aid,
                 MAC2STR(child_disconnected->mac));
        children_count = esp_mesh_get_total_node_num();
        update_parent_children_count(children_count);
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
