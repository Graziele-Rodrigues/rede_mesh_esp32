# Rede Mesh com MQTT - ESP32

Este código é um exemplo de comunicação interna em uma malha (mesh) utilizando a plataforma **ESP-IDF**. Ele demonstra como criar e gerenciar uma rede mesh, processar eventos de rede, enviar e receber dados entre nós da malha e integrar com o protocolo MQTT para publicação de informações de rede.

## Funcionalidades

- **Comunicação entre nós**: Permite a troca de mensagens entre os nós da malha usando protocolos binários.
- **Detecção de pressionamento de botão**: Quando um botão é pressionado, o nó envia uma mensagem de evento para outros nós na malha, informando que o botão foi acionado.
- **Tabela de Roteamento**: O código envia periodicamente a tabela de roteamento de cada nó para os outros nós e para um servidor MQTT.
- **Integração com MQTT**: Publica informações sobre o estado da rede, como o IP atual, a tabela de roteamento e métricas de desempenho, para um broker MQTT.

## Estrutura do Código

### Principais Componentes

1. **Funções de comunicação**:
   - `update_parent_rssi()`: Atualiza e publica o valor RSSI do nó pai (indicando a qualidade do sinal entre nós).
   - `send_with_timestamp()`: Envia dados com um timestamp, permitindo o cálculo da latência de comunicação.
   - `calculate_latency()`: Calcula e publica a latência entre nós.
   - `calculate_success_rate()`: Calcula a taxa de sucesso de envio de pacotes e publica.
   - `calculate_packet_loss()`: Calcula e publica a taxa de perda de pacotes.
   - `publish_network_load()`: Publica a carga de rede, incluindo os bytes enviados e recebidos.
   - `publish_hops()`: Publica o número de saltos (hops) na rede mesh.
   - `send_routing_table()`: Envia a tabela de roteamento para os outros nós da rede mesh.

2. **Função de Botão**:
   - `check_button()`: Monitora um botão configurado e, ao ser pressionado, envia uma mensagem via MQTT e propaga o evento para outros nós na rede.

3. **MQTT**:
   - A tarefa `esp_mesh_mqtt_task()` publica periodicamente informações de rede (como IP, tabela de roteamento e métricas) para um broker MQTT.

### Comandos de Comunicação

- **CMD_KEYPRESSED (0x55)**: Envia um evento indicando que um botão foi pressionado. O payload contém o endereço MAC do nó que enviou o evento.
- **CMD_ROUTE_TABLE (0x56)**: Envia a tabela de roteamento da malha. O payload é uma lista de endereços MAC de todos os nós na tabela.

## Como Usar

### 1. Configuração do ambiente

Certifique-se de ter o **ESP-IDF** instalado em seu sistema. Para isso, siga as instruções na [documentação oficial](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html).

### 2. Compilação e Flash

- Compile o código com o comando:
  ```bash
  idf.py build
  ```
- Faça o upload do firmware para o dispositivo com:
  ```bash
  idf.py -p PORT flash
  ```

### 3. Monitoramento

Use o comando a seguir para monitorar a saída do dispositivo:

```bash
idf.py -p PORT monitor
```

Isso permitirá que você visualize as mensagens enviadas e recebidas, como a tabela de roteamento e a detecção de eventos de pressionamento de botão.

### 4. Teste do Botão

Ao pressionar o botão configurado no dispositivo, um evento será enviado para todos os nós da malha. A tabela de roteamento será publicada periodicamente pelo nó raiz, e as métricas de rede, como latência e taxa de sucesso, serão enviadas para o broker MQTT.

## Exemplo de Uso

Quando o botão configurado é pressionado, um evento será enviado para todos os nós da malha. As informações da rede, como o IP e a tabela de roteamento, serão periodicamente publicadas no broker MQTT. O código também calcula e publica métricas importantes da rede, como a latência de comunicação, taxa de sucesso de pacotes e carga de rede.

## Considerações

- Certifique-se de configurar corretamente o MQTT para que os tópicos possam ser acessados e visualizados corretamente.
- A comunicação entre os nós é feita de forma transparente utilizando a rede Mesh, sem a necessidade de configurações adicionais.
- A publicação de dados de rede é feita em intervalos regulares (5 segundos), mas pode ser ajustada conforme necessário.
