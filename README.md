## Descrição

Este código é um exemplo de comunicação interna em uma malha (mesh) utilizando a plataforma ESP-IDF. Ele demonstra como criar e gerenciar uma rede mesh, processar eventos de rede, enviar e receber dados entre nós da malha e integrar com o protocolo MQTT para publicação de informações de rede.

### Funcionalidades

- **Comunicação entre nós**: Permite a troca de mensagens entre os nós da malha usando protocolos binários.
- **Detecção de pressionamento de botão**: Quando um botão é pressionado, o nó envia uma mensagem de evento para outros nós na malha, informando que o botão foi acionado.
- **Tabela de roteamento**: O código envia periodicamente a tabela de roteamento de cada nó para os outros nós e para um servidor MQTT.
- **Integração com MQTT**: Publica informações sobre o estado da rede, como o IP atual e a tabela de roteamento, para um broker MQTT.

## Estrutura do Código

### Principais componentes

- **Inicialização do botão**: Configura um botão como entrada e detecta sua ação.
- **Recepção de dados**: Processa mensagens recebidas da malha e identifica comandos específicos, como a tabela de roteamento e eventos de pressionamento de botão.
- **Envio de dados**: Envia mensagens de eventos de pressionamento de botão para outros nós na malha.
- **Tarefa de MQTT**: Publica periodicamente informações da malha, como o IP atual e a tabela de roteamento, para um broker MQTT.
- **Tratamento de eventos de malha**: Garante o correto funcionamento e a resposta a diferentes eventos de rede, como conexão e desconexão de nós.

### Comandos de Comunicação

- **CMD_KEYPRESSED (0x55)**: Envia um evento indicando que um botão foi pressionado. O payload contém o endereço MAC do nó que enviou o evento.
- **CMD_ROUTE_TABLE (0x56)**: Envia a tabela de roteamento da malha. O payload é uma lista de endereços MAC de todos os nós na tabela.

## Como Usar

1. **Configuração do ambiente**:
   - Certifique-se de ter o [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) instalado em seu sistema.

2. **Compilação e flash**:
   - Compile o código com o comando:
     ```bash
     idf.py build
     ```
   - Faça o upload do firmware para o dispositivo com:
     ```bash
     idf.py -p PORT flash
     ```

3. **Monitoramento**:
   - Use o comando a seguir para monitorar a saída do dispositivo:
     ```bash
     idf.py -p PORT monitor
     ```



## Exemplo de Uso

Quando o botão configurado é pressionado, um evento é enviado para todos os nós da malha. A tabela de roteamento é publicada periodicamente pelo nó central, e as informações da rede são enviadas para o broker MQTT.

