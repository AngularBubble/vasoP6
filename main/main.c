//________________________________________BIBLIOTECAS________________________________________

#include <stdio.h>      //Biblioteca padrão para entrada e saída de dados e formatação de strings
#include <stdint.h>     //Biblioteca padrão para a locação de memória
#include <stddef.h>     //Biblioteca padrão para uso de ponteiros
#include <string.h>     //Biblioteca padrão para de manipulação de strings
#include <math.h>       //Biblioteca padrão para calculos matemáticos complexos
#include <time.h>       //Biblioteca padrão para manipulação de valores de tempo e data
#include <sys/time.h>       //Biblioteca de funções para manipulação de valores de tempo especializada em valores em alta resolução
#include "freertos/FreeRTOS.h"      //Biblioteca para uso de Real Time Operating System(RTOS)
#include "freertos/task.h"      //Biblioteca para gerenciamento de tarefas RTOS
#include "freertos/event_groups.h"      //Biblioteca para grupos de eventos e sincronização de tarefas
#include "esp_system.h"     //Biblioteca do framework do ESP IDF com ferramentas de monitoramento de sistemas
#include "esp_wifi.h"       //Biblioteca de wifi para ESP
#include "nvs_flash.h"      //Biblioteca para uso de Non Volatile Memory(NVS)
#include "esp_event.h"      //Biblioteca para loop de eventos
#include "esp_netif.h"      //Biblioteca que faz a abstração entre os protocolos de rede e o hardware
#include "esp_log.h"        //Biblioteca de logs
#include "esp_attr.h"       //Biblioteca para controle de alocação de variáveis em determinados lugares de memória
#include "esp_sleep.h"      //Biblioteca com as funções de gerenmciamento de energia e modos de dormir
#include "mqtt_client.h"       //Biblioteca para cliente MQTT
#include "esp_netif_sntp.h"     //Biblioteca que faz integração entre a biblioteca de SNTP e de Netif
#include "esp_sntp.h"       //Biblioteca que habilita o Simple Network Time Protocol(SNTP)
#include "soc/adc_channel.h"    //Biblioteca geral para os canais adc
#include "esp_adc/adc_oneshot.h"    //Biblioteca para a funcionalidade de leitura única (oneshot) do Analog to Digital Converter(ADC)
#include "esp_adc/adc_cali.h"   //Biblioteca para a interface de calibração do ADC
#include "esp_adc/adc_cali_scheme.h"    //Biblioteca que toma conta da curva de calibração do ADC
#include "driver/ledc.h"    //Biblioteca que é responsável pelas funções do LEDC
#include "esp_private/esp_clk.h"    //Biblioteca privada para controle do clock interno
#include "driver/mcpwm_cap.h"   //Biblioteca para o MCPWM
#include "driver/gpio.h"    //Biblioteca de drivers para configurar os GPIO
#include "esp_littlefs.h"   //Biblioteca para uso do sistema de arquivos littlefs

#include "lwip/err.h"       //Biblioteca para gerenciamento de erros de rede
#include "lwip/sys.h"       //Biblioteca para uso de sistemas de rede com RTOS
#include "lwip/ip_addr.h"       //Biblioteca para manipulação de endereços IP

//_____________________________DEFINIÇÕES_PARA_CONFIGURAÇÃO_WIFI_____________________________

#define EXAMPLE_ESP_WIFI_SSID      "GABRIEL"
#define EXAMPLE_ESP_WIFI_PASS      "Master1357@"
#define EXAMPLE_ESP_MAXIMUM_RETRY  100

#if CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

//_____________________________DEFINIÇÕES_PARA_CONFIGURAÇÃO_MQTT_____________________________

#define EXAMPLE_CONFIG_BROKER_URL "mqtts://192.168.1.6"

//_________________________DEFINIÇÕES_PARA_CONFIGURAÇÃO_DE_STACK_SIZE________________________
#define STACK_SIZE_SENSOR_VALUES 4096
#define STACK_SIZE_DECISION 4096
#define STACK_SIZE_PROCESSOS_MQTT 4096
#define STACK_SIZE_DALLY_RESET 4096


//________________________DEFINIÇÕES_PARA_CONFIGURAÇÃO_DE_IP_PARA_SNTP_______________________

#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 48
#endif

//_______________________DEFINIÇÕES_PARA_CONFIGURAÇÃO_DE_PINOS_PARA_ADC______________________

//Canais para os LDRs
#define LDR_N ADC1_GPIO35_CHANNEL //GPIO_35
#define LDR_O ADC1_GPIO34_CHANNEL //GPIO_34
#define LDR_L ADC1_GPIO32_CHANNEL //GPIO_32
#define LDR_S ADC1_GPIO33_CHANNEL //GPIO_33

//Canais para os Thermistores
#define THERMISTOR_N ADC2_GPIO26_CHANNEL  //GPIO_26
#define THERMISTOR_O ADC2_GPIO25_CHANNEL  //GPIO_25
#define THERMISTOR_L ADC2_GPIO27_CHANNEL  //GPIO_27
#define THERMISTOR_S ADC2_GPIO14_CHANNEL  //GPIO_14

//Canal para o Higrometro
#define SOIL_HYGROMETER ADC2_GPIO13_CHANNEL //GPIO_13

//____________________________DEFINIÇÃO_PARA_CONFIGURAÇÕES_DO_LEDC___________________________

#define LEDC_BOMBA_AGUA 22 //GPIO22
#define LEDC_TIMER LEDC_TIMER_0 //Timer do LEDC
#define LEDC_CHANNEL LEDC_CHANNEL_0 //Canal LEDC
#define LEDC_MODE LEDC_LOW_SPEED_MODE //Modo do LEDC
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT //Resolução do LEDC
#define LEDC_DUTY_ON 7372.8 // Define o duty para ligado para 50%. (2 ** 13) * 50% = 4096]
#define LEDC_DUTY_OFF 0 // Define o duty para desligado para 0%

//______________________DEFINIÇÃO_PARA_CONFIGURAÇÃO_DOS_PINOS_DE_MCPWM_______________________

#define AJ_SR04M_TRIG_GPIO  19
#define AJ_SR04M_ECHO_GPIO  21

//___________________________DEFINIÇÕES_PARA_CONFIGURAÇÃO_DAS_FILAS__________________________

//Tamanho da mensagem nas filas
#define messageSize 250
#define queueSize 50

//_____________DEFINIÇÕES_PARA_VALORES_DE_CONSTANTES_PARA_OS_CALCULOS_DE_SENSORES____________

//Valores para os calculos do LDR
#define ldrGamma 0.7  //Constante usada para a conversão R -> L
#define tensaoAlimentacao 3300.0  //3300mV
#define resistorCircuitoLDR 2200.0  //2kOhm
#define resistenciaParaValorLux 14000.0 //14kOhm
#define resistenciaParaValorLuxN 29938.312 //29.9kOhm
#define resistenciaParaValorLuxO 33257.146 //33.2kOhm
#define resistenciaParaValorLuxL 23443.201 //23.4kOhm
#define resistenciaParaValorLuxS 31618.039 //31.6kOhm
#define lux 10  //10Lux

//Valores para os calculos do Thermistor
#define thrB 3950  //Constante usada para a conversão R -> L
#define resistorCircuitoTHR 10000.0 //10kOhm
#define resistenciaParaValorKelvin 10000.0  //10kOhm
#define kelvin 298.15 //298.15°K
#define kelvinCelsius 273.15  //273.15°K

//Valores para os calculos do Higrometro
#define resistorCircuitoSHR 10000.0 //10kOhm
#define maxResistenceSHR 10000000.0 //10MOhm
#define minResistenceSHR 0.0 //0Ohm

//_____________________DEFINIÇÕES_PARA_VALORES_DE_CONSTANTES_PARA_O_TEMPO____________________

//Para atributos que aceitam TickType_t
#define t250ms 250/portTICK_PERIOD_MS
#define t500ms 500/portTICK_PERIOD_MS
#define t1s 1000/portTICK_PERIOD_MS
#define t2s 2000/portTICK_PERIOD_MS
#define t5s 5000/portTICK_PERIOD_MS
#define t10s 10000/portTICK_PERIOD_MS
#define t15s 15000/portTICK_PERIOD_MS
#define t30s 30000/portTICK_PERIOD_MS
#define t45s 45000/portTICK_PERIOD_MS
#define t60s 60000/portTICK_PERIOD_MS
#define t90s 90000/portTICK_PERIOD_MS
#define t300s 300000/portTICK_PERIOD_MS

//Para atributos que aceitam Minutos
#define t1m 1 //1min
#define t10m 10 //1min
#define t30m 30 //30min
#define t60m 60 //1hora/60min

//Para atributos que aceitam Segundos
#define t3600s 3600 //1h/60min/3600seg

//Tempos do dia para ativação da bomba default
#define timeHour1 7 //7
#define timeMin1 30 //30

#define timeHour2 12    //12
#define timeMin2 0      //0

#define timeHour3 18    //18
#define timeMin3 30   //30

#define timeHourNotConf 24 
#define timeMinNotConf 60

//______________________________DEFINIÇÕES_PARA_TÓPICOS_DE_MQTT______________________________

#define topicoLDR "VASO_0/LDR"
#define topicoTHR "VASO_0/THERMISTOR"
#define topicoSHR "VASO_0/SOIL_HYGROMETER"
#define topicoUTS "VASO_0/ULTRASOUND"
#define topicoWPM "VASO_0/WATER_PUMP"
#define topicoSYS "VASO_0/SYSTEM"
#define topicoCON "VASO_0/CONF"

//_____________________DEFINIÇÕES_UTILIZADAS_PARA_O_FUNCIONAMENTO_DO_VASO____________________

#define temperaturaAdequadaPadrão 35
#define umidadeAdequadaPadrão 70
#define quantidadeAtivaçõesPadrão 3
#define quantidadeAtivaçõesMaxima 10
#define alturaMinimaPadrão 32.5

//__________________DEFINIÇÕES_UTILIZADAS_PARA_O_USO_DE_SISTEMA_DE_ARTQUIVOS_________________

#define maxLines 400
#define maxMqttMaxTryout 3
#define dataFileName "/files/data.txt"
#define configFileName "/files/config.txt"

//_____________________________________VARIÁVEIS_GLOBAIS_____________________________________

static const char *sensorValues = "sensorValues";  //Tag para os Logs referentes a tarefa sensorValues
static const char *decision = "decision";  //Tag para os Logs referentes a tarefa sensorValues
static const char *processosMqtt = "processosMqtt";  //Tag para os Logs referentes a tarefa sensorValues
static const char *dallyReset = "dallyReset";   //Tag para os logs referentes a tarefa dallyReset
static const char *saveDataFile = "saveDataFile";   //Tag para os logs referentes para a tarefa saveDataFile
static const char *loadingDefaultValues = "loadingDefaultValues"; //Tag para os logs referentes ao carregamento de valores de configuração
static const char *saveConfFile = "saveConfFile"; //Tag para os logs referentes a função que salva a configuração atual
static const char *sleepSetup = "sleepSetup";   //Tag para os logs referentes a função de dormir
static const char *LittleFS = "LittleFS";  //Tag para os Logs referentes ao LittleFS
static const char *MQTT5 = "MQTT5";     //Tag para os Logs referentes ao MQTT
static const char *MCPWM = "MCPWM";  //Tag para os Logs referentes a MCPWM
static const char *WIFI = "WIFI";    //Tag para os Logs referentes ao Wifi  
static const char *SNTP = "SNTP";  //Tag para os Logs referentes a SNTP
static const char *ADC = "ADC";  //Tag para os Logs referentes a ADC



static TaskHandle_t xHandleSensorValues;
static TaskHandle_t xHandleDecision;
static TaskHandle_t xHandleProcessosMqtt;
static TaskHandle_t xHandleDallyReset;
static TaskHandle_t xHandleSaveSystemData;
static TaskHandle_t xHandleSleepSetup;

static EventGroupHandle_t s_wifi_event_group;   //Gerenciador de eventos de Wifi

static int s_retry_num = 0;     //Numero de tentativas para reconectar ao Wifi

static adc_oneshot_unit_handle_t adc1_handle;   //Gerenciador de eventos do ADC1
static adc_oneshot_unit_handle_t adc2_handle;   //Gerenciador de eventos do ADC2

static adc_cali_handle_t adc1_cali_handle;  //Gerenciador de calibração do ADC1
static adc_cali_handle_t adc2_cali_handle;  //Gerenciador de calibração do ADC1

QueueHandle_t sensor_LDR_queue_handle;  //Fila de mensagens do LDR
QueueHandle_t sensor_THR_queue_handle;  //Fila de mensagens do THR
QueueHandle_t sensor_SHR_queue_handle;  //Fila de mensagens do SHR
QueueHandle_t sensor_UTS_queue_handle;  //Fila de mensagens do SHR
QueueHandle_t funcionamento_WPM_queue_handle;  //Fila de mensagens da WaterPump Motor

SemaphoreHandle_t sensorValuesSemaphore; //Semaforo para proteger o valor final dos sensores
SemaphoreHandle_t dallyControlValuesSemaphore;  //Semaforo para projeter a manipulação das variáveis de controle diario
SemaphoreHandle_t mqttAvailable;  //Semaforo que indica quando o MQTT está disponível
SemaphoreHandle_t writeDataFile;  //Semaforo que indica quando é necessário escrever no arquivo de dados
SemaphoreHandle_t dataFileSemaphore; //Semaforo que protege a escrita e leitura do arquivo de dados salvos
SemaphoreHandle_t configFileSemaphore; //Semaforo que protege a escrita e leitura de arquivo de configuração

esp_mqtt_client_handle_t client; //Gerenciador do cliente MQTT

static float ldr_lux[4];  //LDR - Vetor do valor de lux
static float thr_tem[4];  //THERMISTOR - Vetor do valor de temperatura
static float shr_pct; //SOIL HYGROMETER - Valor da humidade
static float uts_dis; //ULTRASOOUND - Valor de distancia

static float tempAdeq;  //Variável que salva a temperatura adequada configurada
static float umidAdeq;  //Variável que salva a umidade adequada configurada
static float altuMin;   //Variável que salva a altura minima do reservatório

static short int qtdAtivacao;
static int tempoAtivacaoProvisoria; //Armazena o tempo minimo que a bomba precisa ser ativada em caso de urgencia
static int tempoAtivaçãoTotal;      //Armazena o tempo total diário de ativação da bomba
static int tempoAtivaçãoPorPeriodo; //Armazena o tempo que a bomba deve ser ativada em cada período de tempo
static int contaAtivacaoRestante;  //Conta quantas ativações restantes da bomba no dia
static int tempoAtivacaoQuePassou; //Conta quantas vezes passou com a bomba ligada
static bool timestampMarker[quantidadeAtivaçõesMaxima]; //Marca quais tempos do dia ele já passaram
struct tm ultimaAtivacao;   //Salva o valor da última ativação do motor
static int nextDay;    //Variável que armazena o próximo dia

//Variáveis que armazenam as horas fixas de ativação da bomba
static int horaAtivacao[quantidadeAtivaçõesMaxima];  
static int minAtivacao[quantidadeAtivaçõesMaxima];

//Estado da bomba de água
typedef enum{
    BOMBA_DESLIGADA,
    BOMBA_LIGADA
} water_pump_state;

water_pump_state bombaDagua;
//_____________________________________PROPRIEDADES_MQTT_____________________________________

//_______PROPRIEDADES_DO_USUÁRIO_______
static esp_mqtt5_user_property_item_t user_property_arr[] = {
        {"board", "esp32"},
        {"u", "user"},
        {"p", "password"}
    };

//_______TAMANHO_DO_ARRAY_DE_PROPRIEDADES_DO_USUÁRIO_______
#define USE_PROPERTY_ARR_SIZE   sizeof(user_property_arr)/sizeof(esp_mqtt5_user_property_item_t)

//_______PROPRIEDADES_DE_PUBLICAÇÃO_______
static esp_mqtt5_publish_property_config_t publish_property = {
    .payload_format_indicator = 1,
    .message_expiry_interval = 1000,
    .topic_alias = 0,
    .response_topic = "/topic/test/response",
    .correlation_data = "123456",
    .correlation_data_len = 6,
};

//_______PROPRIEDADES_DE_INSCRIÇÃO_______
static esp_mqtt5_subscribe_property_config_t subscribe_property = {
    .subscribe_id = 25555,
    .no_local_flag = false,
    .retain_as_published_flag = false,
    .retain_handle = 0,
    .is_share_subscribe = false,
};

//_______PROPRIEDADES_DE_DESINSCRIÇÃO________
static esp_mqtt5_unsubscribe_property_config_t unsubscribe_property = {
    .is_share_subscribe = true,
    .share_name = "group1",
};

//_______PROPRIEDADES_DE_DESCONEXÃO________
static esp_mqtt5_disconnect_property_config_t disconnect_property = {
    .session_expiry_interval = 60,
    .disconnect_reason = 0,
};

//__________________________________________FUNÇÕES__________________________________________

//_______FUNÇÃO_QUE_IMPRIME_ERROS_QUANDO_OCORRER_ALGUM_ERRO_NA_PARTE_DE_MQTT_______
static void log_error_if_nonzero(const char *message, int error_code){

    if (error_code != 0) {
        ESP_LOGE(MQTT5, "Last error %s: 0x%x", message, error_code);
    }
}

//_______FUNÇÃO_QUE_NOTIFICA_QUANDO_HOUVER_UM_EVENTO_DE_SINCRONIZAÇÃO_DE_TEMPO_______
void time_sync_notification_cb(struct timeval *tv){

    ESP_LOGI(SNTP, "Notification of a time synchronization event");
}

//_______FUNÇÃO_QUE_CALCULA_DIFERENÇA_DE_TEMPO_EM_MINUTOS_______
int tmDifferenceInMinutes(struct tm timeinfo1, struct tm timeinfo2){
    int sumTimeinfo1;
    int sumTimeinfo2;
    int result;

    sumTimeinfo1 = timeinfo1.tm_hour*60 + timeinfo1.tm_min;
    sumTimeinfo2 = timeinfo2.tm_hour*60 + timeinfo2.tm_min;

    result = abs(sumTimeinfo1 - sumTimeinfo2);

    return result;
}

//_______FUNÇÃO_QUE_CALCULA_DIFERENÇA_DE_TEMPO_EM_SEGUNDOS_______
int tmDifferenceInSeconds(struct tm timeinfo1, struct tm timeinfo2){
    int sumTimeinfo1;
    int sumTimeinfo2;
    int result;

    sumTimeinfo1 = timeinfo1.tm_hour*3600 + timeinfo1.tm_min*60 + timeinfo1.tm_sec;
    sumTimeinfo2 = timeinfo2.tm_hour*3600 + timeinfo2.tm_min*60 + timeinfo1.tm_sec;

    result = abs(sumTimeinfo1 - sumTimeinfo2);

    return result;
}

//_______FUNÇÃO_QUE_IMPRIME_A_LISTA_DE_SERVIDORES_NTP_CONFIGURADOS_______
static void print_servers(void){

    ESP_LOGI(SNTP, "List of configured NTP servers:");

    for (uint8_t i = 0; i < SNTP_MAX_SERVERS; ++i){
        if (esp_sntp_getservername(i)){
            ESP_LOGI(SNTP, "server %d: %s", i, esp_sntp_getservername(i));
        } else {
            // we have either IPv4 or IPv6 address, let's print it
            char buff[INET6_ADDRSTRLEN];
            ip_addr_t const *ip = esp_sntp_getserver(i);
            if (ipaddr_ntoa_r(ip, buff, INET6_ADDRSTRLEN) != NULL)
                ESP_LOGI(SNTP, "server %d: %s", i, buff);
        }
    }
}

//_______FUNÇÃO_QUE_IMPRIME_AS_PROPRIEDADES_DO_USUÁRIO_MQTT_______
static void print_user_property(mqtt5_user_property_handle_t user_property){

    if (user_property) {
        uint8_t count = esp_mqtt5_client_get_user_property_count(user_property);
        if (count) {
            esp_mqtt5_user_property_item_t *item = malloc(count * sizeof(esp_mqtt5_user_property_item_t));
            if (esp_mqtt5_client_get_user_property(user_property, item, &count) == ESP_OK) {
                for (int i = 0; i < count; i ++) {
                    esp_mqtt5_user_property_item_t *t = &item[i];
                    ESP_LOGI(MQTT5, "key is %s, value is %s", t->key, t->value);
                    free((char *)t->key);
                    free((char *)t->value);
                }
            }
            free(item);
        }
    }
}
//_______FUNÇÃO_AUXILIAR_QUE_DA_TRIGGER_NO_SENSOR_______
static void gen_trig_output(void)
{
    gpio_set_level(AJ_SR04M_TRIG_GPIO, 1); // set high
    esp_rom_delay_us(10);
    gpio_set_level(AJ_SR04M_TRIG_GPIO, 0); // set low
}

//_______FUNÇÃO_DE_CALLBACK_PARA_O_MCPWM_______
static bool aj_sr04m_echo_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    static uint32_t cap_val_begin_of_sample = 0;
    static uint32_t cap_val_end_of_sample = 0;
    TaskHandle_t task_to_notify = (TaskHandle_t)user_data;
    BaseType_t high_task_wakeup = pdFALSE;
    
    //calculate the interval in the ISR,
    //so that the interval will be always correct even when capture_queue is not handled in time and overflow.
    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        // store the timestamp when pos edge is detected
        cap_val_begin_of_sample = edata->cap_value;
        cap_val_end_of_sample = cap_val_begin_of_sample;
    } else {
        cap_val_end_of_sample = edata->cap_value;
        uint32_t tof_ticks = cap_val_end_of_sample - cap_val_begin_of_sample;

        // notify the task to calculate the distance
        xTaskNotifyFromISR(task_to_notify, tof_ticks, eSetValueWithOverwrite, &high_task_wakeup);
    }

    return high_task_wakeup == pdTRUE;
}

//FUNÇÃO QUE SALVA OS VALORES NO ARQUIVO
static void vSaveConfFile(void){
    ESP_LOGI(saveConfFile,"Updating config file");

    FILE *fConfig;
    if(xSemaphoreTake(configFileSemaphore,t1s)){
        fConfig = fopen(configFileName,"w");
        if(fConfig != NULL){
            fprintf(fConfig, "TA %f\n",tempAdeq);
            fprintf(fConfig, "UA %f\n",umidAdeq);
            fprintf(fConfig, "AM %f\n",altuMin);
            fprintf(fConfig, "TAP %d\n",tempoAtivacaoProvisoria);
            fprintf(fConfig, "TAT %d\n",tempoAtivaçãoTotal);
            fprintf(fConfig, "TAQP %d\n",tempoAtivacaoQuePassou);
            fprintf(fConfig, "CAR %d\n",contaAtivacaoRestante);
            fprintf(fConfig, "UATSEC %d\n",ultimaAtivacao.tm_sec);
            fprintf(fConfig, "UATMIN %d\n",ultimaAtivacao.tm_min);
            fprintf(fConfig, "UATHOUR %d\n",ultimaAtivacao.tm_hour);
            fprintf(fConfig, "UATMDAY %d\n",ultimaAtivacao.tm_mday);
            fprintf(fConfig, "UATMON %d\n",ultimaAtivacao.tm_mon);
            fprintf(fConfig, "UATYEAR %d\n",ultimaAtivacao.tm_year);
            fprintf(fConfig, "UATWDAY %d\n",ultimaAtivacao.tm_wday);
            fprintf(fConfig, "UATYDAY %d\n",ultimaAtivacao.tm_yday);
            fprintf(fConfig, "UATISDST %d\n",ultimaAtivacao.tm_isdst);
            fprintf(fConfig, "H0 %d\n",horaAtivacao[0]);
            fprintf(fConfig, "M0 %d\n",minAtivacao[0]);
            fprintf(fConfig, "H1 %d\n",horaAtivacao[1]);
            fprintf(fConfig, "M1 %d\n",minAtivacao[1]);
            fprintf(fConfig, "H2 %d\n",horaAtivacao[2]);
            fprintf(fConfig, "M2 %d\n",minAtivacao[2]);
            fprintf(fConfig, "H3 %d\n",horaAtivacao[3]);
            fprintf(fConfig, "M3 %d\n",minAtivacao[3]);
            fprintf(fConfig, "H4 %d\n",horaAtivacao[4]);
            fprintf(fConfig, "M4 %d\n",minAtivacao[4]);
            fprintf(fConfig, "H5 %d\n",horaAtivacao[5]);
            fprintf(fConfig, "M5 %d\n",minAtivacao[5]);
            fprintf(fConfig, "H6 %d\n",horaAtivacao[6]);
            fprintf(fConfig, "M6 %d\n",minAtivacao[6]);
            fprintf(fConfig, "H7 %d\n",horaAtivacao[7]);
            fprintf(fConfig, "M7 %d\n",minAtivacao[7]);
            fprintf(fConfig, "H8 %d\n",horaAtivacao[8]);
            fprintf(fConfig, "M8 %d\n",minAtivacao[8]);
            fprintf(fConfig, "H9 %d\n",horaAtivacao[9]);
            fprintf(fConfig, "M9 %d\n",minAtivacao[9]);
            fprintf(fConfig, "TSM1 %d\n",timestampMarker[0]);
            fprintf(fConfig, "TSM2 %d\n",timestampMarker[1]);
            fprintf(fConfig, "TSM3 %d\n",timestampMarker[2]);
            fprintf(fConfig, "TSM4 %d\n",timestampMarker[3]);
            fprintf(fConfig, "TSM5 %d\n",timestampMarker[4]);
            fprintf(fConfig, "TSM6 %d\n",timestampMarker[5]);
            fprintf(fConfig, "TSM7 %d\n",timestampMarker[6]);
            fprintf(fConfig, "TSM8 %d\n",timestampMarker[7]);
            fprintf(fConfig, "TSM9 %d\n",timestampMarker[8]);
            fprintf(fConfig, "TSM10 %d\n",timestampMarker[9]);
            fprintf(fConfig, "ND %d\n",nextDay);
            
            fclose(fConfig);
            ESP_LOGI(saveConfFile,"File saved successfully!");
        }else{
            ESP_LOGI(saveConfFile,"Could not write config file");
        }
        xSemaphoreGive(configFileSemaphore);  
    }
    
    
}

//_______GERENCIADOR_DE_EVENTOS_MQTT_______
static void mqtt5_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){

    //Log do MQTT para debug
    ESP_LOGD(MQTT5, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    
    esp_mqtt_event_handle_t event = event_data;     //Construtor do handler de eventos mqtt
    esp_mqtt_client_handle_t client = event->client;        //Construtor do handler do cliente mqtt

    int msg_id; //Id de mensagem para os logs

    //Variables to treat config messages
    char varTag[10];
    char varValue[10];
    char *endPtr;
    float tempFloatVar;
    int tempIntVar;
    int tempQtdAtivacao = 0;
    //Log do MQTT para debug
    ESP_LOGD(MQTT5, "free heap size is %" PRIu32 ", minimum %" PRIu32, esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
    
    switch ((esp_mqtt_event_id_t)event_id) {

    //Evento ao CONECTAR no Broker
    case MQTT_EVENT_CONNECTED:  
        //Notifica que se conectou
        ESP_LOGI(MQTT5, "MQTT_EVENT_CONNECTED");
        print_user_property(event->property->user_property);
        //Indica através do semaforo que está disponível
        xSemaphoreGive(mqttAvailable);
        
        //O seguinte código representa um exemplo de publicação de mensagem MQTT
        /*
        esp_mqtt5_client_set_user_property(&publish_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
        esp_mqtt5_client_set_publish_property(client, &publish_property);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 1);
        esp_mqtt5_client_delete_user_property(publish_property.user_property);
        publish_property.user_property = NULL;
        ESP_LOGI(MQTT5, "sent publish successful, msg_id=%d", msg_id);
        */

        //O seguinte código representa um exemplo de inscrição em tópico MQTT
        
        esp_mqtt5_client_set_user_property(&subscribe_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
        esp_mqtt5_client_set_subscribe_property(client, &subscribe_property);
        msg_id = esp_mqtt_client_subscribe(client, topicoCON, 2);
        esp_mqtt5_client_delete_user_property(subscribe_property.user_property);
        subscribe_property.user_property = NULL;
        ESP_LOGI(MQTT5, "sent subscribe successful, msg_id=%d", msg_id);
        

        //O Seguinte código representa um exemplo de desinscrição em tópico MQTT
        /*
        esp_mqtt5_client_set_user_property(&unsubscribe_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
        esp_mqtt5_client_set_unsubscribe_property(client, &unsubscribe_property);
        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos0");
        ESP_LOGI(MQTT5, "sent unsubscribe successful, msg_id=%d", msg_id);
        esp_mqtt5_client_delete_user_property(unsubscribe_property.user_property);
        unsubscribe_property.user_property = NULL;
        */
        break;

    //Evento ao DESCONECTAR do Broker
    case MQTT_EVENT_DISCONNECTED:   
        //Notifica que foi desconectado
        ESP_LOGI(MQTT5, "MQTT_EVENT_DISCONNECTED");
        
        if(xSemaphoreTake(mqttAvailable, t1s) == pdTRUE){
            ESP_LOGI(MQTT5, "Disconnection notified successfully");
        }

        print_user_property(event->property->user_property);
        break;
    
    //Evento ao INSCREVER
    case MQTT_EVENT_SUBSCRIBED:     
        ESP_LOGI(MQTT5, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        
        //O seguinte código representa um exemplo de publicação de mensagem MQTT
        /*
        print_user_property(event->property->user_property);
        esp_mqtt5_client_set_publish_property(client, &publish_property);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(MQTT5, "sent publish successful, msg_id=%d", msg_id);
        */
        break;

    //Evento ao DESINSCREVER
    case MQTT_EVENT_UNSUBSCRIBED:       
        ESP_LOGI(MQTT5, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        
        //O seguinte código representa um exemplo de Desconexão
        /*
        esp_mqtt5_client_set_user_property(&disconnect_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
        esp_mqtt5_client_set_disconnect_property(client, &disconnect_property);
        esp_mqtt5_client_delete_user_property(disconnect_property.user_property);
        disconnect_property.user_property = NULL;
        esp_mqtt_client_disconnect(client);
        */
        break;

    //Evento ao PUBLICAR
    case MQTT_EVENT_PUBLISHED:      
        ESP_LOGI(MQTT5, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        break;
        
    //Evento ao RECEBER EVENTO no tópico inscrito
    case MQTT_EVENT_DATA:
        ESP_LOGI(MQTT5, "MQTT_EVENT_DATA");
        print_user_property(event->property->user_property);
        ESP_LOGI(MQTT5, "payload_format_indicator is %d", event->property->payload_format_indicator);
        ESP_LOGI(MQTT5, "response_topic is %.*s", event->property->response_topic_len, event->property->response_topic);
        ESP_LOGI(MQTT5, "correlation_data is %.*s", event->property->correlation_data_len, event->property->correlation_data);
        ESP_LOGI(MQTT5, "content_type is %.*s", event->property->content_type_len, event->property->content_type);
        ESP_LOGI(MQTT5, "TOPIC=%.*s", event->topic_len, event->topic);
        ESP_LOGI(MQTT5, "DATA=%.*s", event->data_len, event->data);
        if(xSemaphoreTake(dallyControlValuesSemaphore, t1s)){

            if(sscanf(event->data, "%9s %9s", varTag, varValue) == 2){
                
                if(strcmp(varTag,"TA") == 0){
                    tempFloatVar = strtof(varValue, &endPtr);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New appropriate temperature saved");
                        tempAdeq = tempFloatVar;
                    }
                }

                if(strcmp(varTag,"UA") == 0){
                    tempFloatVar = strtof(varValue, &endPtr);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New appropriate humidity saved");
                        umidAdeq = tempFloatVar;
                    }
                }

                if(strcmp(varTag,"AM") == 0){
                    tempFloatVar = strtof(varValue, &endPtr);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New minimal water tank level saved");
                        altuMin = tempFloatVar;
                    }
                }

                if(strcmp(varTag,"TAP") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New time for untimed water pump activation saved");
                        tempoAtivacaoProvisoria = tempIntVar;
                    }
                }

                if(strcmp(varTag,"TAT") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New time for total timed activation saved");
                        tempoAtivaçãoTotal = tempIntVar;
                    }
                }

                if(strcmp(varTag,"TAQP") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New active time that has already passed saved");
                        tempoAtivacaoQuePassou = tempIntVar;
                    }
                }
                
                if(strcmp(varTag,"H0") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time H 1/10 saved");
                        horaAtivacao[0] = tempIntVar;
                    }
                }
                
                if(strcmp(varTag,"M0") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time M 1/10 saved");
                        minAtivacao[0] = tempIntVar;
                    }
                }

                if(strcmp(varTag,"H1") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time H 2/10 saved");
                        horaAtivacao[1] = tempIntVar;
                    }
                }
                
                if(strcmp(varTag,"M1") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time M 2/10 saved");
                        minAtivacao[1] = tempIntVar;
                    }
                } 

                if(strcmp(varTag,"H2") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time H 3/10 saved");
                        horaAtivacao[2] = tempIntVar;
                    }
                }
                
                if(strcmp(varTag,"M2") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time M 3/10 saved");
                        minAtivacao[2] = tempIntVar;
                    }
                }

                if(strcmp(varTag,"H3") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time H 4/10 saved");
                        horaAtivacao[3] = tempIntVar;
                    }
                }
                
                if(strcmp(varTag,"M3") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time M 4/10 saved");
                        minAtivacao[3] = tempIntVar;
                    }
                }

                if(strcmp(varTag,"H4") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time H 5/10 saved");
                        horaAtivacao[4] = tempIntVar;
                    }
                }
                
                if(strcmp(varTag,"M4") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time M 5/10 saved");
                        minAtivacao[4] = tempIntVar;
                    }
                }

                if(strcmp(varTag,"H5") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time H 6/10 saved");
                        horaAtivacao[5] = tempIntVar;
                    }
                }
                
                if(strcmp(varTag,"M5") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time M 6/10 saved");
                        minAtivacao[5] = tempIntVar;
                    }
                }

                if(strcmp(varTag,"H6") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time H 7/10 saved");
                        horaAtivacao[6] = tempIntVar;
                    }
                }
                
                if(strcmp(varTag,"M6") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time M 7/10 saved");
                        minAtivacao[6] = tempIntVar;
                    }
                }

                if(strcmp(varTag,"H7") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time H 8/10 saved");
                        horaAtivacao[7] = tempIntVar;
                    }
                }
                
                if(strcmp(varTag,"M7") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time M 8/10 saved");
                        minAtivacao[7] = tempIntVar;
                    }
                }

                if(strcmp(varTag,"H8") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time H 9/10 saved");
                        horaAtivacao[8] = tempIntVar;
                    }
                }
                
                if(strcmp(varTag,"M8") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time M 9/10 saved");
                        minAtivacao[8] = tempIntVar;
                    }
                }

                if(strcmp(varTag,"H9") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time H 10/10 saved");
                        horaAtivacao[9] = tempIntVar;
                    }
                }
                
                if(strcmp(varTag,"M9") == 0){
                    tempIntVar = (int) strtol(varValue, &endPtr, 10);
                    if(varValue == endPtr){
                        esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
                    }else{
                        esp_mqtt_client_publish(client, topicoSYS, "Value accepted", 0, 2, 1);
                        ESP_LOGI(MQTT5,"New activation time M 10/10 saved");
                        minAtivacao[9] = tempIntVar;
                    }
                }
                for(int i = 0; i < quantidadeAtivaçõesMaxima; ++i){
                    if(horaAtivacao[i] < 24 && minAtivacao[i] < 60){
                        ++tempQtdAtivacao;
                    }
                }
                qtdAtivacao = tempQtdAtivacao;
                tempQtdAtivacao = 0;
            }else{
                esp_mqtt_client_publish(client, topicoSYS, "Invalid value", 0, 2, 1);
            }
          
            vSaveConfFile();

            xSemaphoreGive(dallyControlValuesSemaphore);
        }
        break;
    
    //Evento ao OCORRER UM ERRO
    case MQTT_EVENT_ERROR:
        ESP_LOGI(MQTT5, "MQTT_EVENT_ERROR");
        print_user_property(event->property->user_property);
        ESP_LOGI(MQTT5, "MQTT5 return code is %d", event->error_handle->connect_return_code);
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(MQTT5, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(MQTT5, "Other event id:%d", event->event_id);
        break;
    }
}

//_______GERENCIADOR_DE_EVENTOS_PARA_WIFI_______
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data){

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        //Se o Wifi iniciar, então tenta conectar
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        //Se o Wifi desconectar, tenta reconectar um numero de vezes
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(WIFI, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(WIFI,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        //Se conseguir conectar e pegar o IP
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(WIFI, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

//_______FUNÇÃO_QUE_INICIA_O_MQTT_______
static void mqtt5_app_start(void){

    //Propriedades da conexão
    esp_mqtt5_connection_property_config_t connect_property = {
        .session_expiry_interval = 10,
        .maximum_packet_size = 1024,
        .receive_maximum = 65535,
        .topic_alias_maximum = 2,
        .request_resp_info = true,
        .request_problem_info = true,
        .will_delay_interval = 10,
        .payload_format_indicator = true,
        .message_expiry_interval = 10,
        .response_topic = "/test/response",
        .correlation_data = "123456",
        .correlation_data_len = 6,
    };

    //Configurações do Cliente
    esp_mqtt_client_config_t mqtt5_cfg = {
        .broker.address.uri = EXAMPLE_CONFIG_BROKER_URL,
        .session.protocol_ver = MQTT_PROTOCOL_V_5,
        .network.disable_auto_reconnect = false,
        .credentials.username = "",
        .credentials.authentication.password = "",
        .session.last_will.topic = "/topic/will",
        .session.last_will.msg = "i will leave",
        .session.last_will.msg_len = 12,
        .session.last_will.qos = 1,
        .session.last_will.retain = true,
        .broker.verification.skip_cert_common_name_check = true,
        .session.disable_clean_session = false,
    };

    //Construtor do Handler
    client = esp_mqtt_client_init(&mqtt5_cfg);

    /* Define propriedades da conexão e do propriedades do usuário*/
    ESP_ERROR_CHECK(esp_mqtt5_client_set_user_property(&connect_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE));
    ESP_ERROR_CHECK(esp_mqtt5_client_set_user_property(&connect_property.will_user_property, user_property_arr, USE_PROPERTY_ARR_SIZE));
    ESP_ERROR_CHECK(esp_mqtt5_client_set_connect_property(client, &connect_property));

     /* Se for chamar esp_mqtt5_client_set_user_property para definir propriedades do usuário, NÃO esqueça de deleta-los.
     * esp_mqtt5_client_set_connect_property irá alocar um buffer para armazenar as user_property e você pode deleta-la depois.
     */
    esp_mqtt5_client_delete_user_property(connect_property.user_property);
    esp_mqtt5_client_delete_user_property(connect_property.will_user_property);

    /* O último argumento pode ser usado para passar dados para o event handler, nesse exemplo mqtt_event_handler */
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt5_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));
}

//_______FUNÇÃO_QUE_INICIA_O_WIFI_______
void wifi_init_sta(void){

    //Criando grupo de eventos para Wifi
    s_wifi_event_group = xEventGroupCreate();

    //Inicializando Netif
    ESP_ERROR_CHECK(esp_netif_init());

    //Inicializando event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    //Configurações do SNTP (Precisa ser configurado antes de conectar ao Wifi)
#if LWIP_DHCP_GET_NTP_SRV
    /**
     * O endereço do servidor NTP pode ser adquirido pelo servidor DHCP,
     * veja as seguintes opções do menuconfig:
     * 'LWIP_DHCP_GET_NTP_SRV' - habilita SNTP ao invés de DHCP
     * 'LWIP_SNTP_DEBUG' - habilita mensagens de debug
     *
     * NOTE: Essa chamada deve ser feita antes do ESP adiquirir o endereço IP do DHCP,
     * caso contrário a opção NTP seria rejeitada por padrão.
     */
    ESP_LOGI(SNTP, "Initializing SNTP");
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_SNTP_TIME_SERVER);
    config.start = false;                       // Inicia o serviço SNTP de forma explicita (depois de conectar)
    config.server_from_dhcp = true;             // Aceita ofertas de NTP do servidor DHCP, se houver alguma (precisa habilitar *antes* de conectar)
    config.renew_servers_after_new_IP = true;   // deixa o esp-netif atualizar servidores SNTP configurados depois de receber a concessão DHCP
    config.index_of_first_server = 1;           // atualiza do servidor num 1, deixando servidor 0 (do DHCP) intacto
    // configure the event on which we renew servers
#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    config.ip_event_to_renew = IP_EVENT_STA_GOT_IP;
#else
    config.ip_event_to_renew = IP_EVENT_ETH_GOT_IP;
#endif
    config.sync_cb = time_sync_notification_cb; // Apenas se precisarmos de alguma função de notificação
    esp_netif_sntp_init(&config);

#endif /* LWIP_DHCP_GET_NTP_SRV */


    esp_netif_create_default_wifi_sta();

    //Configurações de inicialização do Wifi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    //Registrando o handler de eventos
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));


    //Configurações de Wifi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
             /* limite de autenticação (threshold.authmode) reseta para WPA2 como padrão de a senha for compatível com o padrão WPA2 
              * (tamanho da senha => 8). Se você quiser conectar o dispositivo para uma rede com WEP/WPA depreciado, por favor defina
              * o valor do limite para WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK e defina a senha com tamanho e formato compatíveris com padrões 
              * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };

    //Inicializando o Wifi
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(WIFI, "wifi_init_sta finished.");
     /* Esperando até a conexão ser estabelecida (WIFI_CONNECTED_BIT) ou a conexão falaher pelo número máximo
     * de tentativas(WIFI_FAIL_BIT). Os bits são definidor pelo event_handler() (Veja acima) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

     /* xEventGroupWaitBits() retornar os bits antes da chamada retornar, por isso podemos testar qual evento realmente
     * aconteceu. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(WIFI, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(WIFI, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(WIFI, "UNEXPECTED EVENT");
    }
}

//_______FUNÇÃO_QUE_INICIA_O_SNTP_______
void SNTP_init_sta(void){

    //Iniciando o SNTP
#if LWIP_DHCP_GET_NTP_SRV
    ESP_LOGI(SNTP, "Starting SNTP");
    esp_netif_sntp_start();
#if LWIP_IPV6 && SNTP_MAX_SERVERS > 2
    /* Isso demonstra o uso de um endereço IPv6 como um servidor SNTP adicional
     * (designar um endereço IPv6 estático também é possível)
     */
    ip_addr_t ip6;
    if (ipaddr_aton("2a01:3f7::1", &ip6)) {    // fonte ipv6 ntp "ntp.netnod.se"
        esp_sntp_setserver(2, &ip6);
    }
#endif  /* LWIP_IPV6 */

#else
    ESP_LOGI(SNTP, "Initializing and starting SNTP");
#if CONFIG_LWIP_SNTP_MAX_SERVERS > 1
    /* Isso demonstra como configurar mais de um servidor
     */
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(2,
                               ESP_SNTP_SERVER_LIST(CONFIG_SNTP_TIME_SERVER, "pool.ntp.org" ) );
#else
    /*
     * Essa é a configuração basica padrão com um servidor e a inicialização do serviço
     */
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_SNTP_TIME_SERVER);
#endif
    config.sync_cb = time_sync_notification_cb;     // Nota: Isso só é necessário se quisermos
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    config.smooth_sync = false;
#endif

    esp_netif_sntp_init(&config);
#endif

    print_servers();

    // Espera o tempo ser definido
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 5;
    while (esp_netif_sntp_sync_wait(t5s) == ESP_ERR_TIMEOUT && ++retry < retry_count) {
        ESP_LOGI(SNTP, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    }
    time(&now);
    localtime_r(&now, &timeinfo);

    esp_netif_sntp_deinit();
}

//_______FUNÇÃO_QUE_INICIA_O_ADC1_E_ADC2_______
void ADC1_plus_ADC2_init_sta(void){

    ESP_LOGI(ADC, "Starting ADC1 and ADC2");
    // Configuração de inicialização do ADC1
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // Configuração de inicialização do ADC2
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    // Configuração do canal de leitura oneshot
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    ESP_LOGI(ADC, "Starting pin channels for LDR;");
    // Configurando as entradas ADC para leitura dos sensores
    //--------------LDR--------------//
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, LDR_N, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, LDR_O, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, LDR_L, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, LDR_S, &config));

    ESP_LOGI(ADC, "Starting pin channels for Thermistor;");
    //--------------THERMISTOR--------------//
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, THERMISTOR_N, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, THERMISTOR_O, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, THERMISTOR_L, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, THERMISTOR_S, &config));

    ESP_LOGI(ADC, "Starting pin channels for Soil Hygrometer;");
    //--------------SOIL HIGROMETER--------------//
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, SOIL_HYGROMETER, &config));

    ESP_LOGI(ADC, "Starting calibration scheme for ADC1 and ADC2");
    // Configuração de inicialização de calibração ADC1
    adc_cali_line_fitting_config_t cali_config1 = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config1, &adc1_cali_handle));
    
    // Configuração de inicialização de calibração ADC2
    adc_cali_line_fitting_config_t cali_config2 = {
        .unit_id = ADC_UNIT_2,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config2, &adc2_cali_handle));
}

//_______FUNÇÃO_QUE_INICIA_O_LEDC_______
void ledc_init_sta(void){

    //Prepara e aplica a configuração do timer do LEDC PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = 4000,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    //Prepara e aplica a configuração do canal LEDC PWM
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .gpio_num       = LEDC_BOMBA_AGUA,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0,
#if CONFIG_PM_ENABLE
        .sleep_mode     = LEDC_SLEEP_MODE_KEEP_ALIVE,
#endif
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

//_______FUNÇÃO_QUE_INICIA_O_MCPWM_______
void mcpwm_init_sta(void){
    ESP_LOGI(MCPWM, "Install capture timer");
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer));

    ESP_LOGI(MCPWM, "Install capture channel");
    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = AJ_SR04M_ECHO_GPIO,
        .prescale = 1,
        // capture on both edge
        .flags.neg_edge = true,
        .flags.pos_edge = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));
    // pull up the GPIO internally
    ESP_ERROR_CHECK(gpio_set_pull_mode(AJ_SR04M_ECHO_GPIO, GPIO_PULLUP_ONLY));

    ESP_LOGI(MCPWM, "Register capture callback");
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = aj_sr04m_echo_callback,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, xHandleSensorValues));

    ESP_LOGI(MCPWM, "Enable capture channel");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));

    ESP_LOGI(MCPWM, "Configure Trig pin");
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << AJ_SR04M_TRIG_GPIO,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // drive low by default
    ESP_ERROR_CHECK(gpio_set_level(AJ_SR04M_TRIG_GPIO, 0));

    ESP_LOGI(MCPWM, "Enable and start capture timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

}

//_______FUNÇÃO_QUE_INICIA_O_LITTLEFS_______
void littlefs_init_sta(){
    
    ESP_LOGI(LittleFS, "Initializing LittleFS");

    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/files",
        .partition_label = "storage",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(LittleFS, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(LittleFS, "Failed to find LittleFS partition");
        } else {
            ESP_LOGE(LittleFS, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(LittleFS, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
        esp_littlefs_format(conf.partition_label);
    } else {
        ESP_LOGI(LittleFS, "Partition size: total: %d, used: %d", total, used);
    }

}

//_______FUNÇÃO_QUE_LÊ_OS_VALORES_DE_CONFIGURAÇÃO_ARMAZENADOS_______
void vLoadingDefaultValues(){

    //Pegando dados do arquivo de configuração
    FILE *fConfig;

    char varTag[10];
    char varValue[10];
    char *endPtr;

    //Pegando tempo de agora
    time_t tempoAgora;
    struct tm timeinfo;
    time(&tempoAgora);
    setenv("TZ", "UTC+3", 1);
    tzset();
    localtime_r(&tempoAgora, &timeinfo);
    ESP_LOGI(loadingDefaultValues, "Opening config file values");
    if(xSemaphoreTake(configFileSemaphore,t1s)){
        qtdAtivacao = 0;
        fConfig = fopen(configFileName, "r");
        if(xSemaphoreTake(dallyControlValuesSemaphore,t1s)){
            if(fConfig != NULL){
                ESP_LOGI(loadingDefaultValues, "File opened successfully!");
                while(fscanf(fConfig, "%9s %9s", varTag, varValue) == 2){
                    //-------------------TA-------------------
                    if(strcmp(varTag,"TA") == 0){
                        tempAdeq = strtof(varValue, &endPtr);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load appropriate temperature, using default: 35");
                            tempAdeq = temperaturaAdequadaPadrão;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded appropriate temperature value, successfully!");
                        }
                    }
                    //-------------------UA-------------------
                    if(strcmp(varTag,"UA") == 0){
                        umidAdeq = strtof(varValue, &endPtr);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load appropriate humidity, using default: 70");
                            umidAdeq = umidadeAdequadaPadrão;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded appropriate humidity value, successfully!");
                        }
                    }
                    //-------------------AM-------------------
                    if(strcmp(varTag,"AM") == 0){
                        altuMin = strtof(varValue, &endPtr);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load minimal water tank level, using default: 32.5");
                            altuMin = alturaMinimaPadrão;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded minimal water tank level value, successfully!");
                        }
                    }
                    //-------------------TAP-------------------
                    if(strcmp(varTag,"TAP") == 0){
                        tempoAtivacaoProvisoria = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load time for untimed water pump activation, using default: 1");
                            tempoAtivacaoProvisoria = t1m;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded time for untimed water pump activation, successfully!");
                        }
                    }
                    //-------------------TAT-------------------
                    if(strcmp(varTag,"TAT") == 0){
                        tempoAtivaçãoTotal = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load time for total timed activation, using default: 10");
                            tempoAtivaçãoTotal = t10m;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded time for total timed activation, successfully!");
                        }
                    }
                    //-------------------TAQP-------------------
                    if(strcmp(varTag,"TAQP") == 0){
                        tempoAtivacaoQuePassou = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load active time that has already passed, using default: 0");
                            tempoAtivacaoQuePassou = t10m;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded active time that has already passed, successfully!");
                        }
                    }
                    //-------------------CAR-------------------
                    if(strcmp(varTag,"CAR") == 0){
                        contaAtivacaoRestante = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load amount of activation times that are left, using default: 3");
                            contaAtivacaoRestante = quantidadeAtivaçõesPadrão;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded amount of activation times that are left, successfully!");
                        }
                    }
                    //-------------------UATSEC-------------------
                    if(strcmp(varTag,"UATSEC") == 0){
                        ultimaAtivacao.tm_sec = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load active time that has already passed (1/9), using current time");
                            ultimaAtivacao.tm_sec = timeinfo.tm_sec;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded active time that has already passed (1/9), successfully!");
                        }
                    }
                    //-------------------UATMIN-------------------
                    if(strcmp(varTag,"UATMIN") == 0){
                        ultimaAtivacao.tm_min = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load active time that has already passed (2/9), using current time");
                            ultimaAtivacao.tm_min = timeinfo.tm_min;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded active time that has already passed (2/9), successfully!");
                        }
                    }
                    //-------------------UATHOUR-------------------
                    if(strcmp(varTag,"UATHOUR") == 0){
                        ultimaAtivacao.tm_hour = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load active time that has already passed (3/9), using current time");
                            ultimaAtivacao.tm_hour = timeinfo.tm_hour;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded active time that has already passed (3/9), successfully!");
                        }
                    }
                    //-------------------UATMDAY-------------------
                    if(strcmp(varTag,"UATMDAY") == 0){
                        ultimaAtivacao.tm_mday = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load active time that has already passed (4/9), using current time");
                            ultimaAtivacao.tm_mday = timeinfo.tm_mday;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded active time that has already passed (4/9), successfully!");
                        }
                    }
                    //-------------------UATMON-------------------
                    if(strcmp(varTag,"UATMON") == 0){
                        ultimaAtivacao.tm_mon = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load active time that has already passed (5/9), using current time");
                            ultimaAtivacao.tm_mon = timeinfo.tm_mon;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded active time that has already passed (5/9), successfully!");
                        }
                    }
                    //-------------------UATYEAR-------------------
                    if(strcmp(varTag,"UATYEAR") == 0){
                        ultimaAtivacao.tm_year = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load active time that has already passed (6/9), using current time");
                            ultimaAtivacao.tm_year = timeinfo.tm_year;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded active time that has already passed (6/9), successfully!");
                        }
                    }
                    //-------------------UATWDAY-------------------
                    if(strcmp(varTag,"UATWDAY") == 0){
                        ultimaAtivacao.tm_wday = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load active time that has already passed (7/9), using current time");
                            ultimaAtivacao.tm_wday = timeinfo.tm_wday;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded active time that has already passed (7/9), successfully!");
                        }
                    }
                    //-------------------UATYDAY-------------------
                    if(strcmp(varTag,"UATYDAY") == 0){
                        ultimaAtivacao.tm_yday = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load active time that has already passed (8/9), using current time");
                            ultimaAtivacao.tm_yday = timeinfo.tm_yday;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded active time that has already passed (8/9), successfully!");
                        }
                    }
                    //-------------------UATISDST-------------------
                    if(strcmp(varTag,"UATISDST") == 0){
                        ultimaAtivacao.tm_isdst = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load active time that has already passed (9/9), using current time");
                            ultimaAtivacao.tm_isdst = timeinfo.tm_isdst;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded active time that has already passed (9/9), successfully!");
                        }
                    }
                    //-------------------H0-------------------
                    if(strcmp(varTag,"H0") == 0){
                        horaAtivacao[0] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time H 1/10, using current time, using default: 7");
                            horaAtivacao[0] = timeHour1;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time H 1/10, successfully!");
                        }
                        if(horaAtivacao[0] != 24){
                            ++qtdAtivacao;
                        }
                    }
                    //-------------------M0-------------------
                    if(strcmp(varTag,"M0") == 0){
                        minAtivacao[0] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time M 1/10, using current time, using default: 30");
                            minAtivacao[0] = timeMin1;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time M 1/10, successfully!");
                        }
                    }
                    //-------------------H1-------------------
                    if(strcmp(varTag,"H1") == 0){
                        horaAtivacao[1] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time H 2/10, using current time, using default: 12");
                            horaAtivacao[1] = timeHour2;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time H 2/10, successfully!");
                        }
                        if(horaAtivacao[1] != 24){
                            ++qtdAtivacao;
                        }
                    }
                    //-------------------M1-------------------
                    if(strcmp(varTag,"M1") == 0){
                        minAtivacao[1] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time M 2/10, using current time, using default: 0");
                            minAtivacao[1] = timeMin2;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time M 2/10, successfully!");
                        }
                    }
                    //-------------------H2-------------------
                    if(strcmp(varTag,"H2") == 0){
                        horaAtivacao[2] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time H 3/10, using current time, using default: 18");
                            horaAtivacao[2] = timeHour3;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time H 3/10, successfully!");
                        }
                        if(horaAtivacao[2] != 24){
                            ++qtdAtivacao;
                        }
                    }
                    //-------------------M2-------------------
                    if(strcmp(varTag,"M2") == 0){
                        minAtivacao[2] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time M 3/10, using current time, using default: 30");
                            minAtivacao[2] = timeMin3;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time M 3/10, successfully!");
                        }
                    }
                    //-------------------H3-------------------
                    if(strcmp(varTag,"H3") == 0){
                        horaAtivacao[3] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time H 4/10, using current time, using default: 24");
                            horaAtivacao[3] = timeHourNotConf;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time H 4/10, successfully!");
                        }
                        if(horaAtivacao[3] != 24){
                            ++qtdAtivacao;
                        }
                    }
                    //-------------------M3-------------------
                    if(strcmp(varTag,"M3") == 0){
                        minAtivacao[3] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time M 4/10, using current time, using default: 60");
                            minAtivacao[3] = timeMinNotConf;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time M 4/10, successfully!");
                        }
                    }
                    //-------------------H4-------------------
                    if(strcmp(varTag,"H4") == 0){
                        horaAtivacao[4] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time H 5/10, using current time, using default: 24");
                            horaAtivacao[4] = timeHourNotConf;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time H 5/10, successfully!");
                        }
                        if(horaAtivacao[4] != 24){
                            ++qtdAtivacao;
                        }
                    }
                    //-------------------M4-------------------
                    if(strcmp(varTag,"M4") == 0){
                        minAtivacao[4] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time M 5/10, using current time, using default: 60");
                            minAtivacao[4] = timeMinNotConf;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time M 5/10, successfully!");
                        }
                    }
                    //-------------------H5-------------------
                    if(strcmp(varTag,"H5") == 0){
                        horaAtivacao[5] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time H 6/10, using current time, using default: 24");
                            horaAtivacao[5] = timeHourNotConf;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time H 6/10, successfully!");
                        }
                        if(horaAtivacao[5] != 24){
                            ++qtdAtivacao;
                        }
                    }
                    //-------------------M5-------------------
                    if(strcmp(varTag,"M5") == 0){
                        minAtivacao[5] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time M 6/10, using current time, using default: 60");
                            minAtivacao[5] = timeMinNotConf;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time M 6/10, successfully!");
                        }
                    }
                    //-------------------H6-------------------
                    if(strcmp(varTag,"H6") == 0){
                        horaAtivacao[6] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time H 7/10, using current time, using default: 24");
                            horaAtivacao[6] = timeHourNotConf;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time H 7/10, successfully!");
                        }
                        if(horaAtivacao[6] != 24){
                            ++qtdAtivacao;
                        }
                    }
                    //-------------------M6-------------------
                    if(strcmp(varTag,"M6") == 0){
                        minAtivacao[6] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time M 7/10, using current time, using default: 60");
                            minAtivacao[6] = timeMinNotConf;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time M 7/10, successfully!");
                        }
                    }
                    //-------------------H7-------------------
                    if(strcmp(varTag,"H7") == 0){
                        horaAtivacao[7] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time H 8/10, using current time, using default: 24");
                            horaAtivacao[7] = timeHourNotConf;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time H 8/10, successfully!");
                        }
                        if(horaAtivacao[7] != 24){
                            ++qtdAtivacao;
                        }
                    }
                    //-------------------M7-------------------
                    if(strcmp(varTag,"M7") == 0){
                        minAtivacao[7] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time M 8/10, using current time, using default: 60");
                            minAtivacao[7] = timeMinNotConf;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time M 8/10, successfully!");
                        }
                    }
                    //-------------------H8-------------------
                    if(strcmp(varTag,"H8") == 0){
                        horaAtivacao[8] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time H 9/10, using current time, using default: 24");
                            horaAtivacao[8] = timeHourNotConf;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time H 9/10, successfully!");
                        }
                        if(horaAtivacao[8] != 24){
                            ++qtdAtivacao;
                        }
                    }
                    //-------------------M8-------------------
                    if(strcmp(varTag,"M8") == 0){
                        minAtivacao[8] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time M 9/10, using current time, using default: 60");
                            minAtivacao[8] = timeMinNotConf;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time M 9/10, successfully!");
                        }
                    }
                    //-------------------H9-------------------
                    if(strcmp(varTag,"H9") == 0){
                        horaAtivacao[9] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time H 10/10, using current time, using default: 24");
                            horaAtivacao[9] = timeHourNotConf;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time H 10/10, successfully!");
                        }
                        if(horaAtivacao[9] != 24){
                            ++qtdAtivacao;
                        }
                    }
                    //-------------------M9-------------------
                    if(strcmp(varTag,"M9") == 0){
                        minAtivacao[9] = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load activation time M 10/10, using current time, using default: 60");
                            minAtivacao[9] = timeMinNotConf;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded activation time M 10/10, successfully!");
                        }
                    }
                    //-------------------TSM1-------------------
                    if(strcmp(varTag,"TSM1") == 0){
                        timestampMarker[0] = (bool) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load timestamp marker 1 value, using default: false");
                            timestampMarker[0] = false;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded timestamp marker 1 value, successfully!");
                        }
                    }
                    //-------------------TSM2-------------------
                    if(strcmp(varTag,"TSM2") == 0){
                        timestampMarker[1] = (bool) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load timestamp marker 2 value, using default: false");
                            timestampMarker[1] = false;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded timestamp marker 2 value, successfully!");
                        }
                    }
                    //-------------------TSM3-------------------
                    if(strcmp(varTag,"TSM3") == 0){
                        timestampMarker[2] = (bool) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load timestamp marker 3 value, using default: false");
                            timestampMarker[2] = false;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded timestamp marker 3 value, successfully!");
                        }
                    }
                    //-------------------TSM4-------------------
                    if(strcmp(varTag,"TSM4") == 0){
                        timestampMarker[3] = (bool) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load timestamp marker 4 value, using default: false");
                            timestampMarker[3] = false;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded timestamp marker 4 value, successfully!");
                        }
                    }
                    //-------------------TSM5-------------------
                    if(strcmp(varTag,"TSM5") == 0){
                        timestampMarker[4] = (bool) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load timestamp marker 5 value, using default: false");
                            timestampMarker[4] = false;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded timestamp marker 5 value, successfully!");
                        }
                    }
                    //-------------------TSM6-------------------
                    if(strcmp(varTag,"TSM6") == 0){
                        timestampMarker[5] = (bool) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load timestamp marker 6 value, using default: false");
                            timestampMarker[5] = false;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded timestamp marker 6 value, successfully!");
                        }
                    }
                    //-------------------TSM7-------------------
                    if(strcmp(varTag,"TSM7") == 0){
                        timestampMarker[6] = (bool) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load timestamp marker 7 value, using default: false");
                            timestampMarker[6] = false;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded timestamp marker 7 value, successfully!");
                        }
                    }
                    //-------------------TSM8-------------------
                    if(strcmp(varTag,"TSM8") == 0){
                        timestampMarker[7] = (bool) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load timestamp marker 8 value, using default: false");
                            timestampMarker[7] = false;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded timestamp marker 8 value, successfully!");
                        }
                    }
                    //-------------------TSM9-------------------
                    if(strcmp(varTag,"TSM9") == 0){
                        timestampMarker[8] = (bool) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load timestamp marker 9 value, using default: false");
                            timestampMarker[8] = false;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded timestamp marker 9 value, successfully!");
                        }
                    }
                    //-------------------TSM10-------------------
                    if(strcmp(varTag,"TSM10") == 0){
                        timestampMarker[9] = (bool) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load timestamp marker 9 value, using default: false");
                            timestampMarker[9] = false;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded timestamp marker 9 value, successfully!");
                        }
                    }
                    //-------------------ND-------------------
                    if(strcmp(varTag,"ND") == 0){
                        nextDay = (int) strtol(varValue, &endPtr, 10);
                        if(varValue == endPtr){
                            ESP_LOGI(loadingDefaultValues, "Could not load next day value, using default: false");
                            nextDay = 7;
                        }else{
                            ESP_LOGI(loadingDefaultValues, "Loaded next day value, successfully!");
                        }
                    }

                }
                fclose(fConfig);
                ESP_LOGI(loadingDefaultValues,"Calculating activation time per period");

                if(contaAtivacaoRestante == 0){
                    tempoAtivaçãoPorPeriodo = 0;
                }else{
                    tempoAtivaçãoPorPeriodo = (tempoAtivaçãoTotal - tempoAtivacaoQuePassou)/contaAtivacaoRestante;
                }
                
            }else{
                ESP_LOGI(loadingDefaultValues, "Could not open config file, loading default values");
                
                //Carrega valores padrão para varificar o estado do vaso
                tempAdeq = temperaturaAdequadaPadrão;
                umidAdeq = umidadeAdequadaPadrão;
                altuMin = alturaMinimaPadrão;

                //Carrega os valores padrões de tempo 
                qtdAtivacao = quantidadeAtivaçõesPadrão;
                tempoAtivacaoProvisoria = t1m;
                tempoAtivaçãoTotal = t10m;
                tempoAtivaçãoPorPeriodo = tempoAtivaçãoTotal/qtdAtivacao;
                contaAtivacaoRestante = qtdAtivacao; 
                tempoAtivacaoQuePassou = 0;
                
                ultimaAtivacao = timeinfo;
                nextDay = 7;
                for(int i = 0; i < quantidadeAtivaçõesMaxima; i++){
                    switch (i)
                    {
                    case 0:
                        horaAtivacao[i] = timeHour1;
                        minAtivacao[i] = timeMin1;
                        break;

                    case 1:
                        horaAtivacao[i] = timeHour2;
                        minAtivacao[i] = timeMin2;
                        break;

                    case 2:
                        horaAtivacao[i] = timeHour3;
                        minAtivacao[i] = timeMin3;
                        break;
                    
                    default:
                        horaAtivacao[i] = timeHourNotConf;
                        minAtivacao[i] = timeMinNotConf;
                        break;
                    }
                }
            }
            ESP_LOGI(loadingDefaultValues,"Loading new next day value...");
            if(nextDay == 7){
                if(timeinfo.tm_wday == 6){
                    nextDay = 0;
                }else{
                    nextDay = timeinfo.tm_wday + 1;
                }
            }
            ESP_LOGI(loadingDefaultValues,"Next day value loaded!");

            xSemaphoreGive(dallyControlValuesSemaphore);
        }
        xSemaphoreGive(configFileSemaphore);
    }
    
    ESP_LOGI(loadingDefaultValues, "Configurations loaded!");
}
//_______________________________DECLARAÇÃO_DE_TAREFAS_DO_RTOS_______________________________
void vDecision( void * pvParameters );
void vDallyReset( void * pvParameters );
void vSensorValues( void * pvParameters );
void vProcessosMqtt( void * pvParameters );
void vSaveSystemData( void * pvParameters );
void vSleepSetup( void * pvParameters );

void app_main(void){
    //Logs do Sistema
    ESP_LOGI(MQTT5, "[APP] Startup..");
    ESP_LOGI(MQTT5, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(MQTT5, "[APP] IDF version: %s", esp_get_idf_version());

    //Logs do MQTT
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    //Inicializa o as variáveis de tempo
    for(int i = 0; i < quantidadeAtivaçõesMaxima; i++){
        switch (i)
        {
        case 0:
            horaAtivacao[i] = timeHour1;
            minAtivacao[i] = timeMin1;
            break;

        case 1:
            horaAtivacao[i] = timeHour2;
            minAtivacao[i] = timeMin2;
            break;

        case 2:
            horaAtivacao[i] = timeHour3;
            minAtivacao[i] = timeMin3;
            break;
        
        default:
            horaAtivacao[i] = timeHourNotConf;
            minAtivacao[i] = timeHourNotConf;
            break;
        }
    }

    bombaDagua = BOMBA_DESLIGADA;

    //Cria fila de caracteres
    sensor_LDR_queue_handle = xQueueCreate( queueSize, messageSize);
    sensor_THR_queue_handle = xQueueCreate( queueSize, messageSize );
    sensor_SHR_queue_handle = xQueueCreate( queueSize, messageSize );
    sensor_UTS_queue_handle = xQueueCreate( queueSize, messageSize );
    funcionamento_WPM_queue_handle = xQueueCreate( queueSize,  messageSize);

    //Cria semaforos para proteção de dados
    sensorValuesSemaphore = xSemaphoreCreateMutex();
    dallyControlValuesSemaphore = xSemaphoreCreateMutex(); 
    mqttAvailable = xSemaphoreCreateBinary();
    writeDataFile = xSemaphoreCreateBinary();
    dataFileSemaphore = xSemaphoreCreateMutex();
    configFileSemaphore = xSemaphoreCreateMutex();

    //Iniciando NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (CONFIG_LOG_MAXIMUM_LEVEL > CONFIG_LOG_DEFAULT_LEVEL) {
         /* Se você só quiser abrir mais logs no módulo Wifi, você precisa fazer o nível máximo maior do que o nível padrão,
         * e chamar esp_log_level_set() antes esp_wifi_init() para melhorar o nível de log do módulo Wifi. */
        esp_log_level_set("wifi", CONFIG_LOG_MAXIMUM_LEVEL);
    }

    //Chamando a função que iniciar o WIFI
    ESP_LOGI(WIFI, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    //Configurações de tempo
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    ESP_LOGI(SNTP, "Time is not set yet. Connecting to WiFi and getting time over NTP.");

        //Chamando a função que atualiza a hora atual por SNTP;
    SNTP_init_sta();

        //Atualiza "now" com o tempo atual
    time(&now);


    //Chamando a função que inicia MQTT
    mqtt5_app_start();

    //Chamando a função que inicia as funções do ADC
    ADC1_plus_ADC2_init_sta();

    //Chamando a função que inicia o LEDC
    ledc_init_sta();

    //Chamando a função que inicia o LittleFS
    littlefs_init_sta();

    //Chamando função que inicia os valores de configuração
    vLoadingDefaultValues();

    //Inicializando Handlers de tarefas como nulo
    xHandleSensorValues = NULL;
    xHandleDecision = NULL;
    xHandleProcessosMqtt = NULL;
    xHandleDallyReset = NULL;
    xHandleSaveSystemData = NULL;
    xHandleSleepSetup = NULL;

    char strftime_buf[64];
    // Definindo timezone para o horário padrão de Brazília (UTC+3)
    setenv("TZ", "UTC+3", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(SNTP, "The current date/time in Brasilia is: %s", strftime_buf);

    // Criação das tarefas
    xTaskCreatePinnedToCore(&vSensorValues, "sensorValues", STACK_SIZE_SENSOR_VALUES, ( void * ) 1, 5, &xHandleSensorValues, 0);
    xTaskCreatePinnedToCore(&vDecision, "decision", STACK_SIZE_DECISION, ( void * ) 1, 4, &xHandleDecision, 0);
    xTaskCreatePinnedToCore(&vProcessosMqtt, "processosMqtt", STACK_SIZE_PROCESSOS_MQTT, ( void * ) 1, 5, &xHandleProcessosMqtt, 1);
    xTaskCreatePinnedToCore(&vSaveSystemData, "saveSystemData", STACK_SIZE_PROCESSOS_MQTT, ( void * ) 1, 4, &xHandleSaveSystemData, 1);
    xTaskCreate(&vDallyReset, "dallyReset", STACK_SIZE_DALLY_RESET, ( void * ) 1, 3, &xHandleDallyReset);
    xTaskCreate(&vSleepSetup, "sleepSetup", STACK_SIZE_DALLY_RESET, ( void * ) 1, 3, &xHandleSleepSetup);

}

//_____________________________IMPLEMENTAÇÃO_DAS_TAREFAS_DO_RTOS_____________________________

void vSleepSetup( void * pvParameters ){
    //Função que vai colocar o sistema para dormir
    bool queueStatus = false;
    
    //Variáveis de tempo
    time_t tempoAgora;
    struct tm timeinfo;
    struct tm nextTime;
    uint64_t timeInMs = 0;
    while(1){    
        vTaskDelay(t90s);
        //Pegando o tempo atual
        time(&tempoAgora);
        setenv("TZ", "UTC+3", 1);
        tzset();
        localtime_r(&tempoAgora, &timeinfo);
        nextTime = timeinfo;

        ESP_LOGI(sleepSetup,"Starting Sleep setup...");
        if(xSemaphoreTake(dataFileSemaphore, t1s) == pdTRUE){
            ESP_LOGI(sleepSetup,"Not writing data file, OK");
            if(xSemaphoreTake(configFileSemaphore, t1s) == pdTRUE){
                ESP_LOGI(sleepSetup,"Not writing config file, OK");
                if(xSemaphoreTake(dallyControlValuesSemaphore, t1s) == pdTRUE){
                    ESP_LOGI(sleepSetup,"Not manipulating control values, OK");
                    if(xSemaphoreTake(sensorValuesSemaphore, t1s) == pdTRUE){
                        ESP_LOGI(sleepSetup,"Not reading sensor values, OK");
                        if(bombaDagua == BOMBA_DESLIGADA){
                            ESP_LOGI(sleepSetup,"Water Pump is turned off, OK");
                            
                            ESP_LOGI(sleepSetup,"Checking queues status...");
                            queueStatus = 0 == uxQueueMessagesWaiting(sensor_LDR_queue_handle);
                            queueStatus = queueStatus && 0 == uxQueueMessagesWaiting(sensor_THR_queue_handle);
                            queueStatus = queueStatus && 0 == uxQueueMessagesWaiting(sensor_SHR_queue_handle);
                            queueStatus = queueStatus && 0 == uxQueueMessagesWaiting(sensor_UTS_queue_handle);
                            queueStatus = queueStatus && 0 == uxQueueMessagesWaiting(funcionamento_WPM_queue_handle);
                            if(queueStatus){
                                ESP_LOGI(sleepSetup, "No messages left on queue. OK");
                                ESP_LOGI(sleepSetup,"Falling asleep");
                                for(int i = 1; i<=5 ; i++){
                                    if((timeinfo.tm_min+i)%5 == 0){
                                        nextTime.tm_min = (timeinfo.tm_min + i) % 60;
                                        nextTime.tm_sec = 0;
                                        timeInMs = ((uint64_t)(i * 60) - timeinfo.tm_sec) * 1000000ULL;
                                        break;
                                    }
                                }
                                gpio_deep_sleep_hold_en();
                                gpio_pulldown_en(LEDC_BOMBA_AGUA);
                                esp_sleep_enable_timer_wakeup(timeInMs);
                                esp_deep_sleep_start();
                            }else{
                                ESP_LOGI(sleepSetup,"There are still untreated messages left on queue. Failed to fall asleep");
                            }
                        }else{
                            ESP_LOGI(sleepSetup,"Water Pump is turned on. Failed to fall asleep");
                        }
                        xSemaphoreGive(sensorValuesSemaphore);
                    }else{
                        ESP_LOGI(sleepSetup,"Reading sensor values. Failed to fall asleep");
                    }
                    xSemaphoreGive(dallyControlValuesSemaphore);
                }else{
                    ESP_LOGI(sleepSetup,"Manipulating control values. Failed to fall asleep");
                }
                xSemaphoreGive(configFileSemaphore);
            }else{
                ESP_LOGI(sleepSetup,"Writing config file. Failed to fall asleep");
            }
            xSemaphoreGive(dataFileSemaphore);
        }else{
            ESP_LOGI(sleepSetup,"Writing data file. Failed to fall asleep");
        }


    }
}

void vSaveSystemData(void * pvParameters){

    ESP_LOGI(saveDataFile,"Starting system data save task");
    
    FILE *fData;    //Variável para leitura do arquivo
    char fileLine[messageSize];     //Variável que recebe o valor das mensagens das filas

    while(1){
        //Verifica se pode escrever no arquivo
        ESP_LOGI(saveDataFile, "Waiting for permission to write file...");
        if(xSemaphoreTake(writeDataFile, t1s)==pdTRUE){
            
            ESP_LOGI(saveDataFile, "Permission granted!");

            if(xSemaphoreTake(dataFileSemaphore,t1s) == pdTRUE){

                ESP_LOGI(saveDataFile, "Trying to open file.");
                
                //Abrindo arquivo de dados
                fData = fopen(dataFileName, "a");
                
                //Se o arquivo for null então imprime mensagem de erro
                if(fData != NULL){
                    ESP_LOGI(saveDataFile, "File open, writing data...");
                    //Enquanto houverem mensagens na fila
                    while(0 < uxQueueMessagesWaiting(sensor_LDR_queue_handle)){
                        //Recebendo mensagem da fila do LDR
                        if(xQueueReceive(sensor_LDR_queue_handle, fileLine, t1s)){
                            fprintf(fData,"LDR\n");
                            fprintf(fData,"%s\n",fileLine);
                        }
                    }
                    
                    while(0 < uxQueueMessagesWaiting(sensor_THR_queue_handle)){
                        //Recebendo mensagem da fila do Thermistor
                        if(xQueueReceive(sensor_THR_queue_handle, fileLine, t1s)){
                            fprintf(fData,"THR\n");
                            fprintf(fData,"%s\n",fileLine);
                        }
                    }

                    while(0 < uxQueueMessagesWaiting(sensor_SHR_queue_handle)){
                        //Recebendo mensagem da fila do Soil Hygrometer
                        if(xQueueReceive(sensor_SHR_queue_handle, fileLine, t1s)){
                            fprintf(fData,"SHR\n");
                            fprintf(fData,"%s\n",fileLine);
                        }
                    }

                    while(0 < uxQueueMessagesWaiting(sensor_UTS_queue_handle)){
                        //Recebendo mensagem da fila do Ultrassound
                        if(xQueueReceive(sensor_UTS_queue_handle, fileLine, t1s)){
                            fprintf(fData,"UTS\n");
                            fprintf(fData,"%s\n",fileLine);
                        }
                    }

                    while(0 < uxQueueMessagesWaiting(funcionamento_WPM_queue_handle)){
                        //Recebendo mensagem da fila da Bomba d'agua
                        if(xQueueReceive(funcionamento_WPM_queue_handle, fileLine, t1s)){
                            fprintf(fData,"WPM\n");
                            fprintf(fData,"%s\n",fileLine);
                        }
                    }

                    fclose(fData);
                    ESP_LOGI(saveDataFile, "Data written successfully!");
                }else{
                    ESP_LOGE(saveDataFile, "Could not open file.");
                }
                xSemaphoreGive(dataFileSemaphore);
            }else{
                ESP_LOGE(saveDataFile, "Failed to write data!");
            }
            
        }
        vTaskDelay(t60s);
    }
}
//_______TAREFA_QUE_É_RESPONSÁVEL_PELO_FUNCIONAMENTO_DA_BOMBA_______
void vDecision( void * pvParameters ){

    ESP_LOGI(decision,"Starting decision making task");

    //Variáveis referentes aos sensores
    float umidade = 0.0f;  //Variável para salvar o valor de umidade localmente
    float temperaturaMedia = 0.0f; //Variável para calcular a temperatura média
    float altura = 0.0f;    //Variável que salva o valor da altura da culuna d'agua

    bool comTimer = 0;  //Variável que controla quando a bomba foi ativada com ou sem timer
    
    bool condicaoAtivacao[quantidadeAtivaçõesMaxima] = {0}; //Verifica se chegou na condição de ativação
    bool ativou = 0; //Indica para o sistema que ele chegou a uma das condições de ativação

    char timeStringUltimaAtivacao[64];  //Converte ultimaAtivacao em string

    int timeDiff = 0;    //Salva a diferença de horas em minutos

    //Marca quais tempos do dia ele já passaram
    for(int i = 0; i < qtdAtivacao; i++){
        timestampMarker[i] = 0;
    }

    // Variáveis para salvar os valores do RTC
    time_t tempoAgora;
    struct tm timeinfo;
    char timeString[64];
    
    //Mensagem para a fila
    char message_WPM[messageSize];   //Mensagem para os sensores LDRs

    while(1){
        
        //Pegando o tempo atual
        time(&tempoAgora);
        setenv("TZ", "UTC+3", 1);
        tzset();
        localtime_r(&tempoAgora, &timeinfo);
        //Se a última ativação não tiver sido definido, defina como igual ao tempo atual, só na primeira vez.
        if(ultimaAtivacao.tm_year < 100){
            ultimaAtivacao = timeinfo;
        }

        //Pega o semáforo para calcular a média de temperaturas
        if(xSemaphoreTake(sensorValuesSemaphore, t1s) == pdTRUE){
            ESP_LOGI(decision,"Calculating avarage temperature and getting sensor values");
            //Pega a temperatura média dos quatro sensores de temperatura
            temperaturaMedia = (thr_tem[0] + thr_tem[1] + thr_tem[2] + thr_tem[3])/4;
            umidade = shr_pct;
            altura = uts_dis;
                
            //Devolve o semaforo
            xSemaphoreGive(sensorValuesSemaphore);
        }

        if(xSemaphoreTake(dallyControlValuesSemaphore, t1s) == pdTRUE){
            //Verifica o que fazer caso
            switch(bombaDagua){
                case BOMBA_DESLIGADA:
                    ESP_LOGI(decision,"Water pump OFF");

                    //Verifica diferentes condições para ativar a bomba de água
                    if(temperaturaMedia > tempAdeq && umidade < umidAdeq && altuMin > altura){
                        ESP_LOGI(decision,"Turning on water pump (emergency activation)");
                        //Altera a variável para ligado
                        bombaDagua = BOMBA_LIGADA;
                                
                        //Salva o tempo da última ativação
                        ultimaAtivacao = timeinfo;

                        //Sem timer
                        comTimer = 0;

                        //Atualiza o duty cycle e ativa a bomba
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_ON));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL)); 
                    }         
                    //Verifica condição de ativação por tempo
                    ESP_LOGI(decision,"Checking turn on timed values for the water pump");
                    for(int i = 0; i< qtdAtivacao; i++){
                        condicaoAtivacao[i] = timestampMarker[i] == 0 && timeinfo.tm_hour == horaAtivacao[i] && timeinfo.tm_min == minAtivacao[i];
                        if(condicaoAtivacao[i]){
                            ativou = true;
                        }
                    }

                    //Verifica se é para ativar a bomba de ativação da bomba
                    if(ativou){
                        ESP_LOGI(decision,"Turning on water pump (timed activation)");
                        //Marca qual o timestamp que passou e subtrai de ativações restantes
                        for(int i = 0; i < qtdAtivacao; i++){
                            if(condicaoAtivacao[i]){
                                timestampMarker[i] = 1;
                                --contaAtivacaoRestante;
                            }
                        }

                        //Altera a variável para ligado
                        bombaDagua = BOMBA_LIGADA;

                        //Salva o tempo da última ativação
                        ultimaAtivacao = timeinfo;
                                
                        //Com timer
                        comTimer = 1;
                                
                        //Reseta variável de ativação
                        ativou = false;

                        //Atualiza o duty cycle e ativa a bomba
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_ON));
                        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    }

                    break;
                case BOMBA_LIGADA:
                    ESP_LOGI(decision,"Water pump ON");
                        
                    //Se a bomba for ativada sem timer
                    if(comTimer == 0){
                        //Calcula a diferença de tempo
                        timeDiff = tmDifferenceInMinutes(timeinfo, ultimaAtivacao);

                        //Se tiver passado do tempo máximo
                        if(timeDiff >= tempoAtivacaoProvisoria){
                            ESP_LOGI(decision,"Turning off water pump (emergency activation)");
                            //Altera a variável para desligado
                            bombaDagua = BOMBA_DESLIGADA;

                            //Atualiza info de quando ativou pela última vez
                            ultimaAtivacao = timeinfo;

                            //Soma o tempo que passou ativado
                            tempoAtivacaoQuePassou += timeDiff;

                            //Recalcula o tempo para ativar baseado no que já passou
                            if(contaAtivacaoRestante > 0){
                                tempoAtivaçãoPorPeriodo = (tempoAtivaçãoTotal - tempoAtivacaoQuePassou)/contaAtivacaoRestante;
                            }else{
                                tempoAtivaçãoPorPeriodo = 0;
                            }

                            //Atualiza o duty cycle e desliga a bomba
                            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_OFF));
                            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                        }

                    }else{
                
                        //Calcula a diferença de tempo
                        timeDiff = tmDifferenceInMinutes(timeinfo, ultimaAtivacao);

                        ESP_LOGI(decision,"Turning off water pump (timed activation)");
                        //Compara a diferença de tempo com o tempo de ativação por período
                        if(timeDiff >= tempoAtivaçãoPorPeriodo){
                                    
                            //Altera a variável para desligado
                            bombaDagua = BOMBA_DESLIGADA;

                            //Atualiza info de quando ativou pela última vez
                            ultimaAtivacao = timeinfo;

                            //Soma o tempo que passou ativado
                            tempoAtivacaoQuePassou += timeDiff;

                            //Recalcula o tempo para ativar baseado no que já passou
                            if(contaAtivacaoRestante > 0){
                                tempoAtivaçãoPorPeriodo = (tempoAtivaçãoTotal - tempoAtivacaoQuePassou)/contaAtivacaoRestante;
                            }else{
                                tempoAtivaçãoPorPeriodo = 0;
                            }

                            //Atualiza o duty cycle e desliga a bomba
                            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_OFF));
                            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                        }
                    }
                        
                break;
            }
            
            //Pegando o tempo atual
            time(&tempoAgora);
            setenv("TZ", "UTC+3", 1);
            tzset();
            localtime_r(&tempoAgora, &timeinfo);
            strftime(timeString, sizeof(timeString), "%c", &timeinfo);
            strftime(timeStringUltimaAtivacao, sizeof(timeStringUltimaAtivacao), "%c", &ultimaAtivacao);
            
            vSaveConfFile();

            xSemaphoreGive(dallyControlValuesSemaphore);
        }
        
        //Criando mensagem para as filas
        ESP_LOGI(decision, "Creating queue Bomb status message and adding to queue");
        sprintf(message_WPM,"{\"DataHora\": \"%s\", \"Estado_Bomba\": %d, \"Ultima_Ativacao\": \"%s\", \"Quanto_tempo_ficou_ligado(Min)\": %d}", timeString , bombaDagua, timeStringUltimaAtivacao, tempoAtivacaoQuePassou);
        
        ESP_LOGI(decision,"%s",message_WPM);
        //Enviando mensagem para a fila 
        xQueueSend(funcionamento_WPM_queue_handle, message_WPM, t500ms);
        vTaskDelay(t300s);
    }
}

//_______TAREFA_QUE_FAZ_O_RESET_DIARIO_DO_SISTEMA_______
void vDallyReset( void * pvParameters ){

    ESP_LOGI(sensorValues,"Iniciando tarefa de reset diario");
    
    //Variáveis para salvar os valores do RTC
    time_t tempoAgora;
    struct tm timeinfo;
    char timeString[64];

    //Pegando o tempo atual
    time(&tempoAgora);
    setenv("TZ", "UTC+3", 1);
    tzset();
    localtime_r(&tempoAgora, &timeinfo);

    while(1){

        //Pegando o tempo atual
        time(&tempoAgora);
        setenv("TZ", "UTC+3", 1);
        tzset();
        localtime_r(&tempoAgora, &timeinfo);

        //Se o próximo dia for igual ao dia atual, significa que o dia passou
        if(xSemaphoreTake(dallyControlValuesSemaphore, t1s) == pdTRUE){

            if(nextDay == timeinfo.tm_wday){
                //Define qual dia será o próximo dia
                ESP_LOGI(SNTP, "Updating next day condition");
                if(timeinfo.tm_wday == 6){
                    nextDay = 0;
                }else{
                    nextDay = timeinfo.tm_wday + 1;
                }

                //Carrega os valores padrões de controle do sistema
                ESP_LOGI(SNTP, "Reseting dally values");
                contaAtivacaoRestante = qtdAtivacao;
                tempoAtivacaoQuePassou = 0;
                tempoAtivaçãoPorPeriodo = tempoAtivaçãoTotal/contaAtivacaoRestante;
                for(int i = 0; i < qtdAtivacao; i++){
                    timestampMarker[i] = 0;
                }
            }

            xSemaphoreGive(dallyControlValuesSemaphore);
        }

        vTaskDelay(t60s);

    }

}

//_______TAREFA_QUE_FAZ_A_LEITURA_DOS_SENSORES_______
void vSensorValues( void * pvParameters ){
    
    ESP_LOGI(sensorValues,"Iniciando tarefa de leitura dos Sensores");
    //Variáveis para tentar atualizar o relógio
    int retry = 0;
    const int retry_count = 15;

    // Variáveis para salvar os valores do RTC
    time_t tempoAgora;
    struct tm timeinfo;
    char timeString[64];

    //Variáveis para processamente dos valores recebidos pelo ADC1 e ADC2
    static int ldr_raw[4];  //LDR - Vetor do valor de entrada
    static int ldr_vol[4];  //LDR - Vetor do valor da tensão
    static float ldr_res[4];  //LDR - Vetor do valor da resistencia

    static int thr_raw[4];  //THERMISTOR - Vetor do valor de entrada
    static int thr_vol[4];  //THERMISTOR - Vetor do valor da tensão
    static float thr_res[4];  //THERMISTOR - Vetor do valor da resistencia

    static int shr_raw; //SOIL HYGROMETER - Valor de entrada
    static int shr_vol; //SOIL HYGROMETER - Valor da tensão
    static float shr_res; //SOIL HYGROMETER - Valor da resistencia

    char message_LDR[messageSize];   //Mensagem para os sensores LDRs
    char message_THR[messageSize];   //Mensagem para os sensores THERMISTORs
    char message_SHR[messageSize];   //Mensagem para o sensor SOIL HYGROMETER
    char message_UTS[messageSize];   //Mensagem para o sensor ULTRASOUND

    uint32_t tof_ticks; //Variável necessária para receber a notificação do sensor ultrassom

    //Chamando a função que inicia o MCPWM
    mcpwm_init_sta();

    while(1){
        
        // Tenta atualizar o relógio interno
        retry = 0;
        while (esp_netif_sntp_sync_wait(t500ms) == ESP_ERR_TIMEOUT && ++retry < retry_count) {
            ESP_LOGI(sensorValues,"Waiting for system time to be set... (%d/%d)\n", retry, retry_count);
        }

        //_________________LEITURA_________________
        //Leitura dos sensores para os LDRs (utilizam o ADC1)
        ESP_LOGI(sensorValues,"Reading oneshot ADC1 values");
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, LDR_N, &ldr_raw[0]));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, LDR_O, &ldr_raw[1]));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, LDR_L, &ldr_raw[2]));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, LDR_S, &ldr_raw[3]));

        // Para o Wifi para conseguir ler os sensores (o ADC2 utiliza o mesmo circuito que oc Wifi)
        ESP_LOGI(sensorValues,"Stopping Wifi, before reading ADC2 values");
        ESP_ERROR_CHECK(esp_wifi_stop());
        
        //Leitura dos sensores para os THERMISTORs (utilizam o ADC2)
        ESP_LOGI(sensorValues,"Reading oneshot ADC2 values");
        ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, THERMISTOR_N, &thr_raw[0]));
        ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, THERMISTOR_O, &thr_raw[1]));
        ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, THERMISTOR_L, &thr_raw[2]));
        ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, THERMISTOR_S, &thr_raw[3]));

        //Leitura do sensor para o Soil Hygrometer (utiliza o ADC2)
        ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, SOIL_HYGROMETER, &shr_raw));

        //Inicia o Wifi novamente (acabou a leitura dos sensores que utilizam o ADC2)
        ESP_LOGI(sensorValues,"Starting Wifi, to after ADC2 read");
        ESP_ERROR_CHECK(esp_wifi_start()); 

        //_________________CALIBRAÇÃO_________________
        //Pega o valor aproximado de tensão para os LDRs (usa ADC1)
        ESP_LOGI(sensorValues,"Using calibration scheme to convert raw value into voltage");
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, ldr_raw[0], &ldr_vol[0]));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, ldr_raw[1], &ldr_vol[1]));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, ldr_raw[2], &ldr_vol[2]));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, ldr_raw[3], &ldr_vol[3]));

        //Pega o valor aproximado de tensão para os THERMISTORs (usa ADC2)
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_handle, thr_raw[0], &thr_vol[0]));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_handle, thr_raw[1], &thr_vol[1]));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_handle, thr_raw[2], &thr_vol[2]));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_handle, thr_raw[3], &thr_vol[3]));

        //Pega o valor aproximado de tensão para o Soil Hygrometer (usa ADC2)
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_handle, shr_raw, &shr_vol));

        //_________________CONVERSÃO_________________
        //Convertendo os valores de tensão para resistência do LDR
        ESP_LOGI(sensorValues,"Converting voltage values into resistence values");
        ldr_res[0] = ldr_vol[0]*resistorCircuitoLDR/(tensaoAlimentacao - ldr_vol[0]);
        ldr_res[1] = ldr_vol[1]*resistorCircuitoLDR/(tensaoAlimentacao - ldr_vol[1]);
        ldr_res[2] = ldr_vol[2]*resistorCircuitoLDR/(tensaoAlimentacao - ldr_vol[2]);
        ldr_res[3] = ldr_vol[3]*resistorCircuitoLDR/(tensaoAlimentacao - ldr_vol[3]);

        //Convertendo os valores de tensão para resistência do THERMISTOR
        thr_res[0] = thr_vol[0]*resistorCircuitoTHR/(tensaoAlimentacao - thr_vol[0]);
        thr_res[1] = thr_vol[1]*resistorCircuitoTHR/(tensaoAlimentacao - thr_vol[1]);
        thr_res[2] = thr_vol[2]*resistorCircuitoTHR/(tensaoAlimentacao - thr_vol[2]);
        thr_res[3] = thr_vol[3]*resistorCircuitoTHR/(tensaoAlimentacao - thr_vol[3]);

        //Convertendo os valores de tensão para resistência Soil Hygrometer
        shr_res = shr_vol*resistorCircuitoSHR/(tensaoAlimentacao - shr_vol);

        //_________________VALOR_FINAL_________________
        // Encapsula as variáveis pois elas podem ser utilizadas em outras funções
        if(xSemaphoreTake(sensorValuesSemaphore, t1s) == pdTRUE){
            //Convertendo os valores de resistencia para Lux
            ESP_LOGI(sensorValues,"Converting resistence values into respective sensor units");
            ldr_lux[0] = lux*pow(resistenciaParaValorLuxN/ldr_res[0],1/ldrGamma);
            ldr_lux[1] = lux*pow(resistenciaParaValorLuxO/ldr_res[1],1/ldrGamma);
            ldr_lux[2] = lux*pow(resistenciaParaValorLuxL/ldr_res[2],1/ldrGamma);
            ldr_lux[3] = lux*pow(resistenciaParaValorLuxS/ldr_res[3],1/ldrGamma);

            //Convertendo os valores de resistencia para Graus Celsius
            thr_tem[0] = (thrB*kelvin)/(kelvin*log(thr_res[0]/resistenciaParaValorKelvin)+thrB) - kelvinCelsius;
            thr_tem[1] = (thrB*kelvin)/(kelvin*log(thr_res[1]/resistenciaParaValorKelvin)+thrB) - kelvinCelsius;
            thr_tem[2] = (thrB*kelvin)/(kelvin*log(thr_res[2]/resistenciaParaValorKelvin)+thrB) - kelvinCelsius;
            thr_tem[3] = (thrB*kelvin)/(kelvin*log(thr_res[3]/resistenciaParaValorKelvin)+thrB) - kelvinCelsius;

            //Convertendo os valores de resistencia para Humidade
            shr_pct = (shr_res-minResistenceSHR)/(maxResistenceSHR - minResistenceSHR);

            //Gerando trigger para o sensor ultrassom
            ESP_LOGI(sensorValues,"Ativando trigger para sensor de distancia");
            gen_trig_output();

            //Aguardando resposta do sensor de distancia
            if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks, pdMS_TO_TICKS(1000)) == pdTRUE) {
                float pulse_width_us = tof_ticks * (1000000.0 / esp_clk_apb_freq());
                if (pulse_width_us > 35000) {
                    // out of range
                    continue;
                }
                //Converde tamanho do pulso em distancia medida em cm
                uts_dis = (float) pulse_width_us / 58;

                //Pegando o tempo atual
                time(&tempoAgora);
                setenv("TZ", "UTC+3", 1);
                tzset();
                localtime_r(&tempoAgora, &timeinfo);
                strftime(timeString, sizeof(timeString), "%c", &timeinfo);

                //Criando mensagem para as filas
                ESP_LOGI(sensorValues, "Creating sensor values queue messages and adding to queue");
                sprintf(message_LDR,"{\"DataHora\": \"%s\", \"LDR_N(Lux)\": %f, \"LDR_O(Lux)\": %f, \"LDR_L(Lux)\": %f, \"LDR_S(Lux)\": %f}", timeString , ldr_lux[0], ldr_lux[1], ldr_lux[2], ldr_lux[3]);
                sprintf(message_THR,"{\"DataHora\": \"%s\", \"Thermistor_N(°C)\": %f, \"THERMISTOR_O(°C)\": %f, \"THERMISTOR_L(°C)\": %f, \"THERMISTOR_S(°C)\": %f}", timeString , thr_tem[0], thr_tem[1], thr_tem[2], thr_tem[3]);
                sprintf(message_SHR,"{\"DataHora\": \"%s\", \"Soil Hygrometer(percentage)\": %f}", timeString , shr_pct);
                sprintf(message_UTS,"{\"DataHora\": \"%s\", \"Ultrasound(cm)\": %f}", timeString , uts_dis);

                ESP_LOGI(sensorValues,"%s",message_LDR);
                ESP_LOGI(sensorValues,"%s",message_THR);
                ESP_LOGI(sensorValues,"%s",message_SHR);
                ESP_LOGI(sensorValues,"%s",message_UTS);

                //Adicionando mensagens nas suas respectivas filas
                xQueueSend(sensor_LDR_queue_handle, message_LDR, t500ms);
                xQueueSend(sensor_THR_queue_handle, message_THR, t500ms);
                xQueueSend(sensor_SHR_queue_handle, message_SHR, t500ms);
                xQueueSend(sensor_UTS_queue_handle, message_UTS, t500ms);
            }
             //Devolve o semaforo
            xSemaphoreGive(sensorValuesSemaphore);
        }

        vTaskDelay(t300s);
    }
}

//_______TAREFA_QUE_ENVIA_OS_DADOS_POR_MQTT_______
void vProcessosMqtt( void * pvParameters ){
    
    char messageBuffer[messageSize]; //Variável para retirar os valores da fila e envia-los por MQTT
    FILE *fData; //Variável que recebe o arquivo de dados
    uint8_t contagemTentativas = 0; //Conta a quantidade de tentativas para enviar as mensagem pelo MQTT 
    while(1){

        //Verifica se a conexão com o MQTT está disponível
        //ESP_LOGI(processosMqtt, "Verifying if MQTT5 connection is available");
        if(xSemaphoreTake(mqttAvailable, t1s)==pdTRUE){
            ESP_LOGI(processosMqtt, "Connection is available");

            //Devolve semaforo após verificação
            xSemaphoreGive(mqttAvailable);
            if(xSemaphoreTake(dataFileSemaphore, t1s)){

                ESP_LOGI(processosMqtt,"Opening data file...");
                //Variável para leitura do arquivo
                fData = fopen(dataFileName, "r");
                //Se ele conseguir abrir o arquivo
                if(fData != NULL){

                    //Enquanto ainda houver linhas
                    while(fgets(messageBuffer, messageSize, fData)){
                        ESP_LOGI(processosMqtt,"TAG: %s",messageBuffer);
                        if(strcmp(messageBuffer,"LDR\n") == 0){
                            //Se a mensagem tiver a tag de LDR
                            fgets(messageBuffer, messageSize, fData);
                            esp_mqtt_client_publish(client, topicoLDR, messageBuffer, 0, 2, 1);
                        }else if(strcmp(messageBuffer,"THR\n") == 0){
                            //Se a mensagem tiver a tag de THR
                            fgets(messageBuffer, messageSize, fData);
                            esp_mqtt_client_publish(client, topicoTHR, messageBuffer, 0, 2, 1);
                        }else if(strcmp(messageBuffer,"SHR\n") == 0){
                            //Se a mensagem tiver a tag de SHR
                            fgets(messageBuffer, messageSize, fData);
                            esp_mqtt_client_publish(client, topicoSHR, messageBuffer, 0, 2, 1);
                        }else if(strcmp(messageBuffer,"UTS\n") == 0){
                            //Se a mensagem tiver a tag de UTS
                            fgets(messageBuffer, messageSize, fData);
                            esp_mqtt_client_publish(client, topicoUTS, messageBuffer, 0, 2, 1);
                        }else if(strcmp(messageBuffer,"WPM\n") == 0){
                            //Se a mensagem tiver a tag de WPM
                            fgets(messageBuffer, messageSize, fData);
                            esp_mqtt_client_publish(client, topicoWPM, messageBuffer, 0, 2, 1);
                        }

                        //Aguarda 1 segundo para não bugar o envio
                        vTaskDelay(t500ms);
                    }
                    ESP_LOGI(processosMqtt,"All file lines sent! Clearing file...");
                    freopen(dataFileName, "w", fData);
                    if(fData != NULL){
                        fclose(fData);
                        ESP_LOGI(processosMqtt,"File cleared!");
                    }else{
                        ESP_LOGE(processosMqtt, "Failed to clear file!");
                    }
                }else{
                    ESP_LOGE(processosMqtt,"Could not open data file");
                }
                xSemaphoreGive(dataFileSemaphore);
            }
            ESP_LOGI(processosMqtt, "Reading queue and sending messages");
            //Envia para o tópico do LDR
            if(xQueueReceive(sensor_LDR_queue_handle, messageBuffer, t1s)){
                esp_mqtt_client_publish(client, topicoLDR, messageBuffer, 0, 2, 1);
            }

            //Envia para o tópico do Thermistor
            if(xQueueReceive(sensor_THR_queue_handle, messageBuffer, t1s)){
                esp_mqtt_client_publish(client, topicoTHR, messageBuffer, 0, 2, 1);
            }

            //Envia para o tópico do Soil Hygrometer
            if(xQueueReceive(sensor_SHR_queue_handle, messageBuffer, t1s)){
                esp_mqtt_client_publish(client, topicoSHR, messageBuffer, 0, 2, 1);
            }

            //Envia para o tópico do Ultrassound
            if(xQueueReceive(sensor_UTS_queue_handle, messageBuffer, t1s)){
                esp_mqtt_client_publish(client, topicoUTS, messageBuffer, 0, 2, 1);
            }

            //Envia para o tópico da Bomba d'agua
            if(xQueueReceive(funcionamento_WPM_queue_handle, messageBuffer, t1s)){
                esp_mqtt_client_publish(client, topicoWPM, messageBuffer, 0, 2, 1);
            }

            contagemTentativas = 0;
        }else{
            //Informa que não foi possível se conectar ao MQTT
            ESP_LOGI(processosMqtt, "Connection not available at the moment");
            ++contagemTentativas;
            if(contagemTentativas == maxMqttMaxTryout){
                ESP_LOGI(processosMqtt,"Granting permission to write file with data...");
                contagemTentativas = 0;
                xSemaphoreGive(writeDataFile);
            }
        }
        //Faz a tarefa esperar por 10 segundos
        vTaskDelay(t60s);
    }
}