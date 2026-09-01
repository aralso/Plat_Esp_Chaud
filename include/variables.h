#ifndef VARIABLES_H

#define VARIABLES_H

#include <stddef.h>  // for size_t
#include <Arduino.h>  // for IPAddress, String types

// variables externes

#define Graph_Specifique

#define ESP_TJ_ACTIF     // Rôle principal 


// Hardware
//#define MODE_WT32  // WT32-Eth01 sinon ESP32-CAM ou DOIT ESP32 Devkit V1

//#define DEBUG  // mode station, pas de websocket, pas de sécurite, emulation valeurs STM32
//#define ESP32_v1    // DOIT ESP32 DEVKIt V1

#ifdef ESP_VEILLE
  //#define ESP32_Fire2
  #define ESP32_uPesy
  //#define Temp_int_HDC1080  // Capteur I2C HDC1080
  #define MODE_Wifi  // Wifi sinon Ethernet
  //#define Sans_securite
  #define Sans_websocket
  #define OTA
  #define ENVOI
  #define STOCKAGE

#endif

#ifdef ESP_TJ_ACTIF  // Serveur
  #define ESP32_S3
  #define SDCARD   // Carte Micro SD
  #define OTA

  //#define ESP32_Fire2
  #define MODE_Wifi  // Wifi sinon Ethernet
  //#define Sans_websocket
  //#define Sans_securite
  //#define WatchDog
#endif

#define BTN_COUNT 1  // Nombre de boutons

//#define Temp_int_DHT22
//#define Temp_int_DS18B20

// Réseau
//#define NO_RESEAU

//#define Wifi_AP    // AP sinon STA

//#define STM32  //incompatible du modbus, sauf à changer les pin
// #define OTA


// Definir le canal WIFI ici (doit correspondre au routeur pour la Chaudière)
// ⚠️ IMPORTANT : Ce canal DOIT correspondre au canal de votre routeur WiFi
// "garches" Pour le trouver : regardez les logs de la chaudière au démarrage

// Adresse par défaut de la chaudière (B0:CB:D8:E9:0C:74)
const uint8_t MAC_SERVEUR[] = {0xB0, 0xCB, 0xD8, 0xE9, 0x0C, 0x74};


#define MAX_PAYLOAD 200
#define NB_VAL_TAB 12   // taille max autorisée pour la reception des temp/humid
#define SERVER_ADD 'H'

typedef struct __attribute__((packed)) {   // packed permet d'éviter les octets de padding ajoutés par le compilateur
    uint8_t destinataire;
    uint8_t emetteur;
    uint8_t longueur;
    uint8_t code;
    uint8_t code2;
    uint8_t num_seq;  // pour renvoi Ack
    uint8_t payload[MAX_PAYLOAD];
} Message_EspNow;

// Structure pour la queue de réception ESP-NOW (message + adresse source)
typedef struct {
  uint8_t src_addr[6];
  Message_EspNow msg;
  int len;
} EspNowRecvMsg_t;



// structure des paramètres 
typedef enum ParamType {
  U8,
  U16,
  IP,
  STR,
  U32
} ParamType;

typedef struct Param {
  const char* key;
  uint8_t order;
  ParamType type;
 
  uint32_t min16;   // numeric lower bound (used for U8/U16/U32)
  uint32_t max16;   // numeric upper bound
 
  uint32_t def_u16; // default numeric value (fits U8/U16/U32)
  uint8_t rtc_valid;  // 0: not valid, 1: valid
  const char* def_str;
  void* var;
  uint8_t size;      // taille du buffer (0 pour U8/U16)
} Param;

// Forward declarations for variables used in PARAMS
extern uint8_t log_detail;
extern uint8_t mode_reseau;
extern uint16_t nb_reset;
extern  uint8_t mode_rapide;
extern  uint8_t periode_cycle;
extern uint8_t DelaiWebsocket;
extern  uint8_t skip_graph;
extern uint16_t Seuil_batt_sonde;
extern  uint8_t Nb_jours_Batt_log;
extern uint8_t pas_de_veille;
extern  uint16_t prolong_veille;
extern  uint8_t action_stockage;
extern  uint8_t action_envoi;
extern uint8_t boot_rapide;
extern char latitude[];
extern char longitude[];
extern  uint8_t last_wifi_channel;
extern uint8_t WIFI_CHANNEL;
extern uint8_t local_ip[4];
extern uint8_t gateway[4];
extern uint8_t subnet[4];
extern uint8_t primaryDNS[4];
extern uint8_t secondaryDNS[4];
extern char mac_gw_str[20];

extern char nom_routeur[];
extern char mdp_routeur[];
extern uint8_t websocket_on;
extern char ip_websocket[];
extern uint8_t id_websocket;

// Current camera JPEG quality (camera sensor 'quality' value, e.g. 63..4)
extern uint8_t current_sensor_quality;

extern const size_t PARAMS_COUNT;
extern Param PARAMS[];


//  -------  CONFIGURATION DES PINS
//  -----------------------------------------------

/* PINOUT DOIT :
0: pour programmation
1: TX pour prog & debug
2: OUT2 : sortie PAC
3: RX pour prog & debug
4: OUT1 : sortie Pompe
5(pin4:markIO35): OUT3 : sortie Arret
12: Btn_reveil
14: SDA Eclairs
15: DSB1820 capteur temp piscine
17: SCL Eclairs
21-22 : SDA, SCL HDC1080
32: IN:capteur DHT22 N°2
33: IN:capteur DHT22 N°1
35:(pin17) (in only) IN : interruption éclairs
36: (in only) IN : capteur pression
39: (in only) IN
*/
/*#define BTN1 14  // Defaut secteur (pullup)
#define BTN2 12  // intrusion    (pullup)
#define BTN3 14  // autoprotection    (pullup)
#define BTN4 15  // marche/Arret    (no pull)  0V:arret 12V:marche
#define BNT5 16  // Reset pour Accesspoint*/

#ifdef MODE_WT32  // WT32_Eth01
  // const int PIN_Tint = 11;   // GPIO IN1 Temp interieure DS18B20
  const int PIN_Tint22 = 5;  // GPIO IN1 Temp interieure DHT22
  const int PIN_PAC = 4;     // GPIO OUT PAC PWM
  const int PIN_Text = 36;   //  Text:Entrée analogique 32 à 36 et 39
#else                      // ESP32_DevKit
  // const int PIN_Tint = 13;  Défini dans le fichier appli.ino
  const int PIN_Tint22 = 5;  // GPIO IN1 Temp interieure DHT22
  const int PIN_PAC = 4;     //  OUT PAC - PWM  40kOhm+100nF(Fc=40Hz) et PWM=40khz
#endif

// Pin Reveil
#ifdef ESP32_v1
  #define PIN_REVEIL 12  // Pin de réveil (Bouton externe)
#endif

#ifdef ESP32_S3
  #define PIN_REVEIL 12  // Pin de réveil (Bouton externe)
  #define PIN_Vbatt 10        // Pin Surveillance Batterie (LiPo/2)
  #define PIN_REVEIL2 11  // Pin d'entrée pour interruption (ex: detecteur)
  #define PIN_OUT0 4     // Power pour alimenter le capteur PIR : 10uA
  #define PIN_SDA 8
  #define PIN_SCL 9
  const int PIN_RXModbus = 16;  // s3:18  devkitv1:16 RO
  const int PIN_TXModbus = 17;  // s3:17  devkitv1:17 DI
  #define MAX485_RE_NEG 35  // S3:35
  #define MAX485_DE 36      // s3:36
  const int PIN_RXSTM = 18;  // RX STM32
  const int PIN_TXSTM = 17;  // TX STM32
#endif

#ifdef ESP32_Fire2    // Firebeetle
  #define PIN_REVEIL 4  // Pin de réveil (Bouton externe) PIN RTC : 0 à 7
#endif
#ifdef ESP32_uPesy
  #define PIN_REVEIL 34  // Pin de réveil (Bouton externe)
#endif

// Structure d'un message uart
#define MSG_SIZE 40
typedef struct {
  char message[MSG_SIZE];
  uint16_t length;
} UartMessage;

typedef struct {
  uint8_t Add_node;
  uint16_t nb_mess_recu;
  uint8_t actif;
  uint8_t mac_node[6];
  uint16_t last_update; // derniere reception de message en jours (1er:1/1/2026)
} S_Node;

typedef struct {
  uint16_t longueur;  // longueur
  char msg[MSG_SIZE];
} UartMessage_t;

float readBatteryVoltage();
void lectureHeure();
void requete_status(char* json_response, uint8_t socket, uint8_t type);
void recep_message1(UartMessage_t* messa);  // recept_uart1
void maj_etat_chaudiere_delai(uint8_t delai);
void modif_timer_cycle(void);
void traitement_rx(UartMessage_t* mess);
uint8_t requete_Get_appli(const char* var, float* valeur);
uint8_t requete_Set_appli(String param, float valf);
uint8_t requete_GetReg(int reg, float* valeur);
void enreg_24h( uint8_t veille);

template<typename T>
void payloadWrite(uint8_t* payload, uint8_t& pos, const T& value)
{
    memcpy(&payload[pos], &value, sizeof(T));
    pos += sizeof(T);
}

void passage_deep_sleep(uint64_t temps);

extern float Vbatt_Th;   // Tension batterie thermomètre
extern bool Vbatt_Th_I;  // indicateur de réception batt sonde


/* ESP32S3 : Serial0:Pin 42 et 43
 */

typedef enum {
  EVENT_NONE = 0,
  EVENT_INIT,
  EVENT_UART,
  EVENT_SENSOR,
  EVENT_GPIO_ON,
  EVENT_GPIO_OFF,
  EVENT_ERREUR,
  EVENT_ECOUTE_WebSock,
  EVENT_WATCHDOG,
  EVENT_24H,
  EVENT_3min,
  EVENT_CYCLE,
  EVENT_UART1,
  EVENT_ESP_RECV
} systeme_eve_type_t;

// Structure d'un événement tache sequenceur
typedef struct {
  systeme_eve_type_t type;  // Type d'événement
  uint32_t data;            // Donnée associée (ex: valeur capteur, byte UART)
} systeme_eve_t;


/* Codes erreur*/
#define Code_erreur_Tint 1
#define Code_erreur_Text 2
#define Code_erreur_Heure 3
#define Code_erreur_depass_tab_status 4
#define Code_erreur_queue_full 5
#define Code_erreur_Json 5
#define Code_erreur_queue 6
#define Code_erreur_google 7
#define Code_erreur_http_local 8
#define Code_erreur_wifi 9
#define Code_erreur_esp_now 10


constexpr int NB_Val_Graph = 99;
constexpr int NB_Graphique = 10;  // Example value, replace with the actual number of graphics

#define MAX_DUMP 6900              // 600 + 1050 car par graphique

extern char buffer_dmp[MAX_DUMP];  // max 250 logs, 16 octets chacun
extern int BTN_PIN[];  // Pins des boutons

extern  uint8_t esp_now_actif;  // 0:esp_now inactif  1:actif

extern uint8_t protocole;
extern QueueHandle_t eventQueue;  // File d'attente des événements sequenceur
extern QueueHandle_t QueueEspNow;  // File d'attente messages ESP-NOW reçus
extern uint8_t nb_capt_sdcard;     // nombre de capteurs trouvés sur SD
extern uint8_t strat_actif;        // stratégie d'affichage active (indice 0..NB_STRAT_CAPT-1)
void requete_GetStrat(uint8_t strat, char* buf, size_t size);
void requete_SetStrat(uint8_t strat, uint8_t* caps, uint8_t* vals, uint8_t nb);
extern uint16_t erreur_queue;
extern TimerHandle_t debounceTimer;
extern TimerHandle_t xTimer_activ_chaud;
extern float Tint, Text, Humid;


extern uint16_t compteur_detection;
extern uint16_t Nb_PI[];

extern float Tint, Text, Humid;

extern uint8_t cpt_securite;
extern uint8_t WIFI_CHANNEL;
extern uint8_t rtc_valid;  // 0:cold reset  1:reset apres deep sleep
extern  uint16_t  cpt_cycle_batt;                   // Compteur cycles pour mesure batterie
extern volatile uint8_t ackReceived;  // global pour indiquer que le peer a acké
extern volatile int ackChannel;       // canal où ça a marché
extern uint8_t init_time;
extern float heure;

extern unsigned long last_remote_Tint_time, last_remote_Text_time,
    last_remote_heure_time;
extern  uint16_t err_Tint, err_Text, err_Heure;

extern  float tempI_moy24h, tempE_moy24h, Hum_24h, HA_moy24h;

extern int16_t graphique[NB_Val_Graph][NB_Graphique];

extern uint16_t Seuil_batt_arret_ESP;  // millivolt
extern uint8_t type_reveil;  //0:pas de reveil 1: réveil par timer, 2: réveil par bouton_reveil 3:reveil par PIR

extern uint8_t Cons_eco;
extern TimerHandle_t xTimer_cycle_chaud;
extern uint8_t compteur_graph;

extern  uint8_t etat_now;
extern uint8_t mac_gw[];   // B0:CB:D8:E9:0C:74  adresse mac esp_dest

extern unsigned long wake_up_time;  // Temps de réveil/dernière activité

void writeLog(uint8_t code, uint8_t c1, uint8_t c2, uint8_t c3,
              const char* message);
void debounceCallback(TimerHandle_t xTimer);
uint16_t crc16_arc(const uint8_t* data, size_t length);
void log_erreur(uint8_t code, uint8_t valeur,
                uint8_t val2);  // Code:1:Tint, 2:Text, 3:TPac;
void init_10_secondes();
void setup_0();
void setup_nvs();
void setup_1();
void setup_2();
void setup_3();
uint8_t requete_action_appli(const char* reg, const char* data);
void appli_event_on(systeme_eve_t evt);
void appli_event_off(systeme_eve_t evt);
void traitement_espnow_recv(EspNowRecvMsg_t &recv);
uint8_t requete_Get_appli(String var, float* valeur);
uint8_t requete_Set_appli(String param, int val);
uint8_t requete_GetReg_appli(int reg, float* valeur);
uint8_t requete_SetReg_appli(int param, float valeurf);
uint8_t requete_Get_String_appli(uint8_t type, String var, char* valeur);
uint8_t requete_Set_String_appli(int param, const char* texte);
uint8_t lecture_Tint(float* mesure, float* humid);
uint8_t lecture_Text(float* mesure);
void event_cycle();

// Fonctions WiFi
uint8_t connectWiFiWithDiagnostic();
uint8_t connectWiFiRapide();
void diagnoseWiFiError();
void protectUARTDuringWiFi();

// Configuration DHT22
#define DHT22_TIMEOUT_MS 5000       // Timeout de lecture DHT22 en millisecondes
#define DHT22_MIN_INTERVAL_MS 2000  // Intervalle minimum entre lectures DHT22

#endif