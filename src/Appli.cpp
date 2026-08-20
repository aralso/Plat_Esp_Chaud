
#include <Arduino.h>
#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "variables.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Preferences.h>  // pour nvs eeprom
#include <PID_v1.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

extern WiFiClient client;
extern Preferences preferences_nvs;  // Déclaration externe


uint8_t  WIFI_CHANNEL;
RTC_DATA_ATTR uint8_t etat_now;
uint16_t Seuil_batt_sonde;  // millivolt
uint8_t Nb_jours_Batt_log;

int16_t batt_sonde[NB_CAPT][20];  // valeur batterie sonde remode

RTC_DATA_ATTR uint8_t compteur_graph;
RTC_DATA_ATTR uint16_t compteur_24h;

S_Node Node[NB_CAPT];

RTC_DATA_ATTR uint8_t mac_gw[6];   // B0:CB:D8:E9:0C:74  adresse mac esp_dest
volatile uint8_t ackReceived = false;  // global pour indiquer que le peer a acké
volatile int ackChannel = -1;       // canal où ça a marché

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
//void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len);
//void OnDataRecv(const esp_now_peer_info_t * info, const uint8_t *incomingData, int len);

#if defined(ARDUINO_ARCH_ESP32) && defined(WIFI_TX_INFO_T)
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);
#else
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);
#endif

uint8_t parseMacString(const char* str, uint8_t mac[6]);



#ifdef Temp_int_DS18B20
  OneWireNg_CurrentPlatform ow(PIN_DS18B20, false);
  OneWireNg_DS18B20 sensor(&ow);

  // Capteur temperature Dallas DS18B20  Temperature intérieure
  typedef uint8_t DeviceAddress[8];
  const int PIN_Tint = 13;      // Tint:Entrée onewire GPIO DS18B20
  //OneWire oneWire(PIN_Tint);
  //DallasTemperature ds(&oneWire);
  int nb_capteurs_temp = 1;  //DS18B20
  DeviceAddress Thermometer[5];
  DeviceAddress adds;
#endif

DHT dht[] = {
  { PIN_Tint22, DHT22 },
};
#ifdef Temp_int_HDC1080
  ClosedCube_HDC1080 hdc1080;
#endif

// Temperature intérieure
float Tint;
float Text;
float Humid;

uint16_t err_Tint, err_Text, err_Heure;  // compteurs d'erreurs




#ifdef Temp_int_DS18B20
  void printAddress(DeviceAddress deviceAddress) {
    for (uint8_t i = 0; i < 8; i++) {
      Serial.print("0x");
      if (deviceAddress[i] < 0x10) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
      if (i < 7) Serial.print(", ");
    }
    Serial.println("");
  }
#endif

int readLastLogsG(int nombre);


void init_10_secondes()
{
}

//setup au debut
void setup_0()
{

  /*if (NB_Graphique==6)
  {
    graphique[0][0] = 180;  //Tint - vert
    graphique[1][0] = 185;
    graphique[2][0] = 190;

    graphique[0][1] = 110;  // Text - bleu
    graphique[1][1] = 80;
    graphique[2][1] = 103;
    graphique[3][1] = 95;

    graphique[0][2] = 150;  // Chaud
    graphique[1][2] = 150;
    graphique[2][2] = 200;
    graphique[3][2] = 200;

    graphique[0][3] = 185;  // Tint moy
    graphique[1][3] = 183;
    graphique[2][3] = 183;
    graphique[3][3] = 195;

    graphique[0][4] = 35;   // Text moy
    graphique[1][4] = 38;
    graphique[2][4] = 42;
    graphique[3][4] = 32;

    graphique[0][5] = 50;  // cout
    graphique[1][5] = 55;
    graphique[2][5] = 48;  
    graphique[3][5] = 52;
  }*/
}


// setup apres la lecture nvs, avant démarrage reseau
void setup_1()
{
  // initialisation capteur de température intérieur
  #ifdef ESP_THERMOMETRE
    Tint = 15;
    #ifdef Temp_int_DHT22
      dht[0].begin();
    #endif
 
    #ifdef Temp_int_HDC1080

      #ifdef ESP32_v1
        Wire.begin(21, 22); // Forçage des pins SDA=21, SCL=22 pour ESP32 DevKit V1
      #endif
      #ifdef ESP32_Fire2
        Wire.begin(19, 20); // Forçage des pins SDA=20, SCL=21 pour ESP32 Firebeetle 2
      #endif
      #ifdef ESP32_uPesy
        Wire.begin(21, 22); // Forçage des pins SDA=21, SCL=22 pour ESP32 uPesy
      #endif

      hdc1080.begin(0x40);
      /*if (i2cDevicePresent(0x40)) {
        Serial.println("HDC1080 détecté");
        hdc1080.begin(0x40);
      } else {
        Serial.println("HDC1080 ABSENT");
      }*/
    #endif

 
    #ifdef Temp_int_DS18B20
      ds.begin();  // Startup librairie DS18B20
      nb_capteurs_temp = ds.getDeviceCount();
      Serial.print("Nb Capteurs DS18B20:");
      Serial.println(nb_capteurs_temp);
      if (nb_capteurs_temp > 1) nb_capteurs_temp = 1;
      int j;
      for (j = 0; j < nb_capteurs_temp; j++) {
        Serial.print(" Capteur :");
        ds.getAddress(Thermometer[j], j);
        printAddress(Thermometer[j]);
      }
    #endif

    // lecture initiale temperature interieure
    uint8_t Tint_err = lecture_Tint(&Tint);
    if ((Tint < 1) || (Tint > 45)) {
      Tint = 20.0;
      Tint_err = 7;
    }
    if (Tint_err) log_erreur(Code_erreur_Tint, Tint_err, 1);
    else
      Serial.printf("Temp int:%.2f\n\r", Tint);
  #endif
}

// apres demarrage reseau
void setup_2()
{
  #ifdef ESP_TJ_ACTIF

    readLastLogsG(99);

    // Configuration WiFi en mode Station pour ESP-NOW

    if ((mode_reseau==13) )
      WiFi.mode(WIFI_STA);
    
    // 🔍 DIAGNOSTIC: Forcer le canal WiFi
    uint8_t current_channel;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&current_channel, &second);
    if (log_detail>=4) Serial.printf("Canal WiFi AVANT config ESP-NOW: %d\n", current_channel);
    
    // Forcer le canal si nécessaire (doit correspondre au routeur)
    // esp_wifi_set_promiscuous(true);
    // esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    // esp_wifi_set_promiscuous(false);
    
    if (esp_now_init() != ESP_OK) {
      Serial.println("Erreur initialisation ESP-NOW");
      return;
    }
    esp_now_register_recv_cb(OnDataRecv);
    
    // Vérifier le canal après init
    esp_wifi_get_channel(&current_channel, &second);
    
    if (log_detail>=4)
    {
      Serial.println("\n\n======================================");
      Serial.println("🔵 ESP-NOW Initialisé (RÉCEPTEUR)");
      Serial.printf("   Canal WiFi: %d\n", current_channel);
      delay(2000); // 2 secondes de pause pour lire
    }
    //if ((mode_reseau==13) )
    //else
    //  Serial.println(WiFi.softAPmacAddress());

    String macStr = WiFi.macAddress();
    sscanf(macStr.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
          &mac_gw[0], &mac_gw[1], &mac_gw[2],
          &mac_gw[3], &mac_gw[4], &mac_gw[5]);
    if (log_detail>=3) {
      Serial.printf("   MAC : %02X:%02X:%02X:%02X:%02X:%02X\n",
            mac_gw[0], mac_gw[1], mac_gw[2],
            mac_gw[3], mac_gw[4], mac_gw[5]);

      Serial.printf("   Canal WiFi: %d\n", current_channel);
      Serial.println("   En attente de messages...");
      Serial.println("======================================\n\n");
      delay(2000); // 2 secondes de pause pour lire
    }
  #endif
}



void appli_event_on(systeme_eve_t evt)
{
}

void appli_event_off(systeme_eve_t evt)
{
}

// type 1
uint8_t requete_Get_appli(const char* var, float *valeur)
  //uint8_t requete_Get_appli (String var, float *valeur) 
{
  uint8_t res=1;

  if (strncmp(var, "Tint",5) == 0) {
    res = 0;
    *valeur = Tint;
  }
  if (strncmp(var, "Text",5) == 0) {
    res = 0;
    *valeur = Text;
  }
  if (strncmp(var, "codeR_pac",10) == 0) {
    res = 0;
    if (cpt_securite)  *valeur=1;
    else *valeur=0;
  }


  return res;
}



// type 1
uint8_t requete_Set_appli (String param, float valf) 
{
  uint8_t res=1;
  int8_t val = round(valf);


  return res;
}

// type 2
uint8_t requete_GetReg_appli(int reg, float *valeur)
{
  uint8_t res=1;

  
  if (reg == 41)  // registre 41 : canal WiFi actuel (dynamic)
  {
    res = 0;
    uint8_t current_channel;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&current_channel, &second);
    *valeur = (float)current_channel;
  }

  return res;
}

// type 2
uint8_t requete_SetReg_appli(int param, float valeurf)
{
  int16_t valeur = int16_t(round(valeurf));
  uint8_t res = 1;


  return res;
}

// type 4
uint8_t requete_Get_String_appli(uint8_t type, String var, char *valeur)
{
  uint8_t res=1;
  int paramV = var.toInt();
  // valeur limité a 50 caractères
  
  if (paramV == 61)  // registre 61 : adresse MAC GW (ce module)
  {
    res = 0;
    strncpy(valeur, WiFi.macAddress().c_str(), 18);
  }

  return res;
}

uint8_t parseMacString(const char* str, uint8_t mac[6]) {
  int v[6];
  if (sscanf(str, "%x:%x:%x:%x:%x:%x",
             &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
    return false;
  }
  for (int i = 0; i < 6; i++) mac[i] = (uint8_t)v[i];
  return true;
}

// type 4
uint8_t requete_Set_String_appli(int param, const char *texte)
{
  uint8_t res=1;
  IPAddress ip;


  return res;
}

void event_cycle()
{

}

// type5 : reception message ACTION par uart ou par page web
uint8_t requete_action_appli(const char *reg, const char *data)
{
  uint8_t res=1;

  if (strcmp(reg, "Test1") == 0) 
    { 
      res=0; 
      requete_status(buffer_dmp, 0, 1);
      Serial.println(buffer_dmp);
    }

  if (strcmp(reg, "Tint") == 0) 
    { 
      res=0; 
      uint8_t Tint_erreur = lecture_Tint(&Tint, &Humid);
      Serial.println(Tint_erreur);
      Serial.println(Tint);
    }
  return res;
}


// erreur :0:ok  sinon erreur 2 à 7
uint8_t lecture_Tint(float *mesure, float*humid)
{
  uint8_t Tint_erreur = 7;
  float valeur = 20;
  float valeur2 = 50;

  #ifdef ESP_THERMOMETRE

    #ifdef Temp_int_DHT22
      //dht[0].begin();

      if (digitalRead(PIN_Tint22) == HIGH || digitalRead(PIN_Tint22) == LOW)
      {
        valeur =  dht[0].readTemperature();
        if (isnan(valeur))
        {
          valeur = 20.0;
          Tint_erreur = 6;
          Serial.println("---DHT:non numérique");
        }
        else
        {
          Tint_erreur=0;
        }
      }
      else
        Serial.println("---DHT:signal non stable!");
    #endif

    #ifdef Temp_int_HDC1080
      valeur = hdc1080.readTemperature();
      if (isnan(valeur) || (valeur>124)) {
        Serial.println("Reset i2c)");
        resetI2C(); 
        hdc1080.begin(0x40); 
        valeur = hdc1080.readTemperature();

        if (isnan(valeur) || (valeur>124)) {
          Serial.println("Recovery i2c)");
          i2cRecovery();
          hdc1080.begin(0x40); 
          valeur = hdc1080.readTemperature();
          if (isnan(valeur) || (valeur>124)) {
            valeur = 20.0;
            Tint_erreur = 4;
          } else  Tint_erreur=0;
        } else  Tint_erreur=0;

      } else {
        Tint_erreur=0;
      }
      valeur2 = hdc1080.readHumidity();
    #endif

    #ifdef Temp_int_DS18B20
      valeur = ds.getTemperature();
      Tint_erreur=0;
    #endif

  #endif

  if (valeur > 50) Tint_erreur = 2;
  if (valeur < -20) Tint_erreur = 3;
  if (log_detail>=4) Serial.printf("lecture Tint : %.2f Err:%i\n\r", valeur, Tint_erreur);
  *mesure = valeur;
  *humid = valeur2;
  return Tint_erreur;
}



//mesure temperature exterieure
uint8_t lecture_Text(float *mesure) {
  uint8_t Text_erreur = 0;
  int16_t Val_Text = 1600;
  float valeur;

  #ifdef MODBUS
    Text_erreur = read_modbus(2, &Val_Text);  // registre 1-2-3 pour temp exterieure
    valeur = (float)Val_Text / 10;
  #else
    valeur = 18.0;

    /*#ifndef DEBUG_SANS_Sonde_Ext
        Val_Text = analogRead( PIN_Text );  // 0 à 4096
        //Serial.println(Val_Text);
      #endif*/
    // calibration
    // Text1:100(10°C) Text1Val:500
    // Text2:200(20°C) Text2Val:2000
    //valeur = ((float)(Text1Val-Val_Text)/(Text1Val-Text2Val)*(Text2-Text1) + Text1)/10;

    /*float Vmesure = ((float)Val_Text / resolutionADC) * 3.66;
      float Rntc = 15000 * Vmesure / (3.3 - Vmesure);  // Calcul de la résistance de la thermistance
      float T_kelvin = 1.0 / ((1.0 / 298.15) + (1.0 / TBeta) * log(Rntc / Therm0));    // Calcul de la température en Kelvin
      valeur = T_kelvin - 273.15;    // Conversion en °C */
    //Serial.printf("val_text:%i vmesure:%.3f rntc:%.0f T_kelvin:%.1f valeur:%.1f\n", Val_Text, Vmesure, Rntc, T_kelvin, valeur);
  #endif

  if ((valeur < -30.0) || (valeur > 60.0)) Text_erreur = 1;
  if (!Val_Text) Text_erreur = 2;

  *mesure = valeur;
  return Text_erreur;
}



uint8_t fetch_internet_temp() {

  uint8_t res=1;

  // 1. Vérifier si le réseau est disponible avant de commencer
  if (WiFi.status() != WL_CONNECTED) {
    // Si vous utilisez l'Ethernet, remplacez par le test approprié
    return res; 
  }

  HTTPClient http;

  // 2. Définir un timeout court (2000ms au lieu des 5-10s par défaut)
  http.setTimeout(2000); 

  char url[150];  // assez grand pour contenir toute l'URL
  sprintf(url, "http://api.open-meteo.com/v1/forecast?latitude=%s&longitude=%s&current=temperature_2m", latitude, longitude);


  if (http.begin(url)) {
    int httpCode = http.GET();
    if (httpCode == 200) {
      String payload = http.getString();
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, payload);
      
      if (!error) {
        float temp = doc["current"]["temperature_2m"] | NAN;
        if (!isnan(temp) && temp > -50.0 && temp < 60.0)
        {
          res = 0;
          Text = temp;
          //Serial.printf("Météo Garches : %.1f°C\n", Text);
          uint32_t mil = millis();
          if (mil - last_remote_Text_time > 35*60*1000) // le precedent message est vieux de plus de 35 minutes
            err_Text++;
          last_remote_Text_time = mil;
          cpt24_Text++;
          tempE_moy24h += Text;
        }
      } else {
        Serial.printf("Erreur parsing JSON Météo : %s\n", error.c_str());
      }
    } else {
      Serial.printf("Erreur HTTP Météo (%d) : %s\n", httpCode, http.errorToString(httpCode).c_str());
    }
    http.end();
  }
  return res;
}

void event_mesure_temp()  // toutes les 15 minutes : modif allumage chaudiere
{
}



float readBatteryVoltage() {
  // Lecture ADC (0-4095) sur PIN_Vbatt
  // Sur ESP32 DevKit V1, l'ADC est calibré par défaut
  int raw = analogRead(PIN_Vbatt);
  
  // Conversion:
  // raw / 4095.0 * 3.3V (tension ref approx) * 2 (pont diviseur) * 1.1 (facteur corection empirique souvent nécessaire sur ESP32)
  // On commence sans facteur 1.1 pour tester
  float voltage = (raw / 4095.0) * 3.3 * 2.5; 
  return voltage;
}

String getTimestamp()
{
    struct tm timeinfo;

    if (!getLocalTime(&timeinfo))
    {
        return "1970-01-01 00:00:00";
    }

    char buffer[25];

    strftime(
        buffer,
        sizeof(buffer),
        "%Y-%m-%d %H:%M:%S",
        &timeinfo
    );

    return String(buffer);
}

uint8_t suppression_node(uint8_t node)
{
  uint8_t index = NB_CAPT;
  uint8_t total = 0;

  // nombre total de node
  for (; total < NB_CAPT; total++) {
    if (! Node[total].Add_node) {
      break;
    }
  }

  // est-ce que le node existe ?
  for (uint8_t i = 0; i < total; i++) {
    if (Node[i].Add_node != 0 && Node[i].Add_node == node) {
      index = i;
      break;
    }
  }
  if (index == total) return 1;  // node pas trouvé


  // Décale chaque structure suivante d'une position vers le début du tableau.
  if (index + 1 < total) {
    memmove(&Node[index], &Node[index + 1],
            (total - index - 1) * sizeof(Node[0]));
  }

  // Supprime la dernière structure désormais en double.
  memset(&Node[total - 1], 0, sizeof(Node[0]));

  // decale le tableau valT pour ce node
  for (uint8_t j = index; j < total - 1; j++)
  {
    for (uint8_t i = 0; i < NB_Val_Graph; i++) {
      for (uint8_t k = 0; k < 4; k++) {
        valT[i][j][k] = valT[i][j + 1][k];
      }
    }
  }  
  
  return 0;
}

uint8_t conversion_node(uint8_t emetteur, uint8_t *node)
{
  // Implémentation de la conversion du nœud
  for (uint8_t i = 0; i < NB_CAPT; i++) {
    if (Node[i].Add_node == emetteur) {
      *node = i;
      return 0; // Succès
    }
  }
  // verif s'il reste des places vides pour de nouveaux nodes
  for (uint8_t i = 0; i < NB_CAPT; i++) {
    if (Node[i].Add_node == 0) { // place vide
      Node[i].Add_node = emetteur;
      *node = i;
      return 0; // Succès
    }
  } 
  return 1; // Échec
}

#ifdef ESP_TJ_ACTIF
// Callback reception ESP-NOW
//void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {

/*Structure message : typedef struct __attribute__((packed)) {   // packed permet d'éviter les octets de padding ajoutés par le compilateur
    uint8_t destinataire;  // Bit fort=1 => message hexa
    uint8_t emetteur;
    uint8_t longueur;
    uint8_t code;
    uint8_t code2;
    uint8_t payload[MAX_PAYLOAD];
} Message_EspNow;*/

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
//void OnDataRecv(const esp_now_peer_info_t * info, const uint8_t *incomingData, int len) {
  // 🔍 DIAGNOSTIC: Afficher infos de réception
  Serial.println("\n📥 ========== RECEPTION ESP-NOW ==========");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", info->src_addr[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  
  // Afficher le canal WiFi actuel
  uint8_t current_channel;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&current_channel, &second);
  Serial.printf("   Canal WiFi actuel: %d\n", current_channel);
  Serial.printf("   Taille reçue: %d octets\n", len);
  
  if (len > sizeof(Message_EspNow)) {
    Serial.println("⚠️ Taille de message trop ");
    return;
  }
  Message_EspNow msg;
  memcpy(&msg, data, sizeof(msg));

  for (uint8_t i = 0; i < len; i++) {
    Serial.printf("%02X ", data[i]);
  }

  if ((msg.destinataire & 0x7F) == SERVER_ADD)
  {
    uint8_t node;
    uint8_t res_node = conversion_node(msg.emetteur, &node);
    Serial.println("Message destiné au serveur");
    if (msg.code == 'C')
    {
      Serial.println("Message de type Capteur");
      if (msg.code2 == 'T')
      {
        Serial.println("   Sous-type Température");
        /* Structure payload : nb_val, periode en sec(2Bytes), (temp & hum)* Nb_valeurs(4Bytes)*/
        uint16_t pos = 0;
        uint8_t nb_valeurs = msg.payload[pos++];
        uint16_t periode =  msg.payload[pos] | (msg.payload[pos + 1] << 8);
        pos += 2;

        if (!res_node && periode && periode <60 && nb_valeurs && nb_valeurs < MAX_TEMP)
        {
          // décalage de nb de valeurs
          for (uint8_t i = NB_Val_Graph - 1; i >= nb_valeurs; i--) {
            valT[i][node][0] = valT[i - nb_valeurs][node][0];  // temp
            valT[i][node][1] = valT[i - nb_valeurs][node][1];  // hr
          }
          Serial.printf("   Période: %d min, Nombre de valeurs: %d\n", periode, nb_valeurs);
          for (uint8_t i = 0; i < nb_valeurs; i++)
          {
            int16_t temp = msg.payload[pos] | (msg.payload[pos + 1] << 8);
            valT[i][node][0] = temp;  // pour affichage en temps réel
            pos += 2;

            uint16_t hr = msg.payload[pos] | (msg.payload[pos + 1] << 8);
            valT[i][node][1] = hr;  // pour affichage en temps réel
            pos += 2;

            Serial.printf( "T=%.1f  HR=%u\n", temp / 100.0 -10.0, hr/100.0);
          }
      
          /*File file = SD_MMC.open("/historique.csv", FILE_APPEND);

          if (!file)
          {
              Serial.println("Erreur ouverture fichier");
              return;
          }

          time_t timestamp;
          time(&timestamp);
          char buffer[25];

          uint8_t nb_val = msg.nb_valeurs;
          Serial.printf("✅ valeurs recues: %d°C\n", nb_val);

          for (uint8_t i = 0; i < nb_val; i++)
          {
            // formattage de la date/heure
            struct tm timeinfo;
            localtime_r(&timestamp, &timeinfo);

            strftime(buffer,  sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
            String Stimestamp = String(buffer);

              file.printf(
                  "%s,%u,%u,%u,%u\n",
                  Stimestamp.c_str(),
                  msg.emetteur,
                  msg.data[i].temp,
                  msg.data[i].ha,
                  msg.data[i].hr
              );

            // ajouter 15 minutes
            timestamp += msg.periode * 60;

            Serial.printf("   Valeur %d: %u\n", i+1, msg.data[i].temp);
          }
          file.close();

          Serial.println("Données sauvegardées");
          }*/
        }
      }
      if (msg.code2 == 'J')
      {
        Serial.println("   Sous-type 24h");

        if (!res_node && len==9)
        {
          // décalage de 1
          for (uint8_t i = NB_Val_Graph - 1; i >= 1; i--) {
            valT[i][node][2] = valT[i - 1][node][2];  // temp
            valT[i][node][3] = valT[i - 1][node][3];  // hr
          }
          uint8_t pos=0;
          int16_t temp = msg.payload[pos] | (msg.payload[pos + 1] << 8);
          valT[0][node][2] = temp;  // pour affichage en temps réel
          pos += 2;

          uint16_t hr = msg.payload[pos] | (msg.payload[pos + 1] << 8);
          valT[0][node][3] = hr;  // pour affichage en temps réel
          pos += 2;

          Serial.printf( "T=%.1f  HR=%u\n", temp / 10.0, hr/10.0);        
        }
      }
    }
    else if (msg.code == 'B') { // Batterie
        Serial.printf(" Type de message batterie: code:%d\n", msg.code);
      

      //Vbatt_Th = receivedMessage.value;
      //Serial.printf("✅ Vbatt_Th mise à jour: %.2fV\n", Vbatt_Th);
      //Vbatt_Th_I = 1;
    }
    else 
        Serial.printf("⚠️ Type de message inconnu: code:%d\n", msg.code);
  }
  else
  {
    Serial.printf("⚠️Autre destinataire: dest:%d \n", msg.destinataire);
  }
}
#endif



uint8_t envoi_now(uint8_t id_node, uint8_t channel, esp_now_peer_info_t * peerInfo)
{
  uint8_t result = false;

  // Fixer le canal
  Serial.printf("\n--- Essai canal %d ---\n", channel);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  
  // Vérifier que le canal a bien été changé
  uint8_t actual_channel;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&actual_channel, &second);
  
  if (actual_channel != channel)
  {
    Serial.printf("⚠️ Échec changement canal (demandé:%d, actuel:%d)\n", channel, actual_channel);
    delay(100); // Attendre un peu plus
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_get_channel(&actual_channel, &second);
    Serial.printf("   2ème tentative: canal actuel=%d\n", actual_channel);
  } else {
    //Serial.printf("✅ Canal changé: %d\n", actual_channel);
  }
  
  delay(50); // Délai pour stabilisation du canal

  // Ajouter le peer sur ce canal
  if (esp_now_is_peer_exist(Node[id_node].mac_node)) {
    esp_now_del_peer(Node[id_node].mac_node);
  }
  peerInfo->channel = actual_channel; // Utiliser le canal réel
  if (esp_now_add_peer(peerInfo) != ESP_OK){
    Serial.println("❌ Échec ajout peer");
  }
  //Serial.println("✅ Peer ajouté");

  // Envoi Température
  Message_EspNow message;
  
  // 🔍 DIAGNOSTIC: Afficher les infos avant envoi
  /*Serial.printf("📤 Tentative envoi sur canal %d vers MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                actual_channel,
                mac_chaudiere[0], mac_chaudiere[1], mac_chaudiere[2],
                mac_chaudiere[3], mac_chaudiere[4], mac_chaudiere[5]);*/
  //Serial.printf("   Message: Type=%d, Valeur=%.2f°C\n", message.type, message.value);
  
  ackReceived=0;
  ackChannel = -1;
  esp_err_t resulta = esp_now_send(Node[id_node].mac_node, (uint8_t *) &message, sizeof(message));

  if (resulta == ESP_OK)
  {
    //Serial.printf("Envoye sur canal %d\n", actual_channel);

    // attendre la réponse max 100 ms
    int wait = 0;
    while (!ackReceived && wait < 10) { // 10 * 10ms = 100ms
        delay(10);
        wait++;
    }

    if (ackReceived) // canal trouvé
    {
      result = true; 
      Serial.println("✅ Ack Recu");
      if (last_wifi_channel != actual_channel)
      {
        last_wifi_channel = actual_channel;
      }
    }
  }
  else Serial.println("❌ Echec d'envoi");

  return result;
}
