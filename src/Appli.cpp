
#include <Arduino.h>
#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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

//void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len);
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

// setup : lecture nvs
void setup_nvs_rtc()
{

    Nb_jours_Batt_log = preferences_nvs.getUChar("FrBL", 100);
    if ((Nb_jours_Batt_log > 15)) {  // 0 à 15
      Nb_jours_Batt_log = 2;  // Freq : tous les2 jours   0:inactif
      preferences_nvs.putUChar("FrBL", Nb_jours_Batt_log);
      Serial.printf("Raz Freq log Batt: %i\n\r", Nb_jours_Batt_log);
    }
    else  Serial.printf("Freq log batt: %i\n\r", Nb_jours_Batt_log);

    // esp_now_actif
    esp_now_actif = preferences_nvs.getUChar("EspN", 10);
    if (esp_now_actif < 2)  
      Serial.printf("Esp_now actif : %i\n\r", esp_now_actif);
    else {
      esp_now_actif = 0;
      Serial.println("Raz Esp_now : inactif");
    }
    // seuil batterie basse pour arret ESP
    Seuil_batt_arret_ESP = preferences_nvs.getUShort("SeAr", 100);
    if ( (!Seuil_batt_arret_ESP) || ((Seuil_batt_arret_ESP >= 3000) && (Seuil_batt_arret_ESP <= 3600)))   // 3V à 3,6V
        Serial.printf("Seuil batterie arret ESP: %i\n\r", Seuil_batt_arret_ESP);
    else {
      Seuil_batt_arret_ESP = 3300;
      preferences_nvs.putUShort("SeAr", Seuil_batt_arret_ESP);
      Serial.printf("Raz seuil batterie arret ESP: %i\n\r", Seuil_batt_arret_ESP);
    }


    // periode du cycle : lecture Temp ext par internet
    periode_cycle = preferences_nvs.getUChar("cycle", 0);  // de 10 a 120
    if ((periode_cycle < 2) || (periode_cycle > 60)) {
      periode_cycle = 15;
      preferences_nvs.putUChar("cycle", periode_cycle);
      Serial.printf("Raz periode cycle : val par defaut %imin\n\r", periode_cycle);
    }
    else Serial.printf("periode cycle : %imin\n", periode_cycle);


    mode_rapide = preferences_nvs.getUChar("Rap", 0);  // mode=12 => mode_rapide
    if ((mode_rapide) && (mode_rapide != 12)) {
      mode_rapide=0;
      preferences_nvs.putUChar("Rap", 0);
      Serial.println("Raz Mode rapide:0");
    }
    else
      Serial.printf("Mode rapide : %i\n\r", mode_rapide);

    // Initialisation variable adresse Mac Gateway
    String storedString = preferences_nvs.getString("MacC", "");

    if (parseMacString(storedString.c_str(), mac_gw))
    {
      Serial.printf("MAC chaudiere : %02X:%02X:%02X:%02X:%02X:%02X\n",
        mac_gw[0], mac_gw[1], mac_gw[2],
        mac_gw[3], mac_gw[4], mac_gw[5] );
    }
    else {  Serial.println("MAC chaudière absente ou invalide");  }


    // Initialisation du channel préférentiel wifi-esp-now
    WIFI_CHANNEL = preferences_nvs.getUChar("WifiC", 0);
    if ((WIFI_CHANNEL < 1) || (WIFI_CHANNEL > 13)) {
      WIFI_CHANNEL = 6;  // 1 à 13
      preferences_nvs.putUChar("WifiC", WIFI_CHANNEL);
      Serial.printf("Raz Wifi Channel: %i\n", WIFI_CHANNEL);
    }
    else
      Serial.printf("Wifi channel preferentiel: %i\n", WIFI_CHANNEL);
    last_wifi_channel = WIFI_CHANNEL;



    Seuil_batt_sonde = preferences_nvs.getUShort("SeBa", 0);
    if ((Seuil_batt_sonde < 1800) || (Seuil_batt_sonde >4500)) {  // 1,8V à 4,5V
      Seuil_batt_sonde = 3800;  // Seuil 3.8V
      preferences_nvs.putUShort("SeBa", Seuil_batt_sonde);
      Serial.printf("Raz batterie sonde: %i\n\r", Seuil_batt_sonde);
    }
    else  Serial.printf("Seuil batterie sonde: %i\n\r", Seuil_batt_sonde);



}

// setup : lecture nvs
void setup_nvs()
{



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
    Serial.printf("Canal WiFi AVANT config ESP-NOW: %d\n", current_channel);
    
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
    
    Serial.println("\n\n======================================");
    Serial.println("🔵 ESP-NOW Initialisé (RÉCEPTEUR)");
    Serial.print("   MAC Address: ");
    //if ((mode_reseau==13) )
    //else
    //  Serial.println(WiFi.softAPmacAddress());

    String macStr = WiFi.macAddress();
    sscanf(macStr.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
          &mac_gw[0], &mac_gw[1], &mac_gw[2],
          &mac_gw[3], &mac_gw[4], &mac_gw[5]);
    Serial.printf("   MAC : %02X:%02X:%02X:%02X:%02X:%02X\n",
          mac_gw[0], mac_gw[1], mac_gw[2],
          mac_gw[3], mac_gw[4], mac_gw[5]);

    Serial.printf("   Canal WiFi: %d\n", current_channel);
    Serial.println("   En attente de messages...");
    Serial.println("======================================\n\n");
    delay(2000); // 2 secondes de pause pour lire
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

  if (reg == 9)  // registre 9 : Seuil batterie sonde
  {
    res = 0;
    *valeur = Seuil_batt_sonde;
  }
  if (reg == 10)  // registre 10 : Nb de jours Log batterie
  {
    res = 0;
    *valeur = Nb_jours_Batt_log;
  }
  if (reg == 15)  // registre 15 : seuil batterie basse arret ESP
  {
    res = 0;
    *valeur = Seuil_batt_arret_ESP;
  }
  if (reg == 40)  // registre 40 : activation esp_now
  {
    res = 0;
    *valeur = esp_now_actif;
  }
  
  if (reg == 41)  // registre 41 : canal WiFi actuel
  {
    res = 0;
    uint8_t current_channel;
    #ifdef ESP_THERMOMETRE
      current_channel = last_wifi_channel;
    #else
      wifi_second_chan_t second;
      esp_wifi_get_channel(&current_channel, &second);
    #endif
    *valeur = (float)current_channel;
  }
  if (reg == 42)  // registre 42 : canal WiFi preferentiel
  {
    res = 0;
    *valeur = WIFI_CHANNEL;
  }

  return res;
}

// type 2
uint8_t requete_SetReg_appli(int param, float valeurf)
{
  int16_t valeur = int16_t(round(valeurf));
  uint8_t res = 1;

  if (param == 9)  // registre 9 : Seuil batterie sonde
  {
    if ((valeur >=1800 ) && (valeur <= 4500)) {
      res = 0;
      Seuil_batt_sonde = valeur;
      preferences_nvs.putUShort("SeBa", Seuil_batt_sonde);
    }
  }


  if (param == 10)  // registre 10 : Nb jours log batterie
  {
    if ((valeur) && (valeur < 16)) {
      res = 0;
      Nb_jours_Batt_log = valeur;
      preferences_nvs.putUChar("FrBL", Nb_jours_Batt_log);
    }
  }
  if (param == 15)  // registre 15 : seuil batterie basse arret ESP
  {
    if ( (!valeur) ||((valeur >= 3000) && (valeur <= 3600))) {  // 0 (inactif) ou entre 3V et 3,6V
      res = 0;
      Seuil_batt_arret_ESP = valeur;
      preferences_nvs.putUShort("SeAr", Seuil_batt_arret_ESP);
    }
  }
  if (param == 40)  // registre 40 : activation esp_now
  {
    if ((valeur == 0) || (valeur == 1))
    {
      res = 0;
      esp_now_actif = valeur;
      preferences_nvs.putUChar("EspN", esp_now_actif);
    }
  }

  if (param == 41)  // registre 41 : last_wifi_channel
  {
    if ((valeur) && (valeur <= 13))
    {
      res = 0;
      last_wifi_channel = valeur;
    }
  }
  if (param == 42)  // registre 42 : canal wifi preferentiel
  {
    if ((valeur) && (valeur <= 13))
    {
      res = 0;
      WIFI_CHANNEL = valeur;
      preferences_nvs.putUChar("WifiC", WIFI_CHANNEL);
    }
  }

  return res;
}

// type 4
uint8_t requete_Get_String_appli(uint8_t type, String var, char *valeur)
{
  uint8_t res=1;
  int paramV = var.toInt();
  // valeur limité a 50 caractères
  
  if (paramV == 11)  // registre 11 : adresse MAC ce module
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
  Serial.printf("lecture Tint : %.2f Err:%i\n\r", valeur, Tint_erreur);
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
  sprintf(url, "http://api.open-meteo.com/v1/forecast?latitude=%s&longitude=%s&current=temperature_2m", LATITUDE, LONGITUDE);


  if (http.begin(url)) {
    int httpCode = http.GET();
    if (httpCode == 200) {
      String payload = http.getString();
      DynamicJsonDocument doc(512);
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
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
//void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
//void OnDataRecv(const esp_now_peer_info_t * info, const uint8_t *incomingData, int len) {
  // 🔍 DIAGNOSTIC: Afficher infos de réception
  Serial.println("\n📥 ========== RECEPTION ESP-NOW ==========");
  for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", mac[i]);
        if (i < 5) Serial.print(":");
    }
  /*Serial.printf("   Source MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                info->src_addr[0], info->src_addr[1], info->src_addr[2],
                info->src_addr[3], info->src_addr[4], info->src_addr[5]);*/
  
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
  memcpy(&msg, incomingData, sizeof(msg));

  if (msg.destinataire==SERVER_ADD)
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

            Serial.printf( "T=%.1f  HR=%u\n", temp / 10.0, hr/10.0);
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
  }
  else if (msg.code == 2) { // Batterie
    //Vbatt_Th = receivedMessage.value;
    //Serial.printf("✅ Vbatt_Th mise à jour: %.2fV\n", Vbatt_Th);
    //Vbatt_Th_I = 1;
  }
  else {
    Serial.printf("⚠️ Type de message inconnu: %d\n", msg.code);
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


