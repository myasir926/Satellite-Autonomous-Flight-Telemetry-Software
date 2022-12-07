 /*                      ___    ___    _  _   ___   _  _     __  __   ___   _____ 
                        | _ \  / _ \  | \| | |_ _| | \| |   |  \/  | / __| |_   _|
                        |   / | (_) | | .` |  | |  | .` |   | |\/| | \__ \   | |  
                        |_|_\  \___/  |_|\_| |___| |_|\_|   |_|  |_| |___/   |_|  
                                                                                                                                                       
    ___   _      ___    ___   _  _   _____     ___    ___    ___   _____  __      __    _     ___   ___ 
   | __| | |    |_ _|  / __| | || | |_   _|   / __|  / _ \  | __| |_   _| \ \    / /   /_\   | _ \ | __|
   | _|  | |__   | |  | (_ | | __ |   | |     \__ \ | (_) | | _|    | |    \ \/\/ /   / _ \  |   / | _| 
   |_|   |____| |___|  \___| |_||_|   |_|     |___/  \___/  |_|     |_|     \_/\_/   /_/ \_\ |_|_\ |___|

                                                _ _        ___ 
                                         __ __ | | |      ( _ )
                                         \ V / |_  _|  _  / _ \
                                          \_/    |_|  (_) \___/    */   



//***************--KÜTÜPHANE İÇERİ AKTARMA BAŞLANGIÇ--***************\\   
                                                                                                                                                                  
#include <SD.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "LoRa_E32.h"
#include <Wire.h>
#include <mavlink.h>
#include <Adafruit_BMP085.h>
#include <SPI.h>
#include "RF24.h"
#include <nRF24L01.h>
#include <RF24.h>
#include <MPU6050_light.h>




//***************--KÜTÜPHANE İÇERİ AKTARMA BİTİŞ--***************\\


//***************--ÖZEL TANIMLAMALAR BAŞLANGIÇ--***************\\                                                                                                                                                                     

const int chipSelect = BUILTIN_SDCARD;
long int paketno = 1;
int analogPin = 0;
int gucyonetimmosfet = 1;


//***************--ÖZEL TANIMLAMALAR BİTİŞ--***************\\


//***************--FLAG TANIMLAMALARI BAŞLANGIÇ--***************\\

int irtifazamansayac;
int uydustatusu = 0;
int yukselisflag = 0;
int aktifinis = 0;
int pasifinis = 0;
int inistamamlandi = 0;
int donussayisi = 0;
int videoaktarim = 0;
int ayrilmabasladi = 0;

//***************--FLAG TANIMLAMALARI BİTİŞ--***************\\


//***************--ÖZEL INTERVAL TANIMLAMALARI BAŞLANGIÇ--***************\\

int irtifasabitlemeinterval = 100000;
unsigned long mssayaconceki = 0;

//***************--ÖZEL INTERVAL TANIMLAMALARI BİTİŞ --***************\\


//***************--NRF PARAMETRE KONFİGÜRASYONLARI BAŞLANGIÇ --***************\\

RF24 radio(28, 29); // NRF PİN KONFİGÜRASYONU
const uint64_t pipe = 0xE8E8F0F0E1LL; // NRF HABERLEŞMESİ KANAL KONFİGÜRASYONU

//***************--NRF PARAMETRE KONFİGÜRASYONLARI BİTİŞ --***************\\


//***************--GPS PARAMETRE KONFİGÜRASYONLARI BAŞLANGIÇ --***************\\

static const int RXPin = 15, TXPin = 14; //RX TX PİN TANIMLAMALARI
static const uint32_t GPSBaud = 9600; //GPS BAUDRATE KONFİGÜRASYONU
TinyGPSPlus gps; // GPS KÜTÜPHANESİ DEĞİŞKEN KONFİGÜRASYONU
SoftwareSerial portgps(RXPin, TXPin); //RX TX PİN TANIMLAMALARI

//***************--GPS PARAMETRE KONFİGÜRASYONLARI BİTİŞ --***************\\


//***************--LoRa PARAMETRE KONFİGÜRASYONLARI BAŞLANGIÇ --***************\\

SoftwareSerial portlora(7, 8);
LoRa_E32 e32ttl(&portlora);

//***************--LoRa PARAMETRE KONFİGÜRASYONLARI BİTİŞ --***************\\


//***************--HARİCİ KOMUTLAR BAŞLANGIÇ --***************\\

double veri[4];
Adafruit_BMP085 bmp;

//***************--HARİCİ KOMUTLAR BİTİŞ --***************\\



//***************--STRUCT TANIMLAMALARI BAŞLANGIÇ --***************\\

typedef  struct {
  long int takimno;
  long int paketno = 1 ;
  long int basinc;
  float basinc1;
  float sicaklik;
  float yukseklik;
  int   gun;
  int   ay;
  int   yil;
  int   saat;
  int   dakika;
  int   saniye;
  float longti;
  float latti;
} Signal;

Signal data;

typedef  struct {
  float irtifa;
  float yuk1;
  float irtifafark;
  float inishizi;
  float gerilim;
  float gps1x;
  float gps1y;
  float gps1z;
  int   statu;
  float pitch;
  float roll;
  float yaw;
  int   donussayisi;
  int   aktarim;
}Signal1;
Signal1 data1;

struct Signal2 {
byte komut[3];
} 
data2;

//***************--STRUCT TANIMLAMALARI BİTİŞ --***************\\


//***************--SETUP BAŞLANGIÇ--***************\\

void setup() {
Serial.begin(9600); // SERİ HABERLEŞMENİN 9600 BAUDRATE'DE BAŞLATILMASI
unsigned long mssayac = millis(); // MİLİS FONKSİYONU TANIMLAMASI
digitalWrite(3, HIGH); // RÖLE KONFİGÜRASYONU
pinMode(3, OUTPUT); // RÖLE KONFİGÜRASYONU
digitalWrite(3, HIGH); // RÖLE KONFİGÜRASYONU
radio.begin(); // TAŞIYICI YÜK HABERLEŞMESİNİN BAŞLATILMASI
radio.openReadingPipe(1, pipe); // TAŞIYICI YÜK HABERLEŞMESİNİN PARAMETRE AYARLARI
radio.startListening(); // TAŞIYICI YÜK HABERLEŞMENİN AKTİFLEŞTİRİLMESİ
e32ttl.begin(); // LoRa HABERLEŞMESİNİN BAŞLATILMASI
portgps.begin(GPSBaud); // GPS HABERLEŞMESİNİN BAŞLATILMASI
bmp.begin(); // BMP SENSÖRÜNÜN BAŞLATILMASI
if (!SD.begin(chipSelect)) { // SD KART TAKILMA KONTROLÜ

Serial.println("SD KART MEVCUT DEĞİL.");// SD KART TAKILMA KONTROLÜ
while (1) {
// SD KART TAKILMA KONTROLÜ.
}
}
Serial.println("SD KART TAKILI."); // SD KART TAKILMA KONTROLÜ
}

//***************--SETUP BİTİŞ--***************\\



//***************--LOOP BAŞLANGIÇ--***************\\

void loop() {
    
    paketno = EEPROM.read(10);
    uydustatusu = EEPROM.read(11);
    yukselisflag = EEPROM.read(12);
    aktifinis = EEPROM.read(13);
    pasifinis = EEPROM.read(14);
    inistamamlandi = EEPROM.read(15);
    donussayisi = EEPROM.read(16);
    videoaktarim = EEPROM.read(17);
    ayrilmabasladi = EEPROM.read(18);

    alici(); // ALICI FONKSİYONUNUN ÇAĞRILMASI
    sdtelemetrikayit(); // SD KAYIT FONKSİYONUNUN ÇAĞRILMASI
    gucyonetimi(); // GÜÇ YÖNETİMİ KONTROLÜ FONKSİYONUNUN ÇAĞRILMASI
    videoaktarimbilgisi(); // VİDEO AKTARIM BİLGİSİ KONTROLÜ FONKSİYONUNUN ÇAĞRILMASI

    radio.read(&veri, sizeof(veri)); // TAŞIYICI YÜKTEN GELEN VERİ PAKETİNİN PARAMATRE KONFİGÜRASYONU
    

   //***************--TAŞIYICI YÜKTEN GELEN VERİLERİN SERİ EKRANA YAZDIRILMASI BAŞLANGIÇ--***************\\

  Serial.print(veri[0]);
  Serial.print("/");
  Serial.print(veri[1]);
  Serial.print("/");
  Serial.print(veri[2]);
  Serial.print("/");
  Serial.print(veri[3]);
  Serial.print("/");

   //***************--TAŞIYICI YÜKTEN GELEN VERİLERİN SERİ EKRANA YAZDIRILMASI BİTİŞ--***************\\


  portgps.listen(); // GPS VERİSİNİN ELDE EDİLMESİ
  smartDelay(100); // GPS'E BAĞLI smartDelay

  if (millis() > 5000 && gps.charsProcessed() < 10) { // GPS DATASININ KONTROLÜ
    Serial.println(F("No GPS data received: check wiring")); // GPS DATASININ KONTROLÜ
  } 


//***************--TELEMETRİ VERİ TANIMLAMALARI BAŞLANGIÇ--***************\\

  data.takimno = 349845;
  data.paketno;
  data.basinc = bmp.readSealevelPressure();
  data.basinc1 = veri[2];
  data.sicaklik = bmp.readTemperature();
  data.yukseklik = bmp.readAltitude();
  data.gun =    gps.date.day();
  data.ay =     gps.date.month();
  data.yil =     gps.date.year();
  data.saat =   gps.time.hour();
  data.dakika = gps.time.minute();
  data.saniye =  gps.time.second();
  data.longti =  gps.location.lng();
  data.latti =  gps.location.lat();
  data1.irtifa = gps.altitude.meters();
  data1.yuk1 = veri[3];
  data1.irtifafark = 0;
  data1.inishizi = gps.speed.mps();
  data1.gerilim;
  data1.gps1x = veri[0]; // TAŞIYICI YÜK'TEN GELEN GPS1X VERİSİ
  data1.gps1y = veri[1]; // TAŞIYICI YÜK'TEN GELEN GPS1Y VERİSİ
  data1.gps1z = veri[3]; // TAŞIYICI YÜK'TEN GELEN GPS1Z VERİSİ
  data1.statu = uydustatusu;;
  data1.donussayisi = donussayisiint;
  data1.aktarim = videoaktarim;
  

  //***************--TELEMETRİ VERİ TANIMLAMALARI BİTİŞ--***************\\


  //***************--TELEMETRİ VERİ TRANSFERİ BAŞLANGIÇ--***************\\

  ResponseStatus rs = e32ttl.sendFixedMessage(0, 6, 30, &data, sizeof(Signal) ); // LoRa VE STRUCT 1 İÇİN KONFİGÜRASYONLU HABERLEŞME KOMUTU
  delay(200);
  ResponseStatus rs1 = e32ttl.sendFixedMessage(0, 6, 30, &data1, sizeof(Signal1) ); // LoRa VE STRUCT 2 İÇİN KONFİGÜRASYONLU HABERLEŞME KOMUTU

   //***************--TELEMETRİ VERİ TRANSFERİ BİTİŞ--***************\\

    
     
  paketno++; // PAKET NUMARASI ARTIRIMI
  EEPROM.write(10, paketno);



 //***************--SD KARTA KAYIT EDİLECEK VERİ TANIMLAMALARI BAŞLANGIÇ--***************\\
 
  String dataString = "";
  String dataString0 = "/";
  String dataString1 = data.takimno ;
  String dataString2  =  data.paketno;
  String dataString3  =  data.gun;
  String dataString4  =  data.ay;
  String dataString5  =  data.yil;
  String dataString6  =  data.saat;
  String dataString7  =  data.dakika;
  String dataString8  =  data.saniye;
  String dataString9  =  data.basinc;
  String dataString10  =  data.basinc1;
  String dataString11  =  data.yukseklik;
  String dataString12  =  data1.yuk1;
  String dataString13  =  data1.irtifafark;
  String dataString14  =  data1.inishizi;
  String dataString15  =  data.sicaklik;
  String dataString16  =  data1.gerilim;
  String dataString17  =  data.latti;
  String dataString18  =  data.longti;
  String dataString19  =  data1.irtifa;
  String dataString20  =  data1.gps1x;
  String dataString21  =  data1.gps1y;
  String dataString22  =  data1.gps1z;
  String dataString23  =  data1.statu;
  String dataString24  =  data1.pitch;
  String dataString25  =  data1.roll;
  String dataString26  =  data1.yaw;
  String dataString27  =  data1.donussayisi;
  String dataString28  =  data1.aktarim;

   //***************--SD KARTA KAYIT EDİLECEK VERİ TANIMLAMALARI BİTİŞ--***************\\


}

//***************-- LOOP BİTİŞ --***************\\

 




//***************-- ÖZEL FONKSİYON - 1 TANIMLAMASI BAŞLANGIÇ--***************\\

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (portgps.available())
      gps.encode(portgps.read());
  } while (millis() - start < ms);
}


//***************-- ÖZEL FONKSİYON - 1 TANIMLAMASI BİTİŞ--***************\\





//***************-- ALICI FONKSİYONU TANIMLAMASI BAŞLANGIÇ--***************\\

void alici() {

  while (e32ttl.available()  > 0) {
    delay(300);
    ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(Signal2));
    struct Signal2 data2 = *(Signal2*) rsc.data;
    Serial.print("Gelen Komut: ");
    rsc.close();
    delay(300);

    if (*(byte*)(data2.komut) == 1 )
    {
      ayrilmabasla();
    }


    if (*(byte*)(data2.komut) == 2 )
    {
      ayrilmadurdur();
    }


     if (*(byte*)(data2.komut) == 3 )
    {
       Serial.println("MOTOR TAHRİK KOMUTU");
       mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state); 
       uint8_t buf[MAV_CMD_DO_ENGINE_CONTROL = 1];
       int16_t motortahrik = mavlink_msg_to_send_buffer(buf, &msg);
       Serial.write(buf, motortahrik); 

    }

    if (*(byte*)(data2.komut) == 4 )
    {
       Serial.println("MOTOR DURDUR");
       mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state); 
       uint8_t buf[MAV_CMD_COMPONENT_ARM_DISARM];
       int16_t motordurdur = mavlink_msg_to_send_buffer(buf, &msg);
       Serial.write(buf, motordurdur);  
    } 
    
    delay(300);
  }
}

//***************-- ALICI FONKSİYONU TANIMLAMASI BİTİŞ --***************\\


//***************-- AYRILMAYI BAŞLATMA FONKSİYONU TANIMLAMASI BAŞLANGIÇ --***************\\

void ayrilmabasla()
{
 digitalWrite(3, LOW);
 ayrilmabasladi = 1;
 EEPROM.write(18, ayrilmabasladi);
}

//***************-- AYRILMAYI DURDURMA FONKSİYONU TANIMLAMASI BİTİŞ --***************\\

if ((unsigned long)(mssayac - mssayaconceki) >= irtifasabitlemeinterval) {
    digitalWrite(AYRILMAYI DURDUR);
    }

//***************-- AYRILMAYI DURDURMA FONKSİYONU TANIMLAMASI BAŞLANGIÇ --***************\\

void ayrilmadurdur()
{
digitalWrite(3, HIGH);
}

//***************-- AYRILMAYI DURDURMA FONKSİYONU TANIMLAMASI BİTİŞ --***************\\


//***************-- UYDU STATÜSÜ FONKSİYONU VE GÖREVLERİ TANIMLAMALARI BAŞLANGIÇ --***************\\

if ( yukseklik > 20){
  yukselisflag = 1;
  EEPROM.write(12, yukselisflag);

  uydustatusu = 1;
  EEPROM.write(11, uydustatusu);

}

if ( yukseklik > 500){
  pasifinis= 1;
  EEPROM.write(14, pasifinis)

  uydustatusu = 3;
  EEPROM.write(11, uydustatusu);
}

if ( yukseklik > 700){
  uydustatusu = 2;
  EEPROM.write(11, uydustatusu);
}

if ( pasifinis == 1 && yukseklik <= 400){
  ayrilmabasla();
  uydustatusu = 4;
  EEPROM.write(11, uydustatusu);
}

if ( ayrilmabasladi == 1 && yukseklik <= 390 ){

       
       mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state); 
       uint8_t buf[MAV_CMD_DO_SET_MISSION_CURRENT = 1];
       int16_t aktifinissureci = mavlink_msg_to_send_buffer(buf, &msg);
       Serial.write(buf, aktifinissureci);  
       aktifinis = 1;
       EEPROM.write(13, aktifinis);
       Serial.println("AKTIF INIS BASLADI.");

       uydustatusu = 5;
       EEPROM.write(11, uydustatusu);

}

if ( aktifinis == 1 && inishizi > 7.00) {
  
       mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state); 
       uint8_t buf[MAV_CMD_DO_CHANGE_SPEED_2 = 7];
       int16_t motorhiz7 = mavlink_msg_to_send_buffer(buf, &msg);
       Serial.write(buf, motorhiz7); 
       Serial.println("MOTOR HIZ 7 m/s SABITLENDI.");

}

           if ( aktifinis == 1 && yukseklik <= 250 ) {

       mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state); 
       uint8_t buf[MAV_CMD_DO_SET_MISSION_CURRENT = 10];
       int16_t irtifasabitlemebasla = mavlink_msg_to_send_buffer(buf, &msg);
       Serial.write(buf, irtifasabitlemebasla); 
       Serial.println("IRTIFA SABITLEME GOREVI BASLADI.");


            if ((unsigned long)(mssayac - mssayaconceki) >= irtifasabitlemeinterval) {
        
       mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state); 
       uint8_t buf[MAV_CMD_DO_SET_MISSION_CURRENT = 11];
       int16_t irtifasabitlemebitir = mavlink_msg_to_send_buffer(buf, &msg);
       Serial.write(buf, irtifasabitlemebitir); 
       Serial.println("IRTIFA SABITLEME GOREVI SONLANDI.");


        

       mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state); 
       uint8_t buf[MAV_CMD_DO_CHANGE_SPEED_2 = 5];
       int16_t motorhiz5 = mavlink_msg_to_send_buffer(buf, &msg);
       Serial.write(buf, motorhiz5); 
       Serial.println("MOTOR HIZ 5 m/s SABITLENDI.");

    }

           }

    if ( aktifinis == 1 && yukseklik <= 20 ){

       mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state); 
       uint8_t buf[MAV_CMD_DO_CHANGE_SPEED = 100];
       int16_t motorhizyuz = mavlink_msg_to_send_buffer(buf, &msg);
       Serial.write(buf, motorhizyuz);
       Serial.println("MOTOR HIZ %100");

       

    }

     if ( aktifinis == 1 && yukseklik <= 10 ){

              Serial.println("MOTOR DURDUR");
              mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state); 
              uint8_t buf[MAV_CMD_COMPONENT_ARM_DISARM];
              int16_t motordurdur = mavlink_msg_to_send_buffer(buf, &msg);
              Serial.write(buf, motordurdur); 
              buzzeraktif();
              inistamamlandi = 1;
              EEPROM.write(15, inistamamlandi);
              sdtelemetrikayit();
              uydustatusu = 6;
              EEPROM.write(11, uydustatusu);
    }

//***************-- UYDU STATÜSÜ FONKSİYONU VE GÖREVLERİ TANIMLAMALARI BİTİŞ --***************\\



//***************--SD KARTA KAYIT FONKSİYONU BAŞLANGIÇ--***************\\

void sdtelemetrikayit() {
  dataString = dataString1 + dataString0 + dataString2 + dataString0  + dataString3 + dataString0 + dataString4 + dataString0 + dataString5 + dataString0 + dataString6 + dataString0 + dataString7 + dataString0 + dataString8 + dataString0 + dataString9 + dataString0 + dataString10 + dataString0 + dataString11 + dataString0 + dataString12 + dataString0 + dataString13 + dataString0 + dataString14 + dataString0 + dataString15 + dataString0 + dataString16 + dataString0 + dataString17 + dataString0 + dataString18 + dataString0 + dataString19 + dataString0 + dataString20 + dataString0 + dataString21 + dataString0 + dataString22 + dataString0 + dataString23 + dataString0 + dataString24 + dataString0 + dataString25 + dataString0 + dataString26 + dataString0 + dataString27 + dataString0 + dataString28;


  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening datalog.txt");
  }

 
}

//***************--SD KARTA KAYIT FONKSİYONU BİTİŞ--***************\\


//***************--PİL GERİLİMİ ÖLÇME FONKSİYONU BAŞLANGIÇ--***************\\

void pilgerilimiolcum(){

    int hamvoltaj = analogRead(analogPin); // ANALOG PINDEN HAM DEĞERİ OKUMA
    float gerilim = hamvoltaj * (15.0 / 1024.0); // HAM VERI DEĞERINI (0 - 1023) VOLTAJA (0.0V - 5.0V) DÖNÜŞTÜRME
    Serial.println(voltaj);// SERI EKRANA VOLTAJ DEĞERINI YAZDIRMA 
}

//***************--PİL GERİLİMİ ÖLÇME FONKSİYONU BİTİŞ--***************\\


//***************--SD KARTA KAYIT FONKSİYONU BİTİŞ--***************\\


//***************--GÜÇ YÖNETİMİ FONKSİYONU BAŞLANGIÇ--***************\\

void gucyonetimi(){


if (voltaj <= 2.5){

    digitalWrite(gucyonetimmosfet, 0);

}

}
//***************--GÜÇ YÖNETİMİ FONKSİYONU BİTİŞ--***************\\



   //***************--VİDEO AKTARIM BİLGİSİ KONTROLÜ FONKSİYONU BAŞLANGIÇ--***************\\

void videoaktarimbilgisi(){
    if ( digitalRead("ESP-->TEENSY PIN", HIGH) {
      videoaktarim = 1;
      EEPROM.write(17, videoaktarim);

      
    }

    else{
      videoaktarim = 0;
      EEPROM.write(17, videoaktarim);

    }
}
   //***************--VİDEO AKTARIM BİLGİSİ KONTROLÜ FONKSİYONU BİTİŞ--***************\\








/*  
  _____   ___   _  __  _  _    ___    ___   ___   ___   _____   
 |_   _| | __| | |/ / | \| |  / _ \  | __| | __| / __| |_   _|  
   | |   | _|  | ' <  | .` | | (_) | | _|  | _|  \__ \   | |    
   |_|   |___| |_|\_\ |_|\_|  \___/  |_|   |___| |___/   |_|    
                                                                
      _____   _   _   ___   _  __  ___     _     _____          
     |_   _| | | | | | _ \ | |/ / / __|   /_\   |_   _|         
       | |   | |_| | |   / | ' <  \__ \  / _ \    | |           
       |_|    \___/  |_|_\ |_|\_\ |___/ /_/ \_\   |_|           
                                                                
      __  __   _   _  __   __ '' ___    __    ___   ___         
     |  \/  | | | | | \ \ / /   |_  )  /  \  |_  ) |_  )        
     | |\/| | | |_| |  \ V /     / /  | () |  / /   / /         
     |_|  |_|  \___/    |_|     /___|  \__/  /___| /___|        

                                                                */
