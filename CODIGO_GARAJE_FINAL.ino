//librerías
#include <Adafruit_Fingerprint.h>
#include <SoftwareSerial.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

//variables
bool motor1 = 12;
bool motor2 = 11;
bool motor3 = 10;
bool motor4 = 9;
bool VARIABLE;
bool VARIABLE2;
double variablepuerta;
int getFingerprintIDez();


//ajuste de caracteres de la pantalla
LiquidCrystal_I2C lcd(0x27,20,4);

//transmisión de datos por el puerto serial
SoftwareSerial mySerial (52, 53);
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);
void fnc_bt_serial_namepin(String _name,String _pin){
	delay(2000);
	Serial2.print(String("AT+NAME")+_name);
	delay(1000);
	Serial2.print(String("AT+PIN")+_pin);
	delay(1000);
	while(Serial2.available()>0)Serial2.read();
	Serial2.flush();
}

//ajuste de los pines de conexión de la pantalla y de lo que se ejecuta
void setup(){
  lcd.init();                     
  lcd.backlight();
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10,OUTPUT);
  VARIABLE = 0;
  VARIABLE2 = 0;
  Serial.begin(9600);
  Serial2.begin(9600);
  fnc_bt_serial_namepin(String("PUERTA"),String("0000"));
  variablepuerta = 0;
  Serial.println("BÚSQUEDA LECTOR DE HUELLAS");
   finger.begin(57600);
  if (finger.verifyPassword()) {
    Serial.println("LECTOR DE HUELLAS ENCONTRADO");
  } else {
    Serial.println("LECTOR DE HUELLAS NO ENCONTRADO :(");
    while (1);
  }
  Serial.println("ESPERANDO HUELLA VÁLIDA");
  lcd.setCursor (0,0);
  lcd.print ("ESPERANDO HUELLA");
}

//variables del control de puerta via bluetooth
void loop(){
   if ((Serial2.available()>0)) {
      variablepuerta = Serial2.read();

    }
    if ((variablepuerta == 1)) {
      digitalWrite(12, HIGH);
      digitalWrite(11, HIGH);
      delay(2000);
      digitalWrite(12, LOW);
      digitalWrite(11, LOW);
      variablepuerta=0;
      

    }
    if ((variablepuerta == 2)) {
      digitalWrite(9, HIGH);
      digitalWrite(10, HIGH);
      delay(2000);
      digitalWrite(9, LOW);
      digitalWrite(10, LOW);
      variablepuerta=0;

    }

    //sacar ID de la huella
   getFingerprintIDez();
  delay(50);
}

//variables del sensor de huellas e imprimir en la pantalla la respuesta de la huella
uint8_t getFingerprintID() {
 uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("IMAGEN LEÍDA");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println("NO SE DETECTA NINGUNA HUELLA");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("ERROR DE COMUNICACIÓN");
      return p;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("ERROR DE LECTURA");
      return p;
    default:
      Serial.println("ERROR DESCONOCIDO");
      return p;
  }

  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("HUELLA LEÍDA");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("DEMASIADA IMAGEN");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("ERROR DE COMUNICACIÓN");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("NO SE PUEDE RECONOCER LA HUELLA");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("NO SE PUEDE RECONOCER LA HUELLA");
      return p;
    default:
      Serial.println("ERROR DESCONOCIDO");
      return p;
  }
  
  p = finger.fingerFastSearch();
  if (p == FINGERPRINT_OK) {
    Serial.println("HUELLA ENCONTRADA");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("ERROR DE COMUNICACIÓN");
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) {
    Serial.println("NO SE ENCUENTRAN COINCIDENCIAS");
    return p;
  } else {
    Serial.println("ERROR DESCONOCIDO");
    return p;
  }   
  
}

int getFingerprintIDez() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK){
  Serial.print("ENCONTRADA HUELLA NO RECONOCIDA"); 
  Serial.println(" ACCESO DENEGADO ");
  lcd.clear();
  lcd.print ("ACCESO DENEGADO");
  delay(3000);
  lcd.clear();
  lcd.print("ESPERANDO HUELLA");
  }
  
  if (finger.fingerID == 1 and finger.confidence>50){
  VARIABLE = 1;
  }
  if (VARIABLE2 == 0 and VARIABLE == 1){
  lcd.clear();
  Serial.print("ENCONTRADA ID #"); Serial.print(finger.fingerID); 
  Serial.print(" CON UNA COINCIDENCIA DE "); Serial.println(finger.confidence);
  Serial.println(" ENRIQUE PIQUERAS MARTINEZ - APERTURA PERMITIDA"); 
  lcd.print ("ENRIQUE PIQUERAS");
  lcd.setCursor(0,1);
  lcd.print("ABRIENDO  PUERTA");
  digitalWrite(12 , HIGH);
  digitalWrite(11 , HIGH);
  delay(5000);
  lcd.clear();
  lcd.print("ESPERANDO HUELLA");
  digitalWrite(12 , LOW);
  digitalWrite(11 , LOW);
  VARIABLE = 0;
  VARIABLE2 = 1;
  }
  if (VARIABLE2 == 1 and VARIABLE == 1){
  lcd.clear();
  Serial.print("ENCONTRADA ID #"); Serial.print(finger.fingerID); 
  Serial.print(" CON UNA COINCIDENCIA DE "); Serial.println(finger.confidence);
  Serial.println(" ENRIQUE PIQUERAS MARTINEZ - CIERRE PERMITIDO"); 
  lcd.print ("ENRIQUE PIQUERAS");
  lcd.setCursor(0,1);
  lcd.print("CERRANDO  PUERTA");
  digitalWrite(10 , HIGH);
  digitalWrite(9 , HIGH);
  delay(5000);
  lcd.clear();
  lcd.print("ESPERANDO HUELLA");
  digitalWrite(10, LOW);
  digitalWrite(9 , LOW);
  VARIABLE = 0;
  VARIABLE2 = 0;
  }
  }
