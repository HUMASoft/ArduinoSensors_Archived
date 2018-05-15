#define PIN_LED 13
String cadenaCharEntrada = "";  
bool finCadena = false;

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(9600);
  digitalWrite(PIN_LED,LOW);
}

void loop()
{
   //const int encendido = Serial.parseInt();
   int dato=analogRead(0);
  //if(finCadena){                               //Se termino de recibir una cadena completa
   // finCadena = false;                         //Permitimos volver a recibir otra cadena
     // if (Serial.available()) {  
      Serial.print(dato);  
     delay(300);
  //}
}
//
//void serialEvent(){
//  //Recepción de datos Seriales
//  while (Serial.available()) {              //Si existen datos seriales, leer a todos
//    char CaracterEntrada = Serial.read();   //Leer 1 byte serial recibido
//    if (CaracterEntrada == '\n') {          //Si el char o byte recibido es un fin de linea, activa la bandera
//      finCadena = true;                        //Si la bandera finCadena = 1, entonces la transmision esta completa
//    }   
//  }
//}
