#define PIN_LED 13

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(9600);
  digitalWrite(PIN_LED,LOW);
}

void loop()
{
   int dato=analogRead(0);
   int dato1=analogRead(1);
      Serial.print(dato);
      Serial.print(dato1);
      Serial.print('\n');
      Serial.print('\n');
      //delay(10);
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
