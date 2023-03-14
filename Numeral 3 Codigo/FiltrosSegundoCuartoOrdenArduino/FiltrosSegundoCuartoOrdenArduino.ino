/*Codigo Motor-Generador Dc Filtros:
 * Autores: David Santiago Oyola Lozano
 * Diego Fernando Blanco Rueda
 * Heiner Camilo Varela
*/

//DEFINICION DE PINES DE LOS COMPONENTES:-----------------------------------------------------------------------------------------------------

//Encoder:
#define encoder 2

//Puente H:
int ENA = 11;
int IN1 = 8;
int IN2 = 9;

//DEFINICION DE VARIABLES:--------------------------------------------------------------------------------------------------------------------
//Lectura Analogica del Encoder:
volatile unsigned muestreoactualinterrupcion=0; //El tiempo actual del muestreo de la interrupcion.
volatile unsigned muestreoanteriorinterrupcion=0; //El tiempo anterior al tiempo actual del muestreo de interrupcion.
volatile unsigned deltamuestreointerrupcion=0;  //El diferencial de tiempo entre el tiempo anterior y actual de muestreo de interrupcion.

//Variables del calculo de velocidad angular:
int ciclo=0;  //Contador
double frecuencia=0;  //La frecuencia de interrupcion calculada por el promedio de deltas de tiempo.
double w=0; //Velocidad angular calculada del encoder.

//Variables para tiempo de muestro:
float  tiempo1=0;
float  tiempo2=0;
float  tiempo=0;
double n=1; //Numero de interrupciones (o ranuras) del medidor del encoder.

//Vector de suma promnedio:
float vector[]={0,0,0,0,0,0,0,0,0,0}; //Vector donde se guarda cada delta de tiempo para hacer un promedio.
int tam=10; //Tamaño de evaluacion de los ciclos.

//Parametros de control de PWM:                        
int  PWM=0;  //PWM que se controla.
float lecturaAnalogica =0; //Lectura analogica motor.
float voltajeSalida = 0; //Voltaje Generador.

//Parametros Filtros:
//Constantes filtro segundo orden Volotaje:
const double A2V = 0.071949430353172;
const double B2V = 0.071949430353172;

const double C2V = 1.389976633757370;
const double D2V = -0.533875494463715;

//Constantes filtro cuarto orden Voltaje:
const double A4V = 0.002714868048220;
const double B4V = 0.008144604144661;
const double C4V = 0.008144604144661;
const double D4V = 0.002714868048220;

const double E4V = 2.855972422291980;
const double F4V = -3.177254463083176;
const double G4V = 1.613162165337777;
const double H4V = -0.313599068932344;

//Constantes filtro segundo orden Velocidad:
const double A2Vel = 0.124681557045141;
const double B2Vel = 0.124681557045141;

const double C2Vel = 1.165451455649830;
const double D2Vel = -0.414814569740113;

//Constantes filtro cuarto orden Velocidad:
const double A4Vel = 0.008309467203973;
const double B4Vel = 0.024928401611919;
const double C4Vel = 0.024928401611919;
const double D4Vel = 0.008309467203973;

const double E4Vel = 2.416741890621338;
const double F4Vel = -2.384913665665483;
const double G4Vel = 1.098433138549570;
const double H4Vel = -0.196737101137208;

//Variables Filtros:
double yn_volt2, yn_volt4;

//Segundo Orden Voltaje:
double Dos_Voyn2, Dos_Voyn1;
double Dos_Voxn2, Dos_Voxn1;

//Cuarto Orden Voltaje:
double Cuatro_Voyn4, Cuatro_Voyn3, Cuatro_Voyn2, Cuatro_Voyn1;
double Cuatro_Voxn4, Cuatro_Voxn3, Cuatro_Voxn2, Cuatro_Voxn1;

double yn_vel2, yn_vel4;

//Segundo Orden Velocidad.
double Dos_Veyn2, Dos_Veyn1;
double Dos_Vexn2, Dos_Vexn1;

//Cuarto Orden Velocidad:
double Cuatro_Veyn4, Cuatro_Veyn3, Cuatro_Veyn2, Cuatro_Veyn1;
double Cuatro_Vexn4, Cuatro_Vexn3, Cuatro_Vexn2, Cuatro_Vexn1;

//INICIO DEL CODIGO:-------------------------------------------------------------------------------------------------------------------------
//Funcion del encoder:
void encoderVoid(){ //Esta parte del codigo entra a funcionar cuando sucede una interrupcion en el encoder
ciclo++;  //Inicia el tick.

//En esta parte se mide el diferencial de tiempo entre tick y tick medido por el encoder para sacar un promedio de tiempo:
if(ciclo==1){
      float media=0;  //Se inicializa la variable para obtener el promedio de los delta de tiempos
      deltamuestreointerrupcion=muestreoactualinterrupcion-muestreoanteriorinterrupcion;//Se hace el diferencial de tiempo de interrupcion de ticks del encoder.
      for(int i=0;i<tam-1;i++){ //Ciclo for para crear un vector de n posiciones.
        vector[i]=vector[i+1];  //Este vector se utiliza para almacenar los delta de tiempos.
      }
      vector[tam-1]=deltamuestreointerrupcion;
      for(int i=0;i<tam;i++){ //Ciclo for para comenzar a hacer el promedio de tiempos de interrupcion.
        media=vector[i]+media;
      }
      media=media/tam;  //Se divide entre la cantidad de datos para asi obtener el promedio.
      deltamuestreointerrupcion=media;
      muestreoanteriorinterrupcion=muestreoactualinterrupcion;//Se actualiza el tiempo de interrupcion anterior
    }
ciclo=0;  //Deja el tick en 0
muestreoactualinterrupcion=millis();  //Actualiza el muestreo actual.
    
    //if(voltajeSalida==0.00){n=0;}
    
//Calculo de la frecuencia angular:
    if(deltamuestreointerrupcion!=0){ //Según el promedio de tiempo sacado anteriormente se pasa a frecuencia en radianes por segundo:
      frecuencia=(1000*n)/(double)deltamuestreointerrupcion;
    }else{
      frecuencia=0;
    }
    //Se calcula la velocidad angular siendo 2*pi*frecuencia:
    w=(2*3.141516)*frecuencia;
}

void setup() {
  //Activacion de la interrupcion en flanco de bajada del encoder:
  attachInterrupt(digitalPinToInterrupt(encoder),encoderVoid,FALLING);
  Serial.begin(9600); //Monitor Serial en 9600 baudios.
  tiempo1=millis();
  //Se definen las entradas y salidas de los pines:
  pinMode(encoder, INPUT);  //Se necesita el pin del encoder como entrada para la interrupcion.
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT); //Se define los pines del puente H como Salidas.
  pinMode(IN2, OUTPUT);
  
  //Señal Puente H L298N:
  digitalWrite(IN1,HIGH); //Se ajustan IN1 e IN2 para que gire en sentido horario el motor.
  digitalWrite(IN2,LOW);  //Si se quiere girar antihorario definir IN1=LOW e IN2=HIGH.

}

void loop() {
  //Manda el setpoint de PWM Manualmente:
  if(Serial.available()>0){
     String entradaSerial = Serial.readStringUntil('\n'); //Obtiene el valor ingresado para que sea el PWM a tomar.
     PWM=entradaSerial.toFloat();
  }
  analogWrite(ENA,map(PWM,0,100,0,255)); //Enviamos el PWM al puente H.

  //Manda el setpoint de PWM de 10 en 10 automaticamente:
  /*if(millis() > 1000){
    PWM = int(millis()/1000);
  }
  if(millis() >= 101000){
    PWM = 0;
  }*/
  
  lecturaAnalogica=analogRead(A5); //Se lee el voltaje del pin analogico A5 para mensurar el voltaje de salida del generador.
  voltajeSalida=lecturaAnalogica/1023*5.0;
  
  tiempo2=millis();

  //CONFIGURACION FILTROS DIGITALES:
  /*
   * Instruccion de uso de los filtros:
   * - Para usar el filtro de segundo orden (sea de voltaje o velocidad) se debe comentar la linea de la ecuacion de diferencia del filtro de cuarto orden y sus parámetros extras.
   * - Para usar el filtro de cuarto orden (sea de voltaje o velocidad) se debe comentar la linea de la ecuacion de diferencia del filtro de segundo orden.
   */
   
  //Filtros Voltaje:
  //Filtro de Segundo Orden:
  yn_volt2 = A2V*Dos_Voxn1 + B2V*Dos_Voxn2 + C2V*Dos_Voyn1 + D2V*Dos_Voyn2;
  
  Dos_Voyn2=Dos_Voyn1;
  Dos_Voyn1=yn_volt2;

  Dos_Voxn2=Dos_Voxn1;
  Dos_Voxn1=voltajeSalida;
  
  //Filtro de Cuarto Orden:
  yn_volt4 = A4V*Cuatro_Voxn1 + B4V*Cuatro_Voxn2 + C4V*Cuatro_Voxn3 + D4V*Cuatro_Voxn4 + E4V*Cuatro_Voyn1 + F4V*Cuatro_Voyn2 + G4V*Cuatro_Voyn3 + H4V*Cuatro_Voyn4;

  Cuatro_Voyn4=Cuatro_Voyn3;
  Cuatro_Voyn3=Cuatro_Voyn2;
  Cuatro_Voyn2=Cuatro_Voyn1;
  Cuatro_Voyn1=yn_volt4;
 
  Cuatro_Voxn4=Cuatro_Voxn3;
  Cuatro_Voxn3=Cuatro_Voxn2;
  Cuatro_Voxn2=Cuatro_Voxn1;
  Cuatro_Voxn1=voltajeSalida;
  
  
  //Filtros Velocidad:
  //Filtro de Segundo Orden:
  yn_vel2 = A2Vel*Dos_Vexn1 + B2Vel*Dos_Vexn2 + C2Vel*Dos_Veyn1 + D2Vel*Dos_Veyn2;

  Dos_Veyn2=Dos_Veyn1;
  Dos_Veyn1=yn_vel2;
  
  Dos_Vexn2=Dos_Vexn1;
  Dos_Vexn1=w;
  //Filtro de Cuarto Orden:
  yn_vel4 = A4Vel*Cuatro_Vexn1 + B4Vel*Cuatro_Vexn2 + C4Vel*Cuatro_Vexn3 + D4Vel*Cuatro_Vexn4 + E4Vel*Cuatro_Veyn1 + F4Vel*Cuatro_Veyn2 + G4Vel*Cuatro_Veyn3 + H4Vel*Cuatro_Veyn4;
  
  Cuatro_Veyn4=Cuatro_Veyn3;
  Cuatro_Veyn3=Cuatro_Veyn2;
  Cuatro_Veyn2=Cuatro_Veyn1;
  Cuatro_Veyn1=yn_vel4;
  
  Cuatro_Vexn4=Cuatro_Vexn3;
  Cuatro_Vexn3=Cuatro_Vexn2;
  Cuatro_Vexn2=Cuatro_Vexn1;
  Cuatro_Vexn1=w;
  
  if(tiempo2>=tiempo1+15){  //Se obtendrá las muestras de velocidad, voltaje y PWM para los instantes de tiempo correspondientes al tiempo de muestreo.
      //PARA UTILIZAR EL FILTRO EN EL TIEMPO DE MUESTREO *10 SE DEBE MULTIPLICAR DENTRO DEL IF EL MÚLTIPLO DE 10.
      tiempo1 = millis();   //En este caso se ingresa el tiempo de muestreo en el if.
      tiempo = tiempo1/1000;
      
      //Salida de monitor Serial para imprimir los datos: 
      
      //Velocidad:
      Serial.print(" "); Serial.print(yn_vel2); Serial.print(" ");  //Voltaje de salida en voltios.
      Serial.print(" "); Serial.print(yn_vel4); Serial.print(" ");  //Voltaje de salida en voltios.
      Serial.print(" "); Serial.print(w); Serial.print(" ");  //Velocidad en RPM.

      //Voltaje:
      Serial.print(" "); Serial.print(yn_volt2); Serial.print(" ");  //Voltaje de salida en voltios.
      Serial.print(" "); Serial.print(yn_volt4); Serial.print(" ");  //Voltaje de salida en voltios.
      Serial.print(" "); Serial.print(voltajeSalida); Serial.print(" ");  //Voltaje de salida en voltios.
      
      Serial.print(" "); Serial.print(PWM); Serial.print(" ");  //Porcentaje de PWM aplicado.
      Serial.println();
    }
}
