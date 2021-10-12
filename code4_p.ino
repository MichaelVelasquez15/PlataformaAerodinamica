#include <Wire.h>
#include <Servo.h>


Servo right_prop;
Servo left_prop;

/*==El sensor MP605 le brinda datos de 16 bits,
por lo que se debe crear constantes 16int para 
almacenar los datos==*/

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];




float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;

/////////////////PID CONTANTES/////////////////
double kp=2;//2.005
double ki=0.00;//0.001
double kd=0.295;//0.295
///////////////////////////////////////////////

double throttle=1300; //Valor inicial del acelerador para los motores 
float desired_angle = 0; //Angulo donde se quiere el equilibrio de la barra 


void setup() {
  Wire.begin(); //se inicia la comunicacion por cable 
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(250000);
  right_prop.attach(3); //Conectar motor derecho en pin 3
  left_prop.attach(5);  //Conectar motor izquierdo en pin 5

  time = millis(); //Start counting time in milliseconds
  /*Para poner en marcha los ESC tenemos que enviarles 
   un valor mínimo de PWM antes de conectar la batería. 
   De lo contrario, los ESC no se iniciarán ni entrarán 
   en el modo de configuración.El valor mínimo es 1000us
   y el máximo es 2000us, ¡RECUERDA!*/
   
  left_prop.writeMicroseconds(1000); 
  right_prop.writeMicroseconds(1000);
  delay(7000); /*Tiempo para dar tiempo de encender la 
                 fuente*/
}//end of setup void

void loop() {

/////////////////////////////I M U/////////////////////////////////////
    timePrev = time;  // El tiempo anterior se almacena antes del tiempo real leído
    time = millis();  // Tiempo real leído
    elapsedTime = (time - timePrev) / 1000; 
  
  /*El timeStep es el tiempo transcurrido desde el ciclo anterior.
    Este es el valor que usamos en las fórmulas como "tiempo transcurrido"
    en segundos. Trabajamos en ms, así que tenemos que dividir el valor entre 1000.
    para obtener segundos*/

  /*Lea los valores que da el acelerómetro. 
  Sabemos que la dirección esclava de esta 
  IMU es 0x68 en hexadecimal. Para eso en 
  las funciones RequestFrom y beginTransmission se tuvo 
  que poner este valor.*/
   
     Wire.beginTransmission(0x68);
     Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true); 
   
   /*Se solicito el registro 0x3B. 
   La IMU enviará un mensaje de registro. 
   La cantidad de registro para leer se 
   especifica en la función requestFrom. 
   En este caso solicitamos 6 registros. 
   Cada valor de aceleración se compone 
   de dos registros de 8 bits, valores 
   bajos y valores altos. Para eso 
   pedimos los 6 de ellos y 
   simplemente hacemos la suma de cada 
   par. Para se desplazo hacia 
   la izquierda el registro de valores 
   altos (<<) y hacemos una operación 
   o (|) para sumar los valores bajos.*/
    
     Acc_rawX=Wire.read()<<8|Wire.read(); //cada valor necesita dos registros
     Acc_rawY=Wire.read()<<8|Wire.read();
     Acc_rawZ=Wire.read()<<8|Wire.read();

 
    /*///Esta es la parte en la que necesitas calcular los ángulos usando las ecuaciones de Euler.///
     Para obtener los valores de aceleración en unidades "g"
     primero se dividio los valores brutos que acabamosde leer 
     por 16384.0 porque ese es el valor que nos da la hoja 
     de datos MPU6050. A continuación, se calculo el valor 
     de radianes a grados dividiendo 180º por el número PI 
     que es 3.141592654 y almacenar este valor en la variable 
     rad_to_deg. Para no tener que calcular este valor en cada 
     ciclo, lo hemos hecho solo una vez antes de la anulación 
     de la configuración.
    */

    /*Ahora podemos aplicar la fórmula de Euler. 
        El atan calculará el arcotangente. 
        El pow (a, b) elevará el valor a al poder b. 
        Y finalmente la función sqrt calculará el 
        cuadrado de la raíz.*/

        
     /*---X---*/
     Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;

     
     /*---Y---*/
     Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;


     
 
   /*leeimos los datos de Gyro de la misma manera que 
   los datos de Acc. La dirección de los datos del 
   giróscopo comienza en 0x43. Podemos ver estas 
   direcciones si miramos el mapa de registro del MPU6050. 
   En este caso, solicitamos solo 4 valores. No queremos 
   el giróscopo para el eje Z.*/
    
   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Primera dirección de datos del giróscopo
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); //Solo 4 registros
   
   Gyr_rawX=Wire.read()<<8|Wire.read(); //Una vez más cambiamos y sumamos
   Gyr_rawY=Wire.read()<<8|Wire.read();
 
   /*Ahora, para obtener los datos del giróscopo en grados / segundos,
   tenemos que dividir primero el valor sin procesar por 131 porque 
   ese es el valor que nos da la hoja de datos.*/

   /*---X---*/
   Gyro_angle[0] = Gyr_rawX/131.0; 
   /*---Y---*/
   Gyro_angle[1] = Gyr_rawY/131.0;

   /*Para obtener grados tubimos que multiplicar 
   el valor de grados / segundos * por el tiempo transcurrido *.
   Finalmente se pudo aplicar el filtro final donde sumamos la 
   parte de aceleración que afecta los ángulos y por supuesto 
   multiplicamos por 0.98*/

   /*---Ángulo del eje X---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Ángulo del eje Y---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   
   /*Ahora tenemos nuestros ángulos en grados y valores de -10º a 100º aprox.*/
    //Serial.println(Total_angle[1]);

   
  
/*///////////////////////////P I D///////////////////////////////////*/
/*Para el equilibrio se uso solo un eje. se elegió
  el ángulo x para implementar el PID. Eso significa 
  que el eje x de la IMU tiene que ser paralelo al equilibrio*/

/*Primero se calculo el error entre el ángulo deseado 
  y el ángulo medido real*/
error = Total_angle[1] - desired_angle;
    
/*A continuación, el valor proporcional del PID es solo
una constante proporcional multiplicada por el error*/

pid_p = kp*error;

/*La parte integral solo debe actuar si estamos cerca 
  de la posición deseada pero queremos afinar el error.
  Por eso hice una operación if para un error entre -2 y 2 grados.
  Para integrar, simplemente sumamos el valor integral 
  anterior con el error multiplicado por la constante 
  integral. Esto integrará (aumentará) el valor de cada
  bucle hasta que alcancemos el punto 0*/
  
if(-3 <error <3)
{
  pid_i = pid_i+(ki*error);  
}

/*La última parte es la derivada. La derivada actúa sobre 
la velocidad del error. Como sabemos, la velocidad es la 
cantidad de error que se produce en una determinada cantidad 
de tiempo dividida por ese tiempo. Para eso usaremos una 
variable llamada error_previo. Restamos ese valor del error 
real y lo dividimos todo por el tiempo transcurrido. 
Finalmente multiplicamos el resultado por la constante derivada*/

pid_d = kd*((error - previous_error)/elapsedTime);

/*Los valores finales de PID son la suma de cada una de estas 3 partes*/
PID = pid_p + pid_i + pid_d;

/*Sabemos que el valor mínimo de la señal PWM es 1000us y el máximo es 2000.
Entonces eso nos dice que el valor PID puede oscilar más de -1000 y 1000 
porque cuando tenemos un valor de 2000us el valor máximo que podríamos extraer
es 1000 y cuando tenemos un valor de 1000us para el sihnal PWM, el valor 
máximo que podríamos sumar es 1000 para alcanzar el máximo de 2000us*/

if(PID < -1000)
{
  PID=-1000;
}
if(PID > 1000)
{
  PID=1000;
}

/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
pwmLeft = throttle + PID;
pwmRight = throttle - PID;


/*Once again we map the PWM values to be sure that we won't pass the min
and max values. Yes, we've already maped the PID values. But for example, for 
throttle value of 1300, if we sum the max PID value we would have 2300us and
that will mess up the ESC.*/
//Right
if(pwmRight < 1000)
{
  pwmRight= 1000;
}
if(pwmRight > 2000)
{
  pwmRight=2000;
}
//Left
if(pwmLeft < 1000)
{
  pwmLeft= 1000;
}
if(pwmLeft > 2000)
{
  pwmLeft=2000;
}

/*Finnaly using the servo function we create the PWM pulses with the calculated
width for each pulse*/
left_prop.writeMicroseconds(pwmLeft);
right_prop.writeMicroseconds(pwmRight);
previous_error = error; //Remember to store the previous error.

}//end of loop void
