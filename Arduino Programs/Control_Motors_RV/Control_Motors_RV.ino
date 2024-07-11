/*
This code was created to prove the function of the motors
*/

#include <PinChangeInterrupt.h> //Library to use PCINT 


// DEFINITION OF VARIABLES AND PINS
  //PINS
  //Right motor pins
  #define speed_reg_MR 2    //speed regulation pin (PWM:0-5V) of right motor 
  #define turn_dir_MR 22     //direction of rotation pin of right motor
  #define speed_out_MR A9   //speed feeback pin from hall sensors of right motor
  #define brake_MR 24       //brake pin of right motor
  
  //Left motor pins
  #define speed_reg_ML 3    //speed regulation pin (PWM:0-5V) of left motor 
  #define turn_dir_ML 36     //direction of rotation pin of left motor
  #define speed_out_ML A8   //speed feeback pin from hall sensors of right motor
  #define brake_ML 38        //brake pin of right motor


  //VARIABLES FOR READING SERIAL CONSOLE
  String input;

  //VARIABLES FOR CONTROLING MOTORS
  int cmd_w_MR; // angular velocity MR
  int cmd_w_ML; // angurlar velocity ML
  int raw_vel_MR;
  int raw_vel_ML;
  int stop_br;

  //VARIABLES FOR READING VELOCITY
    //For counting pulses
    volatile long pulseCount_MR = 0; 
    volatile long pulseCount_ML = 0;


    //Constants for calculing
    const float pi = 3.14159265359;
    const float pulsesPerRevolution_WR = 90; // Number of pulses for each revolution of right wheel 44.7
    const float pulsesPerRevolution_WL = 90; // Number of pulses for each revolution of left wheel

    //angular velocities
    float angularSpeed_WL = 0.0;
    float angularSpeed_WR = 0.0;

    unsigned long lastTime = 0;




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//FUNCTION FOR CONTROLING RIGHT MOTOR
void control_MR (){

  //Control the breake
  digitalWrite(brake_MR, stop_br);

  //Conditional in order to define the direction of rotation depending the delivered velocity on the serial port
  if (cmd_w_MR >= 0) {
    digitalWrite(turn_dir_MR, LOW);  //Moving wheels foward
  } else{
    digitalWrite(turn_dir_MR, HIGH);   //Moving wheels backward
  }

  analogWrite(speed_reg_MR, abs(cmd_w_MR)); //controling the speed (using PWM 0-255) with the absolute value of the delivered velocity 
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//FUNCTION FOR CONTROLING LEFT MOTOR
void control_ML (){

  //Control the breake
  digitalWrite(brake_ML, stop_br);

  //Conditional in order to define the direction of rotation depending the delivered velocity on the serial port
  if (cmd_w_ML >= 0) {
    digitalWrite(turn_dir_ML, HIGH);  //Moving wheels foward
  } else{
    digitalWrite(turn_dir_ML, LOW);   //Moving wheels backward
  }

  analogWrite(speed_reg_ML, abs(cmd_w_ML)); //controling the speed (using PWM 0-255) with the absolute value of the delivered velocity 
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//INTERRUPTION FUNCTION FOR COUNTING PULSES 
void countPulse_MR(){

  if(cmd_w_MR>=0){
    pulseCount_MR=pulseCount_MR+1;
  }else {
    pulseCount_MR=pulseCount_MR-1;
  }

}

void countPulse_ML(){
  
  if (cmd_w_ML>=0){
    pulseCount_ML=pulseCount_ML+1;
  }else{
    pulseCount_ML=pulseCount_ML-1;
  }
  
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//FUNCION PARA CALCULAR VELOCIDADES
void calc_vel(){

  unsigned long currentTime = millis();
  
  // Si ha pasado un segundo, calcular las velocidades angulares
  if (currentTime - lastTime >=125) {

    // Deshabilitar interrupciones temporalmente para leer los contadores de pulsos de manera segura
    noInterrupts();
    long countMR = pulseCount_MR;
    long countML = pulseCount_ML;

    // Reiniciar los contadores de pulsos
    pulseCount_MR = 0;
    pulseCount_ML = 0;
    // Habilitar interrupciones nuevamente
    interrupts();

    // Calcular la velocidad angular del motor right en rad/s
    //float revolutionsMR = 1*((float)countMR / pulsesPerRevolution_WR);
    //angularSpeed_WR = revolutionsMR * 2 * pi; // rad/s

    float rpm_I=(float)countML*(480.0/pulsesPerRevolution_WL);// rpm 
		float rpm_D=(float)countMR*(480.0/pulsesPerRevolution_WR);// rpm
		
		float radS_I= rpm_I*pi/30;
		float radS_D= rpm_D*pi/30;
    
    // Calcular la velocidad angular del motor 2 en rad/s
    //float revolutionsML = 1*((float)countML / pulsesPerRevolution_WL);
    //angularSpeed_WL = revolutionsML * 2 * pi; // rad/s

    // Actualizar el tiempo
    lastTime = currentTime;

    //Mostrar las velocidades angulares en el monitor serie
    Serial.print("Velocidad angular motor MR: ");
    Serial.print(rpm_D);
    Serial.println(" rpm");
    
    Serial.print("Velocidad angular motor ML: ");
    Serial.print(rpm_I);
    Serial.println(" rpm");
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  

  //Pin mode of pin's motors
  //Right Motor
  pinMode(speed_reg_MR, OUTPUT);
  pinMode(turn_dir_MR, OUTPUT);
  pinMode(speed_out_MR, INPUT);
  pinMode(brake_MR, OUTPUT);

  //Left Motor
  pinMode(speed_reg_ML, OUTPUT);
  pinMode(turn_dir_ML, OUTPUT);
  pinMode(speed_out_ML, INPUT);
  pinMode(brake_ML, OUTPUT);

  //Inicializing all variables at 0
  cmd_w_MR=0;
  cmd_w_ML=0;
  stop_br=0;

  //Inicializing all brakes OFF
  digitalWrite(brake_MR, LOW);
  digitalWrite(brake_ML, LOW);

  //Start serial comunication 
  Serial.begin(9600);
  Serial.println("Ingrese dos números enteros separados por un espacio (VelocidadMotorD VelocidadMotorI) o (s/p) para activar y desactivar el brake:");


  // Configurar interrupciones externas
  //attachInterrupt(digitalPinToInterrupt(speed_out_MR), countPulse_MR, CHANGE); // Interrupción en flanco de subida
  //attachInterrupt(digitalPinToInterrupt(speed_out_ML), countPulse_ML, CHANGE); // Interrupción en flanco de subida

  // Configurar interrupciones PCINT
  attachPCINT(digitalPinToPCINT(speed_out_MR), countPulse_MR, CHANGE); // Interrupción en flanco de subida
  attachPCINT(digitalPinToPCINT(speed_out_ML), countPulse_ML, CHANGE); // Interrupción en flanco de subida

}

void loop() {

  //CONTROL VELOCITY WITH THE SERIAL PORT
  if (Serial.available() > 0)
  {
    input=Serial.readStringUntil('\n'); //Reading the serial port and saving it in the input variable
    int spaceIndex = input.indexOf(' ');  //Detecting the separator (in this case: a space) returning the index where it is located, this value is save a variable name spaceIndex

    if (spaceIndex != -1) { //if it is a space detected, it would extract the numbers 
      // Extracting the first number (before the separator(space))
      String velRString = input.substring(0, spaceIndex);
      // Extracting the second number (after the separator(space))
      String velLString = input.substring(spaceIndex + 1);
      
      // Convert the strings to ints
      cmd_w_MR = velRString.toInt();
      cmd_w_ML= velLString.toInt();

      }
      else if (input == "s"){ //if there no any space but the input has a value of "s" it would activade the brake

        stop_br=1;
        Serial.println("BRAKE ON");

      }else if (input == "p"){ //if there no any space nor an "s" but the input has a value of "p" it would desactivate the brake 

        stop_br=0;
        Serial.println("BRAKE OFF");
      }
      else { //if there no any space, nor an "s" and "p" it would print an error
      Serial.println("Error: Por favor ingrese dos números separados por un espacio o (s/p)");
      }

      // Imprime los valores leídos para verificar
      Serial.print("Velocidad asignada a Motor Derecho: ");
      Serial.println(cmd_w_MR);
      Serial.print("Velocidad Asignada a Motor Izquierdo: ");
      Serial.println(cmd_w_ML);
  }


  //Calling the function to control each motor
  control_ML();
  control_MR();

  calc_vel();


  //Reading the pulse of the motors from their halls sensors
  raw_vel_MR=digitalRead(speed_out_MR);
  raw_vel_ML=digitalRead(speed_out_ML);

  //Showing the velocity values of each motor
  /*Serial.print("Vel_MR:");
  Serial.print(raw_vel_MR);
  Serial.print(",");
  Serial.print("Vel_ML:");
  Serial.println(raw_vel_ML); */

  
}

