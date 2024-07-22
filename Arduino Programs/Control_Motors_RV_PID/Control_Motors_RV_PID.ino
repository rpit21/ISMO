/*
This code was created to prove the function of the motors
*/

#include <PinChangeInterrupt.h> //Library to use PCINT 
#include <PID_v1.h>


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
    const float pulsesPerRevolution_WR = 90; // Number of pulses for each revolution of right wheel 
    const float pulsesPerRevolution_WL = 90; // Number of pulses for each revolution of left wheel

    //angular velocities
    float rpm_R = 0.0;
    float rpm_L = 0.0;
    double radS_R= 0.0;
    double radS_L= 0.0;

    unsigned long lastTime = 0;


  // VARIABLES FOR PID
    //Right Motor

    double Setpoint_w_mr, Outpwm_mr;
    double Kp_mr=1, Ki_mr=1.219201, Kd_mr=0.1171; // change values if you are sure (the default is recomended)

    PID mr_PID(&radS_R, &Outpwm_mr, &Setpoint_w_mr, Kp_mr, Ki_mr, Kd_mr, DIRECT);

    //Left Motor
    double Setpoint_w_ml, Outpwm_ml;
    double Kp_ml=1, Ki_ml=1.219201, Kd_ml=0.1171; // change values if you are sure (the default is recomended) Kp_ml=2.19431, Ki_ml=4.219201, Kd_ml=0.1171;

    PID ml_PID(&radS_L, &Outpwm_ml, &Setpoint_w_ml, Kp_ml, Ki_ml, Kd_ml, DIRECT);




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//FUNCTION FOR CONTROLING RIGHT MOTOR
void control_MR (){

  //Control the breake
  digitalWrite(brake_MR, stop_br);

  //PID control Right Motor
  mr_PID.Compute();

  //Conditional to protect the motor of bugging itself in low velocity
  if((Outpwm_mr<=9 and Outpwm_mr>=0) or (Outpwm_mr<0 and Outpwm_mr>= -9)){
    Outpwm_mr=0;
  }

  //Conditional in order to define the direction of rotation depending the delivered velocity on the serial port
  if (Outpwm_mr >= 0) {
    digitalWrite(turn_dir_MR, LOW);  //Moving wheels foward
  } else{
    digitalWrite(turn_dir_MR, HIGH);   //Moving wheels backward
  }

  analogWrite(speed_reg_MR, abs(Outpwm_mr)); //controling the speed (using PWM 0-255) with the absolute value of the delivered velocity
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//FUNCTION FOR CONTROLING LEFT MOTOR
void control_ML (){

 //Control the breake
  digitalWrite(brake_ML, stop_br);

  //PID control left Motor
  ml_PID.Compute();

  //Conditional to protect the motor of bugging itself in low velocity
  if((Outpwm_ml<=9 and Outpwm_ml>=0) or (Outpwm_ml<0 and Outpwm_ml>= -9)){
    Outpwm_ml=0;
  }

  //Conditional in order to define the direction of rotation depending the delivered velocity on the serial port
  if (Outpwm_ml>= 0) {
    digitalWrite(turn_dir_ML, HIGH);  //Moving wheels foward
  } else{
    digitalWrite(turn_dir_ML, LOW);   //Moving wheels backward
  }

  analogWrite(speed_reg_ML, abs(Outpwm_ml)); //controling the speed (using PWM 0-255) with the absolute value of the delivered velocity 

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//INTERRUPTION FUNCTION FOR COUNTING PULSES 
void countPulse_MR(){


  if (Setpoint_w_mr>0){
    pulseCount_MR=pulseCount_MR+1;
  } else if(Setpoint_w_mr<0){
    pulseCount_MR=pulseCount_MR-1;
  } else {
    if(Outpwm_mr>0){
      pulseCount_MR=pulseCount_MR+1;
    }else if (Outpwm_mr<0){
      pulseCount_MR=pulseCount_MR-1;
    }
  }

}

void countPulse_ML(){
  
  if (Setpoint_w_ml>0){
    pulseCount_ML=pulseCount_ML+1;
  } else if(Setpoint_w_ml<0){
    pulseCount_ML=pulseCount_ML-1;
  } else {
    if(Outpwm_ml>0){
      pulseCount_ML=pulseCount_ML+1;
    }else if (Outpwm_ml<0){
      pulseCount_ML=pulseCount_ML-1;
    }
  }

  
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//FUNCION PARA CALCULAR VELOCIDADES
void calc_vel(){

  unsigned long currentTime = millis();
  
  // Si ha pasado un segundo, calcular las velocidades angulares
  if (currentTime - lastTime >=62.5) {

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

    rpm_L=(float)countML*(960.0/pulsesPerRevolution_WL);// rpm 
		rpm_R=(float)countMR*(960.0/pulsesPerRevolution_WR);// rpm
		
		radS_L= rpm_L*pi/30;
		radS_R= rpm_R*pi/30;
    
    // Calcular la velocidad angular del motor 2 en rad/s
    //float revolutionsML = 1*((float)countML / pulsesPerRevolution_WL);
    //angularSpeed_WL = revolutionsML * 2 * pi; // rad/s

    // Actualizar el tiempo
    lastTime = currentTime;

   /* //Mostrar las velocidades angulares en el monitor serie
    Serial.print("Velocidad angular motor MR: ");
    Serial.print(rpm_D);
    Serial.println(" rpm");
    
    Serial.print("Velocidad angular motor ML: ");
    Serial.print(rpm_I);
    Serial.println(" rpm");*/

    /*//Envio de velocidad por Serial
    Serial.print(radS_R,3);
    Serial.print(" ");
    Serial.println(radS_L,3);*/

    //Showing the velocity values of each motor in plotter
    Serial.print("Vel_MR:");
    Serial.print(radS_R);
    Serial.print(",");
    Serial.print("Vel_ML:");
    Serial.print(radS_L);
    Serial.print(",");
    Serial.print("output:");
    Serial.println(Outpwm_ml);
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
  Outpwm_mr=0;
  Outpwm_ml=0;
  stop_br=0;

  //Inicializing all brakes OFF
  digitalWrite(brake_MR, LOW);
  digitalWrite(brake_ML, LOW);

  //Start serial comunication 
  Serial.begin(9600);
  //Serial.println("Ingrese dos números enteros separados por un espacio (VelocidadMotorD VelocidadMotorI) o (s/p) para activar y desactivar el brake:");


  // Configurar interrupciones externas
  //attachInterrupt(digitalPinToInterrupt(speed_out_MR), countPulse_MR, CHANGE); // Interrupción en flanco de subida
  //attachInterrupt(digitalPinToInterrupt(speed_out_ML), countPulse_ML, CHANGE); // Interrupción en flanco de subida

  // Configurar interrupciones PCINT
  attachPCINT(digitalPinToPCINT(speed_out_MR), countPulse_MR, CHANGE); // Interrupción en flanco de subida
  attachPCINT(digitalPinToPCINT(speed_out_ML), countPulse_ML, CHANGE); // Interrupción en flanco de subida

  //Configuration PID
    //Right Motor
    Setpoint_w_mr=0;
    mr_PID.SetMode(AUTOMATIC);
    mr_PID.SetOutputLimits(-255, 255);
    mr_PID.SetSampleTime(62.5); // on milliseconds

    //Left Motor
    Setpoint_w_ml=0;
    ml_PID.SetMode(AUTOMATIC);
    ml_PID.SetOutputLimits(-255, 255);
    ml_PID.SetSampleTime(62.5); // on milliseconds

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
      Setpoint_w_mr = velRString.toDouble();
      Setpoint_w_ml = velLString.toDouble();

      // Imprime los valores leídos para verificar
      Serial.print("Velocidad asignada a Motor Derecho: ");
      Serial.println(Setpoint_w_mr);
      Serial.print("Velocidad Asignada a Motor Izquierdo: ");
      Serial.println(Setpoint_w_ml);

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

  }


  //Calling the function to control each motor
  control_ML();
  control_MR();
  calc_vel();


  //Reading the pulse of the motors from their halls sensors
  //raw_vel_MR=digitalRead(speed_out_MR);
  //raw_vel_ML=digitalRead(speed_out_ML);

  //Showing the velocity values of each motor
  /*Serial.print("Vel_MR:");
  Serial.print(raw_vel_MR);
  Serial.print(",");
  Serial.print("Vel_ML:");
  Serial.println(raw_vel_ML); */

  
}

