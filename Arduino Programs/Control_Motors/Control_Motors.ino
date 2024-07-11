/*
This code was created to prove the function of the motors

*/

// DEFINITION OF VARIABLES AND PINS
  //Right motor pins
  #define speed_reg_MR 2    //speed regulation pin (PWM:0-5V) of right motor 
  #define turn_dir_MR 22     //direction of rotation pin of right motor
  #define speed_out_MR A0   //speed feeback pin from hall sensors of right motor
  #define brake_MR 24       //brake pin of right motor
  
  //Left motor pins
  #define speed_reg_ML 3    //speed regulation pin (PWM:0-5V) of left motor 
  #define turn_dir_ML 36     //direction of rotation pin of left motor
  #define speed_out_ML A1   //speed feeback pin from hall sensors of right motor
  #define brake_ML 38        //brake pin of right motor


  //Variables for reading the serial console
  String input;

  //Variables for controling the motors
  int cmd_vel_MR;
  int cmd_vel_ML;
  int raw_vel_MR;
  int raw_vel_ML;
  int stop_br;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//FUNCTION FOR CONTROLING RIGHT MOTOR
void control_MR (){

  //Control the breake
  digitalWrite(brake_MR, stop_br);

  //Conditional in order to define the direction of rotation depending the delivered velocity on the serial port
  if (cmd_vel_MR >= 0) {
    digitalWrite(turn_dir_MR, HIGH);  //Moving wheels foward
  } else{
    digitalWrite(turn_dir_MR, LOW);   //Moving wheels backward
  }

  analogWrite(speed_reg_MR, abs(cmd_vel_MR)); //controling the speed (using PWM 0-255) with the absolute value of the delivered velocity 
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//FUNCTION FOR CONTROLING LEFT MOTOR
void control_ML (){

  //Control the breake
  digitalWrite(brake_ML, stop_br);

  //Conditional in order to define the direction of rotation depending the delivered velocity on the serial port
  if (cmd_vel_ML >= 0) {
    digitalWrite(turn_dir_ML, HIGH);  //Moving wheels foward
  } else{
    digitalWrite(turn_dir_ML, LOW);   //Moving wheels backward
  }

  analogWrite(speed_reg_ML, abs(cmd_vel_ML)); //controling the speed (using PWM 0-255) with the absolute value of the delivered velocity 
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//FUNCTION FOR VIEW
void read_vel(){
  
}

void setup() {
  // put your setup code here, to run once:

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
  cmd_vel_MR=0;
  cmd_vel_ML=0;
  stop_br=0;


  digitalWrite(brake_MR, LOW);
  digitalWrite(brake_ML, LOW);

  //Start serial comunication 
  Serial.begin(9600);
  Serial.println("Ingrese dos números enteros separados por un espacio (VelocidadMotorD VelocidadMotorI) o (s/p) para activar y desactivar el brake:");
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
      cmd_vel_MR = velRString.toInt();
      cmd_vel_ML= velLString.toInt();

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
      Serial.println(cmd_vel_MR);
      Serial.print("Velocidad Asignada a Motor Izquierdo: ");
      Serial.println(cmd_vel_ML);
  }


  //Calling the function to control each motor
  control_ML();
  control_MR();


  //Reading the velocity of the motors from their halls sensors
  raw_vel_MR=analogRead(speed_out_MR);
  raw_vel_ML=analogRead(speed_out_ML);

  //Showing the velocity values of each motor
  Serial.print("Vel_MR:");
  Serial.print(raw_vel_MR);
  Serial.print(",");
  Serial.print("Vel_MI:");
  Serial.println(raw_vel_ML);

  
}

