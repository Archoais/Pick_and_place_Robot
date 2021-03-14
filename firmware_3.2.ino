

/*
 * Pick and Place Robot Software.
 * 
 * The device boots up and displays a message with current firmware version.
 * 
 * The device proceed to enter void loop routine, prompting the user to make an imput
 * with the appropiate remote controller. 
 * The user might choose between:
 * Homming:           Bringing the robot to its start position (usable at any time).
 * Coordinate Mode:   Introduce a coordinate using the remote control
 * Instruction Mode:  Execute intructions pre-programmed in the software
 * Remote Control:    Control the robot as a remote controlled device.
 * Info Display:      Displays current position, in steps, relative to the zero position.
 * 
 * 
 * Developer: Benjamin F. Medina
 */

#include <Wire.h>
#include <Servo.h>
#include <IRremote.h>
#include <LiquidCrystal_I2C.h>

//........................................Stepper Pins

int   x_pin   = 2;
int   y_pin   = 3;
int   z_pin   = 4;
int   dx_pin  = 5;
int   dy_pin  = 6;
int   dz_pin  = 7;
int   srv_pin = 9;
int   ir_pin  = 12;
int   pwr_pin = 11;
int   buzzer  = 13;

char  user_dir;
int  user_input;
int   angle  = 0;
int   closed = 92;
int   coord [3] = {0, 0, 0};
int   instr [3];
int   reset [3];
bool  dir   [3];
int   longest_pulse;

bool  X   = HIGH;
bool  Y   = HIGH;
bool  Z   = HIGH;
bool ok   = false;
bool esc  = false;
bool halt = false;

//times
int fast=700;
int slow=1000;
int duty=2000;
int very_slow=3000;

IRrecv ir(ir_pin);
decode_results data;
Servo gripper;
LiquidCrystal_I2C lcd(0x27, 20, 4);


/**********************************************************************************
 *                          FUNCTION PROTOTYPES
 **********************************************************************************
 */


void info(void);
void pwr(int t);
void reset_val(void);
void coord_input(int t);
void ir_step_ctrl(int n);
void ir_translate(int n);
void awaiting_input(void);
void grip_and_release(int dir);
void clean_row(bool i, bool j);
void comparator(int x, int y, int z);
void sync(int x, bool x_dir, int y, bool y_dir, int z, bool z_dir, int t);
void cnst(bool xi, bool x_dir, bool yi, bool y_dir, bool zi, bool z_dir, int t);
void cnc (int x, bool x_dir, int y, bool y_dir, int z, bool z_dir, int t, float a);



void setup() 
{
  Serial.begin(115200);

  pinMode(x_pin,   OUTPUT);
  pinMode(y_pin,   OUTPUT);
  pinMode(z_pin,   OUTPUT);
  pinMode(dx_pin,  OUTPUT);
  pinMode(dy_pin,  OUTPUT);
  pinMode(dz_pin,  OUTPUT);
  pinMode(pwr_pin, OUTPUT);

  ir.enableIRIn();
  lcd.init();
  lcd.backlight();
  gripper.attach(9);

  digitalWrite(pwr_pin, LOW);
  gripper.write(180);
  lcd.setCursor(0,0);
  lcd.print("  Initializing  ");
  lcd.setCursor(0,1);
  lcd.print(" Firmware V3.00 ");
  delay(2000);
  gripper.write(0);
  analogWrite(buzzer, 0);
  awaiting_input();
}


/**********************************************************************************
 *                              FRONT END SOFTWARE
 **********************************************************************************
 */
 

void loop() 
{
  lcd.setCursor(0,1);
  ir_step_ctrl(100);
  delay(100);

  //***********HOMING MODE***********//
  if(user_input == "red" && ok == true)
  {
    lcd.setCursor(0,0);
    lcd.print("Homing  Sequence");
    lcd.setCursor(0,1);
    lcd.print("  IN  PROGRESS  ");

    /*  First homing sequence (valid only at start)
     *  Brings the robot from position (0, 0, 0) to the homming position:
     *  Coordinate (700, 700, 400)
     */
    if(coord[0]==0 && coord[1]==0 && coord[2]==0)
    {
    cnc( 350,  X, 350,  Y, 000,  Z, 1500, 0);
    cnc( 400,  X, 400,  Y, 400,  Z, 1500, 0.5);
    delay(200);
    reset[0] = coord[0];  reset[1] = coord[1];  reset[2] = coord[2];
    }

    /*  Second homming sequence. Usable at any stage. It calculates the steps requiered
     *  to come back to the homming position: Coordinate (700, 700, 400) and excecutes
     *  the necessary routine to bring the robot back.
     */
    else
    {
      for (int i = 0; i < 3; i++)
        {
        if(coord[i] >= reset[i])
          { instr[i]  = coord[i] - reset[i];    dir[i] = LOW;  }
        else if(coord[i] < reset[i])
          { instr[i]  = reset[i] - coord[i];    dir[i] = HIGH; }
        }
        
      Serial.println("Auto-Homing Vector"); 
      
       for(int j=0; j<3; j++)
       {
       Serial.print(instr[j]);
       Serial.print(",  ");
       Serial.print(dir[j]);
       Serial.print(",      ");
       }
      
      Serial.println(" ");
      Serial.println(".....................");
      cnc(instr[0], dir[0], instr[1], dir[1], instr[2], dir[2], 1500, 0);
    }
    instr[0] = 0;   instr[1] = 0;   instr[2]  = 0;
    reset_val();
    lcd.setCursor(0,1);
    lcd.print("    COMPLETE    ");
    delay(1500);
    awaiting_input();
  }

  //***********COORDINATE MODE************//
  else if(user_input == "green" && ok == true)
  {
    reset_val();
    while(esc != true)
    {
    lcd.setCursor(0,0);
    lcd.print("COORD MODE (XYZ)");
    clean_row(0,1);
    coord_input(10);
    
    while(ok == true && esc != true)
    {
      lcd.setCursor(0,1);
      lcd.print("    EXECUTING!  ");
      sync(instr[0], dir[0], instr[1], dir[1], instr[2], dir[2], 2000);
      lcd.setCursor(0,0);
      lcd.print("Repeat? Press ok");
      lcd.setCursor(0,1);
      lcd.print("Reverse?(y/n)   ");
      lcd.setCursor(15,1);
      reset_val();
      
      while(user_input != 'y' && user_input != 'n'  && user_input != "info" && ok != true && esc != true)
        {ir_translate(100);}
      if(user_input == 'y')
      {
        lcd.setCursor(0,1);
        lcd.print("    EXECUTING!  ");
        sync(instr[0], !dir[0], instr[1], !dir[1], instr[2], !dir[2], 2000);
      }
      
      else if(user_input == "info")
      {
        clean_row(0,1);
        info();
        reset_val();
        }
    }
    }
    awaiting_input();
    reset_val();
  }

  //************INSTRUCTION MODE************//

  else if(user_input == "amber" && ok == true)
  { 
    //  Uses the pick and place robot to write AIT (Athlone Institute of Technology)
    //  and performs a short demonstration of the kinematics of the robot.
    
    reset_val();
    lcd.setCursor(0,0);
    lcd.print("Instruction Mode");
    lcd.setCursor(0,1);
    lcd.print("     ACTIVE     ");
    delay(1000);
    lcd.setCursor(0,1);
    lcd.print("Drawing: AIT    ");
    delay(200);
    cnc(525 , !X , 190 , !Y , 80 , Z , duty, 0);
    delay(1000);
    cnc(000 , !X , 000 , !Y , 40 , Z , very_slow, 0); //Z down

    //THE A
    delay(2000);
    cnc(200 , X , 000 , !Y , 75 , Z , very_slow, 0);
    cnc(200 , X , 000 , !Y , 75 , Z , very_slow, 0);
    cnc(100 , X , 000 , !Y , 75 , Z , very_slow, 0);

    delay(1000);
    cnc(150 , !X , 150 , Y , 000 , Z , very_slow, 0);
    cnc(075 ,  X , 075 ,!Y , 000 , Z , very_slow, 0);
    cnc(075 , !X , 075 ,!Y , 000 , Z , very_slow, 0);
    cnc(075 ,  X , 075 , Y , 000 , Z , very_slow, 0);
    cnc(075 , !X , 075 , Y , 000 , Z , very_slow, 0);
    delay(1000);

    //NEXT LETTER
    cnc(000 , !X , 000 , Y , 140 , !Z , duty, 0); // Z climb
    cnc(100 , X , 100 , Y , 100 , Z , duty, 0);

    // Z climb
    delay(1000);
    cnc(000 , !X , 000 , Y , 120 , Z , very_slow, 0); 
    delay(1000);

    //THE I
    cnc(150 , X , 150 , !Y , 000 , Z , very_slow, 0); 
    delay(1000);

    //NEXT LETTER
    cnc(000 , !X , 000 , Y , 150 , !Z , duty, 0); // Z climb
    cnc(050 , X , 065 , Y , 50 , Z , duty, 0); //changed

    delay(1000);
    cnc(000 , !X , 000 , Y , 150 , Z , very_slow, 0); // Z climb

    // THE T
    cnc(150 , X , 150 , Y , 150 , Z , slow, 0); 
    delay(1000);
    cnc(75 , !X , 75 , !Y , 75 , !Z , very_slow, 0);
    cnc(115 , !X , 115 , Y , 000 , Z , very_slow, 0);
    cnc(000 , !X , 000 , Y , 050 , !Z , very_slow, 0);
    lcd.setCursor(0,1);
    lcd.print("    COMPLETE    ");

    cnc(000 , !X , 000 , Y , 200 , !Z , slow, 0);  //Away
    cnc(400 , !X , 400 , !Y , 500 , !Z , fast, 0);

    lcd.setCursor(0,1);
    lcd.print("Movement demonst");

    //Kinematics Demonstration
    delay(1000);
    cnc(200 , X , 200 , Y , 200 , Z , slow, 0);
    delay(1000);
    cnc(350 , X , 350 , Y , 200 , Z , 1000, 0);
    delay(1000);
    cnc(700 , X , 700 , !Y , 000 , Z , 700, 0);
    delay(1000);
    cnc(1400 , !X , 1400 , Y , 000 , Z , 700, 0);
    delay(1000);
    cnc(000 , X , 000 , !Y , 800 , Z , 1000, 0);
    delay(1000);
    cnc(000 , X , 000 , !Y , 800 , !Z , 1000, 0);
    delay(1000);
    cnc(000 , X , 600 , !Y , 000 , Z , 1000, 0);
    cnc(600 , X , 000 , !Y , 000 , Z , 1000, 0);
    delay(1000);
    sync(300 , !X , 300 , !Y , 300 , !Z , 4000);
    delay(1000);
    sync(300 , X , 500 , Y , 650 , Z , 4000);
    delay(1000);
    sync(100 , !X , 500 , !Y , 300 , !Z , 4000);
    delay(1000);
    cnc(400 , !X , 400 , !Y , 200 , !Z , 4000, 0);
    lcd.setCursor(0,1);
    lcd.print("    COMPLETE    ");
    delay(500);





}
  

  //**********REMOTE CONTROL MODE**********//
  else if(user_input == "blue" && ok == true)
  { //  Moves the robot indefinitely until "Stop" comand is given.
    
    lcd.setCursor(0,0);
    lcd.print(" REMOTE CONTROL ");
    lcd.setCursor(0,1);
    lcd.print("Cmd:            ");
    delay(200);
    
    while(esc != true)
    {
      const int t = 3000;
      
      lcd.setCursor(5,1);
      ir_step_ctrl(100);
      
      if(user_input == "fow")
      {
        cnst(HIGH,  X, HIGH,  Y, HIGH,  Z, t);
        lcd.setCursor(0,1);
        lcd.print("Cmd:            ");
      }
      else if(user_input == "back")
      {
        cnst(HIGH, !X, HIGH, !Y, HIGH, !Z, t);
        lcd.setCursor(0,1);
        lcd.print("Cmd:            ");
      }
      else if(user_input == "right")
      {
        cnst(HIGH, !X, HIGH,  Y, LOW, Z, t);
        lcd.setCursor(0,1);
        lcd.print("Cmd:            ");
      }
      else if(user_input == "left")
      {
        cnst(HIGH,  X, HIGH, !Y, LOW, Z, t);
        lcd.setCursor(0,1);
        lcd.print("Cmd:            ");
      }
      else if(user_input == "z fow")
      {
        cnst(LOW, !X, LOW, !Y, HIGH,  Z, t);
        lcd.setCursor(0,1);
        lcd.print("Cmd:            ");
      }
      else if(user_input == "z back")
      {
        cnst(LOW, !X, LOW, !Y, HIGH, !Z, t);
        lcd.setCursor(0,1);
        lcd.print("Cmd:            ");
      }
      else if(user_input == "grip" || user_input == "release")
      {
        ok = false;
        grip_and_release(user_input);
        reset_val(); 
        lcd.setCursor(0,1);
        lcd.print("Cmd:            ");
      }
      else if(user_input == "info")
      {
        clean_row(0,1);
        info();
        awaiting_input();
        reset_val();  
      }
    }
    awaiting_input();
  }


  //********INFO MODE*********//
  else if(user_input == "info")
  {
    clean_row(0,1);
    info();
    awaiting_input();
    reset_val();  
  }


  //********POWER MODE*********//
  else if(user_input == "power")
  {
    clean_row(0,1);
    digitalWrite(pwr_pin, !digitalRead(pwr_pin));
    awaiting_input();
    reset_val();  
  }
}

/**********************************************************************************
 *                         END OF FRONT END SOFTWARE
 *                          
 *                          
 *                          
 *                          
 *                          CUSTOM FUNCTION SECTION
 **********************************************************************************
 */




/**********************************************************************************
 *                                  MOVEMENT

 **********************************************************************************
 */

//Simple movement program, moves three motors in partially synchronized sequence.
void cnc(int xi, bool x_dir, int yi, bool y_dir, int zi, bool z_dir, int t, float a)
{
  digitalWrite(dx_pin,  !x_dir);
  digitalWrite(dy_pin,   y_dir);
  digitalWrite(dz_pin,   z_dir);

  //Updating steps into current robot position
  if(x_dir == HIGH)
  { coord[0] += xi; }
  else
  { coord[0] -= xi; }
  
  if(y_dir == HIGH)
  { coord[1] += yi; }
  else
  { coord[1] -= yi; }

  if(z_dir == HIGH)
  { coord[2] += zi; }
  else
  { coord[2] -= zi; }
  
  float d;
  comparator(xi, yi, zi);
  
  for (int i = 0; i < longest_pulse; i++)
  {   d = t + (float)(longest_pulse/6)*cos((6.2831*(float)i*a)/longest_pulse);

    pwr(d);
    if(xi>0)
    {
      digitalWrite(x_pin, HIGH);
      delayMicroseconds(d);
      digitalWrite(x_pin, LOW);
      delayMicroseconds(d);
      xi--;
    }
    if(yi>0)
    {
      digitalWrite(y_pin, HIGH);
      delayMicroseconds(d);
      digitalWrite(y_pin, LOW);
      delayMicroseconds(d);
      yi--;
    }
    if(zi>0)
    {
      digitalWrite(z_pin, HIGH);
      delayMicroseconds(d);
      digitalWrite(z_pin, LOW);
      delayMicroseconds(d);
      zi--;
    }
  }
}

//Moves three motors, indefinitely until a cancellation order is received
void cnst(bool xi, bool x_dir, bool yi, bool y_dir, bool zi, bool z_dir, int t)
{
  digitalWrite(dx_pin,  !x_dir);
  digitalWrite(dy_pin,   y_dir);
  digitalWrite(dz_pin,   z_dir);
  
  ok = false;
  bool step_n[3] =  { xi,     yi,     zi    };
  bool direc[3] =   { x_dir,  y_dir,  z_dir };
 
  while (ok == false)
  {     
      digitalWrite(x_pin, xi);
      digitalWrite(y_pin, yi);
      digitalWrite(z_pin, zi);
      delayMicroseconds(t-100);
      ir_step_ctrl(100);
      digitalWrite(x_pin, LOW);
      digitalWrite(y_pin, LOW);
      digitalWrite(z_pin, LOW);
      delayMicroseconds(t-100);
      pwr(100);

      //Updating succesful steps into current robot position
      for(int i = 0;  i < 3;  i++)
      {
             if(direc[i] == HIGH && step_n[i] == HIGH)
        { coord[i] += 1;  }
        else if(direc[i] == LOW  && step_n[i] == HIGH)
        { coord[i] -= 1;  }
      }
      
      ir_step_ctrl(100);
  }
  reset_val();
}

//Moves all three motors simultaneosly, at different rates and Updates Position.
void sync(int xi, bool x_dir, int yi, bool y_dir, int zi, bool z_dir, int t)
{


  digitalWrite(dx_pin,  !x_dir);
  digitalWrite(dy_pin,   y_dir);
  digitalWrite(dz_pin,   z_dir);

  //Updating steps into current robot position
    if(x_dir == HIGH)
  { coord[0] += xi; }
  else
  { coord[0] -= xi; }
  
  if(y_dir == HIGH)
  { coord[1] += yi; }
  else
  { coord[1] -= yi; }

  if(z_dir == HIGH)
  { coord[2] += zi; }
  else
  { coord[2] -= zi; }

  comparator(xi, yi, zi);

  const int res = longest_pulse;
  byte  pulse[longest_pulse+1];

  float f = 1.0;
  float x_ratio = (float)longest_pulse / (xi);
  float y_ratio = (float)longest_pulse / (yi);
  float z_ratio = (float)longest_pulse / (zi);

  int xj = 1;
  int yj = 1;
  int zj = 1;

  int pulse_x;
  int pulse_y;
  int pulse_z;
  int pulses;

  for(int n = 0; n < longest_pulse; n++)
  {
    if(f >= x_ratio*xj  &&  f <= 1.0 + x_ratio*xj)
    {
      pulse_x = 1;
      xj++;
    }
    else
    {
      pulse_x = 0;
    }
    
    if(f >= y_ratio*yj  &&  f <= 1.0 + y_ratio*yj)
    {
      pulse_y = 2;
      yj++;
    }
    else
    {
      pulse_y = 0;
    }

    if(f >= z_ratio*zj  &&  f <= 1.0 + z_ratio*zj)
    {
      pulse_z = 4;
      zj++;
    }
    else
    {
      pulse_z = 0;
    }

    pulses = pulse_x + pulse_y + pulse_z;
    //Serial.println(pulse[n], BIN);
    f = f + 1.0;
    byte pulse_data = (byte)pulses;

    digitalWrite(x_pin, bitRead(pulse_data,0));
    digitalWrite(y_pin, bitRead(pulse_data,1));
    digitalWrite(z_pin, bitRead(pulse_data,2));
    delayMicroseconds(t);
    digitalWrite(x_pin, LOW);
    digitalWrite(y_pin, LOW);
    digitalWrite(z_pin, LOW);
    delayMicroseconds(t-100);
    pwr(100); 
  }
/*
  else if (longest_pulse > res*2)
  {
    lcd.setCursor(0,1);
    lcd.print("Error Pulse>1000");
    Serial.println("Error Pulse>1000");
    Serial.print(".......................");
  }
*/
}

//Gripper Movement
void grip_and_release(int dir)
{ 
  if(dir == "grip")
  { 
    
    while(ok != true && angle >= 0)
    {
     ir_step_ctrl(100);
     gripper.write(angle);
     delay(50);
     angle--;
    }
  }
  if(dir == "release")
  { 
    angle = 180;
    gripper.write(angle);
    /*
    while(ok != true && angle >= 0)
    {
     ir_step_ctrl(100); 
     gripper.write(angle);
     delay(50);
     angle--;
    }
    */
  }
}

//Finds larger number amongst three ints
void comparator(int x, int y, int z)
{
    if(x >= y &&  x >= z)
  {
    longest_pulse = x;
  }
  else if(y >= x  &&  y >= z)
  {
    longest_pulse = y;
  }
  else if(z >= x  &&  z >= y)
  {
    longest_pulse = z;
  }

  Serial.print("Longest Pulse is: ");
  Serial.println(longest_pulse);
}



/**********************************************************************************
 *                              IR DATA INTERFACE

 **********************************************************************************
 */

// NUMERICAL INPUT
void ir_translate(int n)
{ //Listens for IR input from remote control. NUMERICAL - YES/NO - OK - '+' & '-' - INFO
  //Prints values to LCD and modifies user_input variable value.

  if(ir.decode())
  {
    delay(n);
  
       if(ir.results.value == 0xAF530CF)  //............................0
    {   user_input = 0;     lcd.print("0");}
    
  else if(ir.results.value == 0xAF5B04F)  //............................1
    {   user_input = 1;     lcd.print("1");}
    
  else if(ir.results.value == 0xAF5708F)  //...........................2
    {   user_input = 2;     lcd.print("2");}
    
  else if(ir.results.value == 0xAF5F00F)  //...........................3
    {   user_input = 3;      lcd.print("3");}
    
  else if(ir.results.value == 0xAF538C7)  //...........................4
    {   user_input = 4;      lcd.print("4");}
    
  else if(ir.results.value == 0xAF5B847)  //...........................5
    {   user_input = 5;      lcd.print("5");}
    
  else if(ir.results.value == 0xAF57887)  //...........................6
    {   user_input = 6;      lcd.print("6");}
    
  else if(ir.results.value == 0xAF5F807)  //...........................7
    {   user_input = 7;      lcd.print("7");}
    
  else if(ir.results.value == 0xAF520DF)  //...........................8
    {   user_input = 8;      lcd.print("8");}
    
  else if(ir.results.value == 0xAF5A05F)  //............................9
    {   user_input = 9;      lcd.print("9");}
    
  else if(ir.results.value == 0xAF50EF1)  //...........................PLUS
    {   user_input = HIGH;    user_dir = '+';      lcd.print("+");}
    
  else if(ir.results.value == 0xAF58E71)  //...........................MINUS
    {   user_input = LOW;     user_dir = '-';      lcd.print("-");}

  else if(ir.results.value == 0xAF54EB1)  //...........................PLUS
    {   user_input = 'y';      lcd.print("y");}
    
  else if(ir.results.value == 0xAF5CE31)  //...........................MINUS
    {   user_input = 'n';      lcd.print("n");}

  else if(ir.results.value == 0xAF508F7)  //...........................INFO
    {   user_input = "info";     lcd.print("INFO            ");}
    
  else if(ir.results.value == 0xAF5CC33)  //...........................OK
    {   ok = true; }

  else if(ir.results.value == 0xAF5C53A)  //...........................EXIT
    {   esc = true; }
    
  else if(ir.results.value == 0xAF5E817)  //...........................POWER
    {   user_input = "power";    lcd.print("POWER  INTERRUPT");}
    
  Serial.println(ir.results.value,HEX);
  ir.resume();
  }
}

//ROBOT FUNCTIONS SELECTOR
void ir_step_ctrl(int n)
{ //Listens for IR input from remote control. ROBOT FUNCTIONS
  //Prints values to LCD and modifies user_input variable value.

  if(ir.decode())
  {
    delay(n);
       if(ir.results.value == 0xAF5CC33)  //...........................OK
    {   ok = true; }
    
  else if(ir.results.value == 0xAF50EF1)  //...........................FORWARD
    {  user_input = "fow";      lcd.print("XY Forward      ");}
    
  else if(ir.results.value == 0xAF58E71)  //...........................BACKWARD
    {   user_input = "back";    lcd.print("XY Back         ");}
    
  else if(ir.results.value == 0xAF54EB1)  //...........................RIGHT
    {   user_input = "right";   lcd.print("XY Right        ");}
    
  else if(ir.results.value == 0xAF5CE31)  //...........................LEFT
    {   user_input = "left";    lcd.print("XY Left         ");}
    
  else if(ir.results.value == 0xAF5F609)  //...........................Z PlUS
    {   user_input = "z fow";   lcd.print("Z Forward       ");}
    
  else if(ir.results.value == 0xAF5CA35)  //...........................Z MINUS
    {   user_input = "z back";  lcd.print("Z Back          ");}

  else if(ir.results.value == 0xAF5D629)  //...........................GRIP
    {   user_input = "grip";   lcd.print("GRIPPING!       ");}
    
  else if(ir.results.value == 0xAF5E619)  //...........................RELEASE
    {   user_input = "release";  lcd.print("RELEASING!      ");}
    
  else if(ir.results.value == 0xAF5E817)  //...........................POWER
    {   user_input = "power";    lcd.print("POWER  INTERRUPT");}

  else if(ir.results.value == 0xAF5C53A)  //...........................EXIT
    {   esc = true; }
  
  else if(ir.results.value == 0xAF508F7)  //...........................INFO
    {   user_input = "info";     lcd.print("INFO            ");}
    
  else if(ir.results.value == 0xAF532CD)  //...........................HOME RED
    {   user_input = "red";    lcd.print("  HOMING  MODE  ");}

  else if(ir.results.value == 0xAF5A25D)  //...........................COORDINATE GREEN
    {   user_input = "green";   lcd.print("COORDINATE  MODE");}

  else if(ir.results.value == 0xAF5629D)  //...........................INSTRUCTION AMBER
    {   user_input = "amber";  lcd.print("INSTRUCTION MODE");}

  else if(ir.results.value == 0xAF5E21D)  //...........................REMOTE CRTL BLUE
    {   user_input = "blue";   lcd.print(" REMOTE CONTROL ");}

  Serial.println(ir.results.value,HEX);
  ir.resume();
  }
}



//COORDINATE ACQISITION
void coord_input(int t)
{ //Listens for IR input from remote control. NUMERICAL VALUES
  //Receives single numbers and transforms it into a complete coordinate.
  
  instr[0]=0; instr[1]=0; instr[2]=0;
  reset_val();
  delay(t);
  int n=1000; 
  lcd.setCursor(0, 1);
  lcd.print("=?    ?    ?");
  
  //        X COORDINATE        //
  
  while(user_dir != '+' && user_dir != '-' && esc == false)
  { 
    lcd.setCursor(1, 1);
    ir_translate(100);
    dir[0] = user_input;
  }

  if(esc != true)
  {  
    reset_val();
    for(int i = 0; i < 4; i++)
    {
      while(user_input == 'NUL')
      { 
        ir_translate(100);
      }
      instr[0] += user_input*n;
      n = n/10;
      reset_val();
    }
}
  
  //        Y COORDINATE        //
  
  while(user_dir != '+' && user_dir != '-' && esc == false)
    { 
      lcd.setCursor(6, 1);
      ir_translate(100);
      dir[1] = user_input;
    }

if(esc == false)
{  
  n=1000;  reset_val();
  for(int j = 0; j < 4; j++)
  {
    while( user_input == 'NUL')
    { 
      ir_translate(100);
    }
    instr[1] += user_input*n;
    n = n/10;;
    reset_val();
  }
}

  //        Z COORDINATE        //
  while(user_dir != '+' && user_dir != '-' && esc == false)
    { 
      lcd.setCursor(11, 1);
      ir_translate(100);
      dir[2] = user_input;
    }

 if(esc == false)
{    
  n=1000;  reset_val();
  for(int i = 0; i < 4; i++)
  {
    while( user_input == 'NUL')
    { 
      ir_translate(100);
    }
    instr[2] += user_input*n;
    n = n/10;
    reset_val();
  }
}
while(ok != true && esc == false)
  { ir_step_ctrl(100);}
}


// Prints current robot position in steps, relative to zero-position.
void info(void)
{
  while(esc != true)
    {
    lcd.setCursor(0,0);
    lcd.print("CURRENT POSITION");
    lcd.setCursor(0,1);
    lcd.print('=');
    
    lcd.setCursor(1,1);
    lcd.print('x');
    lcd.setCursor(2,1);
    lcd.print(coord[0]);

    lcd.setCursor(6,1);
    lcd.print('y');
    lcd.setCursor(7,1);
    lcd.print(coord[1]);

    lcd.setCursor(11,1);
    lcd.print('z');
    lcd.setCursor(12,1);
    lcd.print(coord[2]);
    
    ir_step_ctrl(100);
    }
}

//Resets reelevant values to NUL
void reset_val(void)
{
  user_input = 'NUL';
  user_dir   = 'NUL';
  ok  = false;
  esc = false;
}


//Power Interruptor RESET
void pwr(int t)
{
  if(ir.decode())
  {
    delay(t);
    if(ir.results.value == 0xAF5E817)
    {
      digitalWrite(pwr_pin, !digitalRead(pwr_pin));
      lcd.setCursor(0,1);
      lcd.print("PWR: RELAY RESET");
    }
  }
}

//Cleans a row of LCD
void clean_row(bool i, bool j)
{
  lcd.setCursor(i,j);
  lcd.print("                ");
}

//Displays Awaiting input to LCD
void awaiting_input(void)
{
  lcd.setCursor(0,0);
  lcd.print("Awaiting 4 Input");
  lcd.setCursor(0,1);
  lcd.print("   USE REMOTE   ");
}
