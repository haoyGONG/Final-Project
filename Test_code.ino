//==============================Haoyan GONG====================================
//  Robot car controlling test code using Weixin Yun WiFi module
//=============================================================================

#define run_car     '1'//Forward
#define back_car    '2'//Back
#define left_car    '3'//Left
#define right_car   '4'//Right
#define stop_car    '0'//Stop
#define left_turn   0x06//Left_spin
#define right_turn  0x07//Right_spin

enum{
  enSTOP = 0,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT
}enCarState;
//==============================
//
//Speed control volume
#define level1  0x08  //Speed control flag bit1
#define level2  0x09  //Speed control flag bit2
#define level3  0x0A  //Speed control flag bit3
#define level4  0x0B  //Speed control flag bit4
#define level5  0x0C  //Speed control flag bit5
#define level6  0x0D  //Speed control flag bit6
#define level7  0x0E  //Speed control flag bit7
#define level8  0x0F  //Speed control flag bit8
//==============================
//==============================
int Left_motor_back = 5;     //Left motor back(IN2)
int Left_motor_go = 9;     //Left motor forward(IN1)
int Right_motor_go = 10;    // Right motor forward(IN4)
int Right_motor_back = 6;    // Right motor back(IN3)
int buzzer = 8; //Set the digital IO foot to control the buzzer

int control = 150;  //PWM control volume
int incomingByte = 0;          // Received data byte
String inputString = "";         // Used to store received content
boolean newLineReceived = false; // Previous data end flag
boolean startBit  = false;  //Protocol start mark
int g_carstate = enSTOP; //  1forward;2back;3left;4right;0stop
/*ultrasonic*/
int Echo = A5;  // Echo(P2.0)
int Trig =A4;  //  Trig (P2.1)
int Distance = 0;
String returntemp = ""; //Stored return value 

/*steering engine */
int servopin = 2;  //Set steering gear driver foot to digital port 2

/*Light up*/
int Led = 13;

void setup()
{
  //Initializes motor drive IO as output mode
  pinMode(Left_motor_go, OUTPUT); // PIN 5 (PWM)
  pinMode(Left_motor_back, OUTPUT); // PIN 9 (PWM)
  pinMode(Right_motor_go, OUTPUT); // PIN 6 (PWM)
  pinMode(Right_motor_back, OUTPUT); // PIN 10 (PWM)
  pinMode(buzzer,OUTPUT);//Set the digital IO foot mode 
  pinMode(Echo, INPUT);    // Define the ultrasonic input foot
  pinMode(Trig, OUTPUT);   // Define the ultrasonic output foot
  
  Serial.begin(9600);	//9600 baud rates
 
  pinMode(servopin,OUTPUT);      //Set steering gear interface as output interface
  pinMode(Led, OUTPUT);   // Define the light output pin
   
}

void run()     // Forward
{
  digitalWrite(Right_motor_go,HIGH);  // Right motor forward
  digitalWrite(Right_motor_back,LOW);     
  analogWrite(Right_motor_go,control);//Pulse width modulation ratio 0 ~ 255 speed regulation, the difference between the left and right wheels slightly increased or decreased
  //analogWrite(Right_motor_back,0);
  digitalWrite(Left_motor_go,HIGH);  // Left motor forward
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,control);//Pulse width modulation ratio 0 ~ 255 speed regulation, the difference between the left and right wheels slightly increased or decreased
  //delay(time * 100);   //Execution time can be adjusted
}

void brake()         //Brake
{
  digitalWrite(Left_motor_back, LOW);
  digitalWrite(Left_motor_go, LOW);
  digitalWrite(Right_motor_go, LOW);
  digitalWrite(Right_motor_back, LOW);
  //delay(time * 100);//Execution time can be adjusted
}

void left()         //Turn left(Left wheel keep，right wheel forward)
{
  digitalWrite(Right_motor_go, HIGH);	// Right wheel forward
  digitalWrite(Right_motor_back, LOW);
  analogWrite(Right_motor_go, 180);//control);
  //analogWrite(Right_motor_back,0);
  digitalWrite(Left_motor_go, LOW);  //Left wheel keep
  digitalWrite(Left_motor_back, LOW);
  //analogWrite(Left_motor_go,0);
  //analogWrite(Left_motor_back,0);
  delay(100);	//Execution time can be adjusted
  digitalWrite(Right_motor_go, LOW);	//All stop
  delay(100);	//Execution time can be adjusted
}

void spin_left()         //Turn left(Left wheel back，right wheel forward)
{
  digitalWrite(Right_motor_go, HIGH);	// Right wheel forward
  digitalWrite(Right_motor_back, LOW);
  analogWrite(Right_motor_go,control);
  //analogWrite(Right_motor_back,0);
  digitalWrite(Left_motor_go, LOW);  //Left wheel back
  digitalWrite(Left_motor_back, HIGH);
  //analogWrite(Left_motor_go,0);
  analogWrite(Left_motor_back,control);
  //delay(time * 100);	//Execution time can be adjusted
}

void right()        //Turn right(Right wheel keep，left wheel forward)
{
  digitalWrite(Right_motor_go, LOW);  //Right wheel keep
  digitalWrite(Right_motor_back, LOW);
  //analogWrite(Right_motor_go,0);
  //analogWrite(Right_motor_back,0);
  digitalWrite(Left_motor_go, HIGH); //Left wheel forward
  digitalWrite(Left_motor_back, LOW);
  analogWrite(Left_motor_go,180);  //control);
  delay(100);	//Execution time can be adjusted
  digitalWrite(Left_motor_go, LOW);	//All stop
  delay(100);	//Execution time can be adjusted
}

void spin_right()        //Turn right(Right wheel back，left wheel forward)
{
  digitalWrite(Right_motor_go, LOW);  //Right wheel back
  digitalWrite(Right_motor_back, HIGH);
  //analogWrite(Right_motor_go,0);
  analogWrite(Right_motor_back,control);
  digitalWrite(Left_motor_go, HIGH); //Left wheel forward
  digitalWrite(Left_motor_back, LOW);
  //analogWrite(Left_motor_go,200);
  analogWrite(Left_motor_go,control);
  //delay(time * 100);	//Execution time can be adjusted
}

void back()          //Back
{
  digitalWrite(Right_motor_go, LOW); //Right wheel back
  digitalWrite(Right_motor_back, HIGH);
  //analogWrite(Right_motor_go,0);
  analogWrite(Right_motor_back,control);
  digitalWrite(Left_motor_go, LOW); //Left wheel back
  digitalWrite(Left_motor_back, HIGH);
  //analogWrite(Left_motor_go,0);
  analogWrite(Left_motor_back,control);
  //delay(time * 100);     //Execution time can be adjusted
}

/*Steering gear control*/
void servopulse(int servopin,int myangle)/*Defines a pulse function to generate PWM values in a simulated manner*/
{
  int pulsewidth;    //Define the pulse width variable
  pulsewidth=(myangle*11)+500;//Convert the Angle to a pulse width of 500-2480
  digitalWrite(servopin,HIGH);//Raise the port level of the steering gear
  delayMicroseconds(pulsewidth);//The number of microseconds of the delay
  digitalWrite(servopin,LOW);//Lower the port level of the steering gear
  delay(20-pulsewidth/1000);//The time remaining in the delay period
}

void left_detection()
{
  for(int i=0;i<=15;i++) //Generate PWM number, equivalent delay time to ensure that can turn to the response Angle
  {
    servopulse(servopin,175);//Analog PWM
  }
}

void right_detection()
{
  for(int i=0;i<=15;i++) //Generate PWM number, equivalent delay time to ensure that can turn to the response Angle
  {
    servopulse(servopin,5);//Analog PWM
  }
}

    
void loop() 
{  
   
    if (newLineReceived)
    {
      switch(inputString[1])
      {
        case run_car:   g_carstate = enRUN; Serial.print("run\r\n"); break;
        case back_car:  g_carstate = enBACK;  Serial.print("back\r\n");break;
        case left_car:  g_carstate = enLEFT; Serial.print("left\r\n");break;
        case right_car: g_carstate = enRIGHT; Serial.print("right\r\n");break;
        case stop_car:  g_carstate = enSTOP; Serial.print("brake\r\n");break;
        default:g_carstate = enSTOP;break;
       }
     if(inputString[3] == '1')  //Rotation
     {
        spin_right();
        delay(2000);//delay 2ms 
        brake();
        Serial.print("revolve\r\n");
     }
     if(inputString[5] == '1')  //Accelerate
     {
        control +=50;
        if(control > 255)
        {
          control = 255;
        }
        Serial.print("expedite\r\n");
     }
     if(inputString[7] == '1')  //Slow down
     {
        control -= 50;
        if(control < 50)
        {
          control = 100;
        }
        Serial.print("reduce\r\n");
     }     
     if(inputString[9] == '1')  //Left
     {
        left_detection();
     }
     if(inputString[11] == '1')  //Right
     {
       right_detection();
     }
     
     if(inputString[13] == '1')  //Light up
     {
       digitalWrite(Led, !digitalRead(Led));  //Invert level  
     }
   
     if(inputString[15] == '1')  //Reset
     {
       front_detection();
     }
     //Return status
     Distance_test();
     returntemp = "$0,0,0,0,0,0,0,0,0,0,0,";
     returntemp.concat(Distance);
     returntemp += "cm,4.2V#";
     Serial.print(returntemp);
     
     inputString = "";   // clear the string
     newLineReceived = false;
    }
    
    switch(g_carstate)
    {
      case enSTOP: brake();break;
      case enRUN:run();break;
      case enLEFT:left();break;
      case enRIGHT:right();break;
      case enBACK:back();break;
      default:brake();break;
    }
}
   
void serialEvent()
{
  while (Serial.available()) 
  {    
    incomingByte = Serial.read();              //Read byte by byte, the next sentence is the read into an array of strings to form a completed packet
    if(incomingByte == '$')
    {
      startBit= true;
    }
    if(startBit == true)
    {
       inputString += (char) incomingByte;    
    }  
    if (incomingByte == '#') 
    {
       newLineReceived = true; 
       startBit = false;
    }
  }
}
