//declaring variables

int LED = 13;      // pin 13 is given variable LED
int data;              // Variable to store data
void setup()
{
Serial.begin(9600);              //setting baud rate for communication
pinMode(LED,OUTPUT);    //Pin 13 set as output
digitalWrite(LED,LOW);       //LED is off by default
}
void loop()
{
while(Serial.available())                                 //check if data is available
{
data = Serial.read();                                       //while data is available read the data
}
if(data == '1')                                                  //if data is value '1'
{
digitalWrite(LED,HIGH);                                //turn LED on
Serial.println("LED turned on");                     //print output on serial monitor
}
else if(data == '0')                                          //if data is value '0'
{
digitalWrite(LED,LOW);                                 //turn LED off
Serial.println("LED turned off");                     //print output on serial monitor
}
}
