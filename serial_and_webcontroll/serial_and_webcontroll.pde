/**
 * Simple Write. 
 * 
 * Check if the mouse is over a rectangle and writes the status to the serial port. 
 * This example works with the Wiring / Arduino program that follows below.
 */


import processing.serial.*;
import processing.net.*;

Server s;
Client c;
Serial myPort;  // Create object from Serial class
int val;        // Data received from the serial port
int s1,s2,s3,d1,d2,d3,servo,esc;
int data[];
void setup() 
{
  size(200, 200);
  // I know that the first port in the serial list on my mac
  // is always my  FTDI adaptor, so I open Serial.list()[0].
  // On Windows machines, this generally opens COM1.
  // Open whatever port is the one you're using.
  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 115200);
  s = new Server(this, 12345);
  s1=1000;
  s2=1000;
  d3=1;

}

void draw() {
  background(255);
 
       c = s.available();
  if (c != null) {
    String input = c.readString();
    input = input.substring(0, input.indexOf("\n")); // Only up to the newline
    data = int(split(input, ',')); // Split values into an array
    // Draw line using received coords
   s1=data[0];
    s2=data[1];
     s3=data[2];
      d1=data[3];
    d2=data[4];
     d3=data[5];
      servo=data[6];
   esc=data[7];
   
       
  myPort.write(generateMessage()); 
     
   
   
  }
      
      
      
      
      

}


void mousePressed(){
  
  //moveF(10000);
  //turn(1,10000);
}


void keyPressed(){
  
 if(key=='w'){
 moveF(40000,1);
 
 } 
   if(key=='a'){
turn(1,10000);
 
 } 
    if(key=='d'){
turn(0,10000);
 
 } 
  
      if(key=='s'){
moveF(10000,0);
 
 } 
  
  
}


void keyReleased(){
  
  
 stopM(); 
  
}
void mouseReleased(){
  
 stopM(); 
  //turn(0,20000);
  
}

void moveF(int speed,int dir){
  d1=0;
  d2=1;
  if(dir==0){d1=1;d2=0;}
  
  s3=0;
  s1=speed;
  s2=speed;
  
  
  
  
  
  myPort.write(generateMessage());     
  
  
  
}


void stopM(){
s1=0;
s2=0;
s3=0;

 myPort.write(generateMessage());     

}


void turn(int dir,int speed){
  if (dir==1){
   d1=1;
   d2=1;
   d3=1;
    
    
  }
  else {
     d1=0;
   d2=0;
   d3=0;
   
  }
  s1=speed;
  s2=speed;
  s3=speed;
  
  
  myPort.write(generateMessage());     
  
  
  
  
}


String generateMessage(){
  
  
  
 String out="";
 out+=str(s1);
 out+=";";
  out+=str(s2);
 out+=";";
  out+=str(s3);
 out+=";";
  out+=str(d1);
 out+=";";
  out+=str(d2);
 out+=";";
  out+=str(d3);
 out+=";";
  out+=str(servo);
 out+=";";
  out+=str(esc);
   out+=";";
  out+=str(99999);
 
 return out;
  
  
}
