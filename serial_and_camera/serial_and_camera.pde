
import processing.video.*;
import processing.serial.*;

Serial myPort;  // Create object from Serial class
int val;        // Data received from the serial port
int s1,s2,s3,d1,d2,d3,servo,esc;
Capture cam;
color []colors={color(175,90,80),color(160,160,165),color(0,0,0) , color(90,200,90),color(100,40,40),color(60,80,100)};
//color []colors={color(255,130,10),color(0,0,0),color(255,255,255)};
float x1,x2,y1,y2;

void setup() {
  size(1280, 360);
  cam = new Capture(this, 640, 360, 30);
  cam.start();
  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 115200);
}

void draw() {
  if(cam.available()) {
    cam.read();
  }
  background(0);
  image(cam,0,0);
 PImage img=processImg(cam);
  

  

  image(img, 640,0);
  
  
  
  
  //noFill();
  //rect(x1,y1,mouseX-x1,mouseY-y1);
 // filter(GRAY);
 // filter(POSTERIZE,2);
  
  
}


PImage processImg(PImage came){
int dev=64;
PImage ret=createImage(came.width, came.height, RGB);
  came.loadPixels();
  ret.loadPixels();
  int bluramm=3;
  
 
  
  
  
  
  

  
  
  
  for(int i=0;i<came.width*came.height;i++){
    //ret.pixels[i]=color(ceil(red(came.pixels[i])/dev)*dev,ceil(green(came.pixels[i])/dev)*dev,ceil(blue(came.pixels[i])/dev)*dev);
    float mindist=100;
    
    
    
    
    
    for (color c:colors){
      if(dist(red(came.pixels[i]),green(came.pixels[i]),blue(came.pixels[i]),red(c),green(c),blue(c))<mindist){
       mindist= dist(red(came.pixels[i]),green(came.pixels[i]),blue(came.pixels[i]),red(c),green(c),blue(c));
     ret.pixels[i]=c;
      }
      
    }
    
    
  
  }
  ret.updatePixels();
  //blur
  for(int i=bluramm;i<came.height-bluramm;i+=bluramm*2+1){
  for(int j=bluramm;j<came.width-bluramm;j+=bluramm*2+1){
  //float ared=red(came.pixels[i*came.width+j]),agreen=green(came.pixels[i*came.width+j]),ablue=blue(came.pixels[i*came.width+j]);
  float ared=0,agreen=0,ablue=0;
  for(int h=-bluramm;h<=bluramm;h++){
   for(int k=-bluramm;k<=bluramm;k++){
  ared+=red(ret.pixels[(i+h)*came.width+j+k]);
  agreen+=green(ret.pixels[(i+h)*came.width+j+k]);
  ablue+=blue(ret.pixels[(i+h)*came.width+j+k]);
  
  }
  
  
  }
  ared/=(bluramm*2+1)*(bluramm*2+1);
    agreen/=(bluramm*2+1)*(bluramm*2+1);
    ablue/=(bluramm*2+1)*(bluramm*2+1);
    
     for(int h=-bluramm;h<=bluramm;h++){
   for(int k=-bluramm;k<=bluramm;k++){
ret.pixels[(i+h)*came.width+j+k]=color(ared,agreen,ablue);
 
  
  }
  
  
  }
    
  //  ret.pixels[i*came.width+j]=color(ared,agreen,ablue);
  
  
  }
  
  
  }
    //blurend
    int blobx=0,bloby=0,count=0;
  for(int i=bluramm;i<came.height-bluramm;i+=bluramm*2+1){
  for(int j=bluramm;j<came.width-bluramm;j+=bluramm*2+1){
    //ret.pixels[i]=color(ceil(red(came.pixels[i])/dev)*dev,ceil(green(came.pixels[i])/dev)*dev,ceil(blue(came.pixels[i])/dev)*dev);
    float mindist=80;
    
    
    
    
    
    for (color c:colors){
      if(dist(red(ret.pixels[i*ret.width+j]),green(ret.pixels[i*ret.width+j]),blue(ret.pixels[i*ret.width+j]),red(c),green(c),blue(c))<mindist){
       mindist= dist(red(ret.pixels[i*ret.width+j]),green(ret.pixels[i*ret.width+j]),blue(ret.pixels[i*ret.width+j]),red(c),green(c),blue(c));
    //ret.pixels[i]=c;
       for(int h=-bluramm;h<=bluramm;h++){
   for(int k=-bluramm;k<=bluramm;k++){
ret.pixels[(i+h)*ret.width+j+k]=c;
 
  
  }
  
  
  }
    
    }
      
    }
    if(red(ret.pixels[i*ret.width+j])==90&&green(ret.pixels[i*ret.width+j])==200&&blue(ret.pixels[i*ret.width+j])==90){
    blobx+=j;
    bloby+=i;
    count++;
      
    
    }
    
    
    
  }}
  
  if(count!=0){
   blobx/=count;
   bloby/=count;
   
   ellipse(blobx,bloby,30,30);
    if(blobx-320>30)turn(0,8000);
    else if (320-blobx>30)turn(1,8000);
    else stopM();
    
  }
  
  
  
  
ret.updatePixels();


return ret;




}
void mousePressed(){
  /*cam.loadPixels();
  print(red(cam.pixels[cam.width*mouseY+mouseX]));
  print(" ");
  print(green(cam.pixels[cam.width*mouseY+mouseX]));
    print(" ");
  print(blue(cam.pixels[cam.width*mouseY+mouseX]));
    println(" ");
  */
  //x1=mouseX;
  //y1=mouseY;
  loadPixels();
  
  print("red : ");
  println(red(pixels[mouseX+mouseY*width]));
 
  
   print("green : ");
  println(green(pixels[mouseX+mouseY*width]));
 
  
   print("blue : ");
  println(blue(pixels[mouseX+mouseY*width]));
  
  
  
  
}


void mouseReleased(){
  
  //x2=mouseX;
  //y2=mouseY;
  //int minr=1000;
  //  int ming=1000;
  //    int minb=1000;
  //      int maxr=0;
  //        int maxg=0;
  //          int maxb=0;
  
  //for(int i=int(cam.width*y1+x1);i<cam.width*y2+x2;i++){
  //  minr=min(minr,int(red(cam.pixels[i])));
  //  ming=min(ming,int(green(cam.pixels[i])));
  //  minb=min(minb,int(blue(cam.pixels[i])));
  //  maxr=max(maxr,int(red(cam.pixels[i])));
  //   maxg=max(maxg,int(green(cam.pixels[i])));
  //    maxb=max(maxb,int(blue(cam.pixels[i])));
  //}
  //print("red : ");
  //print(minr);
  //print("-");
  //println(maxr);
  
  // print("green : ");
  //print(ming);
  //print("-");
  //println(maxg);
  
  // print("blue : ");
  //print(minb);
  //print("-");
  //println(maxb);
  
  
  
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
