#include <Wire.h>
#include <DynamixeliR.h>
#include <SoftHalfDuplexSerial.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"
#include <SharpIR.h>

#define sag A0
#define sol A3
#define onSag A1
#define onSol A2
#define model 20150
#define ldr A5
// Her sensör için ayrı bir nesne oluştur
SharpIR ir_onSol(onSol, model);
SharpIR ir_onSag(onSag, model);
SharpIR ir_sol(sol, model);
SharpIR ir_sag(sag, model);


#define redpin 3
#define greenpin 5
#define bluepin 6
#define commonAnode true
float red, green, blue;
int r, g, b, c;
// our RGB -> eye-recognized gamma color
byte gammatable[256];

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

int driveTrainIndex[2][2] = { { 3, 4 }, { 1, 2 } };  // sol, sag (ön, arka)
int leftMotorsIndex[2] = { driveTrainIndex[0][0], driveTrainIndex[0][1] };
int rightMotorsIndex[2] = { driveTrainIndex[1][0], driveTrainIndex[1][1] };
// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to ground
// set to false if using a common cathode LED
//SPEEDS
const int forwardLeft = 520;
const int forwardRight = forwardLeft + 1023;
const int backwardsLeft = 1524;
const int backwardsRight = backwardsLeft - 1023;
const int intakeSpeed = 700 + 1023;
const double wheelBase = 27.675 / 2;
unsigned int eskiZaman;
unsigned int yeniZaman;
//-----Dynamixel------
softHalfDuplexSerial port(6);
dxliR dxlCom(&port);
//-----Dynamixel End--
int yesilkapaksayisi = 0;
int istenenkapaksayisi = 0;
int istenmeyenkapaksayisi = 0;
int maxistenmeyen=4;
int maxgreen=3;
int maxistenen=5;
String startZemin = "blue";
#define backHolder 8
#define frontDoor 10
int bagajIndex[3] = { 14, 6, 16 };  //red, green, blue (11, 7, 6)
int redBagajIndex = 0;
const int greenBagajIndex = 1;
int blueBagajIndex = 2;

String color = "";
String zemin = "";

float rb7_90 = 530.0;
float bb11_90 = 400.0;  // !!! NO INFO
const float gb6_90 = 512.0;
float rb7_180 = 200.0;
float bb11_180 = 850.0;  // !!! NO INFO
const float gb6_180 = 200.0;
//const float startFazladanAci = 188;
void setup() {

  Serial.begin(9600);
  Serial.print("Basladi");
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1) {}
  }
  for (int i = 0; i < 256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
    Serial.println(gammatable[i]);
  }

  dxlCom.begin(57600);
  //Serial.begin(9600);
  Wire.begin();
  //prepareGyro();
  torkOn();
  startBagaj();
  frontHolderClosed();
  backHolderClosed();
  frontDoorOpen();
  istenenBagajClosed();
  greenBagajClosed();
  istenmeyenBagajClosed();
  //Renksensoru

  // use these three pins to drive an LED


  // thanks PhilB for this gamma table!
  // it helps convert RGB colors to what humans see
  renkOku();
  if((red - green > 70) && (red - blue > 75)){
    startZemin="red";
Serial.println("Kırmızı zemin okudum kırmızı kapak topluyorum.");
    
  }
  else{
    startZemin="blue";
Serial.println("Mavi zemin okudum kırmızı kapak topluyorum.");
  }
}

void loop() {
  renkOku();
  int isik = analogRead(ldr);
//Serial.print("ldr değer: ");
  //Serial.println(ldr);
  int dis_onSol = ir_onSol.distance();
  int dis_onSag = ir_onSag.distance();
  int dis_sol = ir_sol.distance();
  int dis_sag = ir_sag.distance();
  //Serial.println(color);
  intake();
  mesafeOku();
  if (ir_sag.distance()  < 25) {
    //Serial.println("Sağımda engel görüyorum");
    Stop();
    // turnRightTimed(400);
    turnLeftDelayed();
  } else if (ir_onSol.distance() <  80) {
    //Serial.println("Önümde engel görüyorum");
    Stop();
   
    //turnRightTimed(400);
    turnLeftDelayed();
  } else if (ir_onSag.distance() <  80) {
    //Serial.println("Önümde engel görüyorum");
    Stop();
    //backwardsTimed(160);
    
    //turnLeftTimed(400);
    turnLeftDelayed();

  } else if (ir_sol.distance() <  22) {
    //Serial.println("solumda engel görüyorum");
    Stop();
    // turnRightTimed(360);
    turnRightDelayed();
  } else {
    renkOku();
    if (startZemin == "blue") {

      if ((red - green > 70) && (red - blue > 75) && c > 120) {
        color = "Red";
        frontDoorClosed();
        Stop();
        Serial.println("kırmızı");
        if(istenmeyenkapaksayisi==maxistenmeyen){
          istenmeyenBagajOpen();
          istenmeyenkapaksayisi=0;
        }
        frontDoorClosed();
        frontHolderLeft();
        backHolderLeft();
        forwardTimed(310);
        istenmeyenkapaksayisi++;
        frontHolderClosed();
        backHolderClosed();
        frontDoorOpen();
      } else if ((blue - green > 7) && (blue - red > 18) && c > 120) {
        color = "Blue";
        Serial.println("mavi");
         frontDoorClosed();
        Stop();
        if(istenenkapaksayisi==maxistenen){
         
              frontHolderLeft();
        backHolderLeft();
        forwardTimed(310);
        istenmeyenkapaksayisi++;
         frontHolderClosed();
        backHolderClosed();
        frontDoorOpen();
        }else if(istenenkapaksayisi < maxistenen){

        frontHolderRight();
        istenenkapaksayisi++;
        forwardTimed(310);
        frontHolderClosed();
        backHolderClosed();
        frontDoorOpen();
        }
      } 
       else if ((green - blue > 20) && (green - red > 20) && c > 120) {
        color = "Green";
        Serial.println("yesil");
        frontDoorClosed();
        Stop();
          if(yesilkapaksayisi==maxgreen){
         
              frontHolderLeft();
        backHolderLeft();
        forwardTimed(310);
        istenmeyenkapaksayisi++;
         frontHolderClosed();
        backHolderClosed();
        frontDoorOpen();
        }
        else if(yesilkapaksayisi<maxgreen){
        frontHolderLeft();
        backHolderRight();
        forwardTimed(330);
        yesilkapaksayisi++;
        frontHolderClosed();
        backHolderClosed();
        frontDoorOpen();
        }
      }
       else if ((blue - green > 7) && (blue - red > 18) && c < 120 && istenenkapaksayisi > 0) {
        zemin = "Blue";
        Stop();
        
        Serial.println("Blue Zemin");
       // forwarddelay(200);
        
        // delay(2000);
        mesafeOku();
         if (dis_sag < 140) {  // Sağ taraf yaklaştıysa, zeminde
 unsigned long previousMillis = millis();
  const unsigned long interval = 50;  // Sensörleri her 50 ms'de bir kontrol et
  
  while (true) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      
      mesafeOku();  // Sensörleri oku
      turnLeft();   // Sürekli dön

      if (ir_onSol.distance() > 320 && ir_onSag.distance() > 320) {
        Stop();
        istenenBagajOpen();
        istenenkapaksayisi = 0;
        forwarddelay(750);
        istenenBagajClosed();
        break; // İş bitince while'dan çık
      }
    }
  }
}
else  {
        unsigned long previousMillis = millis();
  const unsigned long interval = 50;  // Sensörleri her 50 ms'de bir kontrol et
  
  while (true) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      
      mesafeOku();  // Sensörleri oku
      turnRight();   // Sürekli dön

      if (ir_onSol.distance() > 320 && ir_onSag.distance() > 320) {
        Stop();
        istenenBagajOpen();
        istenenkapaksayisi = 0;
        forwarddelay(750);
        istenenBagajClosed();
        break; // İş bitince while'dan çık
      }
    }
  }
        }
      }

      else if ((green - blue > 20) && (green - red > 20) && c < 120 && yesilkapaksayisi > 0) {
        zemin = "Green";
        Stop();
        Serial.println("Green Zemin");
        //forwarddelay(100);
        Stop();

        
        // delay(2000);
        mesafeOku();
       if (dis_sag < 140) {  // Sağ taraf yaklaştıysa, zeminde
  unsigned long previousMillis = millis();
  const unsigned long interval = 50;  // Sensörleri her 50 ms'de bir kontrol et
  
  while (true) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      
      mesafeOku();  // Sensörleri oku
      turnLeft();   // Sürekli dön

      if (ir_onSol.distance() > 320 && ir_onSag.distance() > 320) {
        Stop();
        greenBagajOpen();
        yesilkapaksayisi = 0;
        forwarddelay(750);
        greenBagajClosed();
        break; // İş bitince while'dan çık
      }
    }
  }
}
 else {
  unsigned long previousMillis = millis();
  const unsigned long interval = 50;  // Sensörleri her 50 ms'de bir kontrol et
  
  while (true) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      
      mesafeOku();  // Sensörleri oku
      turnRight();   // Sürekli dön

      if (ir_onSol.distance() > 320 && ir_onSag.distance() > 320) {
        Stop();
        greenBagajOpen();
        yesilkapaksayisi = 0;
        forwarddelay(750);
        greenBagajClosed();
        break; // İş bitince while'dan çık
      }
    }
  }
          
        }
      }

      else {
        frontDoorOpen();
         frontHolderClosed();
  backHolderClosed();
        forward();
        mesafeOku();
        renkOku();
        color = "White";
      }

    } else {  // Eğer Başlangıç zemini Kırmızı ise
      if ((red - green > 70) && (red - blue > 75) && c > 120) {
        color = "Red";
        frontDoorClosed();
        Stop();
      if(istenenkapaksayisi==maxistenen){
         
              frontHolderLeft();
        backHolderLeft();
        forwardTimed(310);
        istenmeyenkapaksayisi++;
         frontHolderClosed();
        backHolderClosed();
        frontDoorOpen();
        }
        else if(istenenkapaksayisi < maxistenen){

        frontHolderRight();
        istenenkapaksayisi++;
        forwardTimed(310);
        frontHolderClosed();
        backHolderClosed();
        frontDoorOpen();
        }
        //Serial.println("kırmızı");
       
      } else if ((green - blue > 20) && (green - red > 20) && c > 120) {
        color = "Green";
        //Serial.println("yesil");
        frontDoorClosed();
        Stop();
          if(yesilkapaksayisi==maxgreen){
         
              frontHolderLeft();
        backHolderLeft();
        forwardTimed(310);
        istenmeyenkapaksayisi++;
         frontHolderClosed();
        backHolderClosed();
        frontDoorOpen();
        }
        else if(yesilkapaksayisi<maxgreen){
        frontHolderLeft();
        backHolderRight();
        forwardTimed(330);
        yesilkapaksayisi++;
        frontHolderClosed();
        backHolderClosed();
        frontDoorOpen();
        }
      } else if ((blue - green > 7) & (blue - red > 18) & c > 120) {
        color = "Blue";
        //Serial.println("mavi");
        Stop();
        frontDoorClosed();
        frontHolderLeft();
        backHolderLeft();
        forwarddelay(310);
        istenmeyenkapaksayisi++;
        Stop();
        frontHolderClosed();
        backHolderClosed();
        frontDoorOpen();
      } 
      
      else if ((red - green > 70) & (red - blue > 75) & c<85 & istenenkapaksayisi > 0) {
        zemin = "Red";
        Stop();
        //Serial.println("Kırmızı Zemin");
        mesafeOku();
        if (dis_sag < 140) {  // Sağ taraf yaklaştıysa, zeminde
 unsigned long previousMillis = millis();
  const unsigned long interval = 50;  // Sensörleri her 50 ms'de bir kontrol et
  
  while (true) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      
      mesafeOku();  // Sensörleri oku
      turnLeft();   // Sürekli dön

      if (ir_onSol.distance() > 320 && ir_onSag.distance() > 320) {
        Stop();
        istenenBagajOpen();
        istenenkapaksayisi = 0;
        forwarddelay(750);
        istenenBagajClosed();
        break; // İş bitince while'dan çık
      }
    }
  }
}
       else {
          //backwardsTimed(100);
          unsigned long previousMillis = millis();
  const unsigned long interval = 50;  // Sensörleri her 50 ms'de bir kontrol et
  
  while (true) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      
      mesafeOku();  // Sensörleri oku
      turnRight();   // Sürekli dön

      if (ir_onSol.distance() > 320 && ir_onSag.distance() > 320) {
        Stop();
        istenenBagajOpen();
        istenenkapaksayisi = 0;
        forwarddelay(750);
        istenenBagajClosed();
        break; // İş bitince while'dan çık
      }
    }
  }
        }
      }   else if ((green - blue > 20) && (green - red > 20) && c < 120 && yesilkapaksayisi > 0) {
        zemin = "Green";
        Stop();
        //Serial.println("Green Zemin");
        //forwarddelay(100);
        Stop();

        
        // delay(2000);
        mesafeOku();
       if (dis_sag < 140) {  // Sağ taraf yaklaştıysa, zeminde
  unsigned long previousMillis = millis();
  const unsigned long interval = 50;  // Sensörleri her 50 ms'de bir kontrol et
  
  while (true) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      
      mesafeOku();  // Sensörleri oku
      turnLeft();   // Sürekli dön

      if (ir_onSol.distance() > 320 && ir_onSag.distance() > 320) {
        Stop();
        greenBagajOpen();
        yesilkapaksayisi = 0;
        forwarddelay(750);
        greenBagajClosed();
        break; // İş bitince while'dan çık
      }
    }
  }
}
 else {
  unsigned long previousMillis = millis();
  const unsigned long interval = 50;  // Sensörleri her 50 ms'de bir kontrol et
  
  while (true) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      
      mesafeOku();  // Sensörleri oku
      turnRight();   // Sürekli dön

      if (ir_onSol.distance() > 320 && ir_onSag.distance() > 320) {
        Stop();
        greenBagajOpen();
        yesilkapaksayisi = 0;
        forwarddelay(750);
        greenBagajClosed();
        break; // İş bitince while'dan çık
      }
    }
  }
          
        }
      }else {
        frontDoorOpen();
         frontHolderClosed();
  backHolderClosed();
        forward();
        mesafeOku();
        renkOku();
        color = "White";
      }
    }
  }
}


void intake() {
  dxlCom.setMovingSpeed(5, 1323);
}
void intakeStop() {
  dxlCom.setMovingSpeed(5, 0);
}



//TORK
void torkOn() {
  dxlCom.setTorqueEnable(1, 1);

  while (!dxlCom.dxlDataReady())
    ;

  dxlCom.setTorqueEnable(2, 1);

  while (!dxlCom.dxlDataReady())
    ;

  dxlCom.setTorqueEnable(3, 1);

  while (!dxlCom.dxlDataReady())
    ;

  dxlCom.setTorqueEnable(4, 1);

  while (!dxlCom.dxlDataReady())
    ;

  dxlCom.setTorqueEnable(5, 1);

  while (!dxlCom.dxlDataReady())
    ;

  dxlCom.setTorqueEnable(6, 1);

  while (!dxlCom.dxlDataReady())
    ;

  dxlCom.setTorqueEnable(8, 1);

  while (!dxlCom.dxlDataReady())
    ;

  dxlCom.setTorqueEnable(9, 1);

  while (!dxlCom.dxlDataReady())
    ;

  dxlCom.setTorqueEnable(10, 1);

  while (!dxlCom.dxlDataReady())
    ;

  dxlCom.setTorqueEnable(11, 1);

  while (!dxlCom.dxlDataReady())
    ;
  dxlCom.setTorqueEnable(16, 1);

  while (!dxlCom.dxlDataReady())
    ;
}

//DRIVETRAIN
void forward() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], forwardRight);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], forwardRight);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], forwardLeft);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], forwardLeft);
  //Serial.print("Sol motor hızım:");
  //Serial.print(forwardLeft);
  //Serial.print("Sağ motor hızım:");
  //Serial.print(forwardRight);
}
void forwarddelay(int delayer) {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], forwardRight + 400);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], forwardRight + 400);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], forwardLeft + 400);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], forwardLeft + 400);
  delay(delayer);
}
void Stop() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 0);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 0);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 0);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 0);
}


void turnLeft() {
   frontDoorClosed();
  frontHolderLeft();
        backHolderLeft();
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 1024 + 990);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 1024 + 990);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 1024 + 990);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 1024 + 990);
  
}
void turnLeftTimed(int time) {
  eskiZaman = millis();
  yeniZaman = eskiZaman;

  while (yeniZaman - eskiZaman <= time) {
    turnLeft();
    yeniZaman = millis();
  }

  Stop();
}
void forwardTimed(int time){
  eskiZaman = millis();
  yeniZaman = eskiZaman;

  while (yeniZaman - eskiZaman <= time) {
    forward();
    yeniZaman = millis();
  }

  Stop();
}

void turnRight() {
   frontDoorClosed();
  frontHolderLeft();
  backHolderLeft();
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 990);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 990);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 990);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 990);
}
void turnRightDelayed() {
   frontDoorClosed();
  frontHolderLeft();
        backHolderLeft();
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 990);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 990);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 990);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 990);
  
  delay(500);
}
void turnLeftDelayed() {
  frontDoorClosed();
  frontHolderLeft();
        backHolderLeft();
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 1024 + 990);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 1024 + 990);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 1024 + 990);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 1024 + 990);
  delay(500);
}

void turnRightTimed(int time) {
  eskiZaman = millis();
  yeniZaman = eskiZaman;

  while (yeniZaman - eskiZaman <= time) {
    turnRight();
    yeniZaman = millis();
  }
  Stop();
}
void backHolderLeft() {
  dxlCom.setGoalPosition(backHolder, 690);
  while (!dxlCom.dxlDataReady())
    ;
}
void frontDoorClosed() {
  dxlCom.setGoalPosition(frontDoor, 270);
  while (!dxlCom.dxlDataReady())
    ;
}
void frontDoorOpen() {
  dxlCom.setGoalPosition(frontDoor, 420);
  while (!dxlCom.dxlDataReady())
    ;
}
void backHolderClosed() {
  dxlCom.setGoalPosition(backHolder, 500);
  while (!dxlCom.dxlDataReady())
    ;
}

void backHolderRight() {
  dxlCom.setGoalPosition(backHolder, 340);
  while (!dxlCom.dxlDataReady())
    ;
}
void frontHolderLeft() {
  dxlCom.setGoalPosition(9, 645);
  while (!dxlCom.dxlDataReady())
    ;
}
void frontHolderRight() {
  dxlCom.setGoalPosition(9, 315);
  while (!dxlCom.dxlDataReady())
    ;
}
void startBagaj() {
  dxlCom.setGoalPosition(bagajIndex[redBagajIndex], 300);
  dxlCom.setGoalPosition(bagajIndex[greenBagajIndex], 700);
  //dxlCom.setGoalPosition(bagajIndex[blueBagajIndex], bb11_90 + startFazladanAci);
}
//BAGGAGE
void istenmeyenBagajClosed() {

  dxlCom.setGoalPosition(bagajIndex[redBagajIndex], rb7_180);
}

void istenenBagajClosed() {

  dxlCom.setGoalPosition(bagajIndex[blueBagajIndex], bb11_180);
}
void istenmeyenBagajOpen() {

  dxlCom.setGoalPosition(bagajIndex[redBagajIndex], rb7_90);
}

void istenenBagajOpen() {

  dxlCom.setGoalPosition(bagajIndex[blueBagajIndex], bb11_90);
}

void greenBagajClosed() {

  dxlCom.setGoalPosition(bagajIndex[greenBagajIndex], gb6_180);
  // gb = false;
}
void greenBagajOpen() {

  dxlCom.setGoalPosition(bagajIndex[greenBagajIndex], gb6_90);
  // gb = false;
}
//HOLDER
void frontHolderClosed() {
  dxlCom.setGoalPosition(9, 495);
  while (!dxlCom.dxlDataReady())
    ;
}
void mesafeOku() {
  // Her sensörden mesafeyi oku
  int dis_onSol = ir_onSol.distance();
  int dis_onSag = ir_onSag.distance();
  int dis_sol = ir_sol.distance();
  int dis_sag = ir_sag.distance();
delay(50);
  // Sonuçları yazdır
  //Serial.print("Ön Sol: ");
  //Serial.println(dis_onSol);

  //Serial.print("Ön Sağ: ");
  //Serial.println(dis_onSag);

  //Serial.print("Sol: ");
  //Serial.println(dis_sol);

  //Serial.print("Sağ: ");
  //Serial.println(dis_sag);
}

void renkOku() {
  tcs.setInterrupt(false);  // turn on LED

  tcs.getRGB(&red, &green, &blue);
  tcs.getRawData(&r, &g, &b, &c);

  tcs.setInterrupt(true);  // turn off LED

  Serial.print("R:\t");
  Serial.print(int(red));
  Serial.print("\tG:\t");
  Serial.print(int(green));
  Serial.print("\tB:\t");
  Serial.print(int(blue));
  Serial.print("\tC:\t");
  Serial.print(int(c));

  Serial.print("\n");
}


void backwards() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], backwardsRight);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], backwardsRight);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], backwardsLeft);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], backwardsLeft);
}
void backwardsDelayed() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], backwardsRight);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], backwardsRight);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], backwardsLeft);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], backwardsLeft);
  delay(300);
}
void backwardsTimed(int time) {
  eskiZaman = millis();
  yeniZaman = eskiZaman;
  while (yeniZaman - eskiZaman <= time) {
    backwards();
    yeniZaman = millis();
  }
  Stop();
}

