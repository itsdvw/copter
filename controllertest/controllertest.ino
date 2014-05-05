/*
 Example sketch for the Xbox 360 USB library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */
//arduino pwms = {11,10,9,6,5,3}
#include <XBOXUSB.h>
#include <Math.h>
                                        //         +Y 
                                        //          N
                                        //          | 
                                        //          |  
                                        //-X W----00000----E X+
                                        //          |
                                        //          | 
                                        //          S
                                        //          -Y
#define NUMMOTORS 4                                           
#define MOTORS 3                                   
//pins for directionals                            
#define N 9                                          
#define S 5                                        
#define E 6                                        
#define W 3                                         
//end pins                                         
#define MNOISE PI/20.0
#define MOTORS_OFF 20
#define MOTORS_MIN 25
#define MOTORS_MAX 254
//sensory magnitude for acknologing change in the joystick//
#define SENSITIVITY 7500
/*check for the x component of the joystick*/
#define LHatXCHECK (Xbox.getAnalogHat(LeftHatX) > SENSITIVITY || Xbox.getAnalogHat(LeftHatX) < -SENSITIVITY)
/*check for the y component of the joystick*/
#define LHatYCHECK (Xbox.getAnalogHat(LeftHatY) > SENSITIVITY || Xbox.getAnalogHat(LeftHatY) < -SENSITIVITY)


struct motorvec{
   float  north;
   float  south;
   float  east;
   float  west;
};

float normalvector[NUMMOTORS-1];
USB Usb;
XBOXUSB Xbox(&Usb);
void getNormal(float normal[]);
void setup() {
  pinMode(MOTORS, OUTPUT);
  //Set each directional motor for output//
  pinMode(N,OUTPUT);
  pinMode(S,OUTPUT);
  pinMode(E,OUTPUT);
  pinMode(W,OUTPUT);
  //end motor pins//
  //initialize the normal vector on start up//
  //getNormal(normalvector);
  Serial.begin(115200);
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
}


void loop() {
  Usb.Task();
  if (Xbox.Xbox360Connected) {
    if (Xbox.getButtonPress(R2)) {
      int in = Xbox.getButtonPress(R2);
      int out = map(in, 0, 255, MOTORS_MIN, MOTORS_MAX);
      Serial.print("R2: ");
      Serial.print(in);
      Serial.print("\tPWM: ");
      Serial.println(out);
      analogWrite(MOTORS, out);
     /*contribution by **David Vadney**/
     /*********************************/
     /*This block of code will poll the left joystick and get its normal vector*/
      if(LHatXCHECK || LHatYCHECK){
        int x = Xbox.getAnalogHat(LeftHatX);
        int y = Xbox.getAnalogHat(LeftHatY);
        float temp = sqrt(x*x + y*y); //find the radius
        Serial.print("X: ");
        Serial.print(x);
        Serial.print("\tY: ");
        Serial.print(y);
        Serial.print("\tvec: (");
        Serial.print(x/temp);
        Serial.print(",");
        Serial.print(y/temp);
        Serial.print(",");
        float norm = sqrt(normalvector[0]*normalvector[0] + normalvector[1]*normalvector[1] + normalvector[2]*normalvector[2]);
        temp = norm*cos(asin(temp/norm));
        Serial.println(temp);
         
      } 
     /*end of contribution*/  
 
  } else {
      analogWrite(MOTORS, MOTORS_OFF);
    }
  } else {
    analogWrite(MOTORS, MOTORS_OFF);
  }  
  delay(5);
}

/*IDK if we are doing code credit but contributed by **DAVID VADNEY**/

#define PITCH 0
#define ROLL  1
#define YAW   2
#define CORWEIGHT 0.5

#define PITCHCHECK (adjvec[PITCH] < noise && adjvec[PITCH] > -noise)
#define ROLLCHECK  (adjvec[ROLL] < noise && adjvec[ROLL] > -noise)
#define YAWCHECK   (adjvec[YAW] < noise && adjvec[YAW] > -noise)
#define PCHANGE    (adjvec[PITCH] > noise || adjvec[PITCH] < -noise)
#define RCHANGE    (adjvec[PITCH] > noise || adjvec[PITCH] < -noise)
#define YCHANGE    (adjvec[PITCH] > noise || adjvec[PITCH] < -noise)


void getAdjVector(float vec[]);
struct motorvec getMotorVector(){
    struct motorvec vector;
    float noise = (float)MNOISE;
    float adjvec[NUMMOTORS-1]; //vector to store the correctional cosines
    getAdjVector(adjvec);    //retrieve the correctional cosines
    
    /*If the current normal vector is within tolerance levels
     *then leave have all motors running at 100% OF INPUT*/
    if(PITCHCHECK && ROLLCHECK && YAWCHECK){
       vector.north = 1.0; vector.south = 1.0; vector.east = 1.0; vector.west = 1.0;
       return vector;
    } 
    if(PCHANGE || !(RCHANGE)){
      vector.north = 1.0 + CORWEIGHT*cos(adjvec[PITCH]);
      vector.south = 1.0 + -1*CORWEIGHT*cos(adjvec[PITCH]);
      vector.east = 1.0;
      vector.west = 1.0;
    }    
    else if(!PCHANGE || RCHANGE){
      vector.east = 1.0 + CORWEIGHT*cos(adjvec[ROLL]);
      vector.west = 1.0 + -1*CORWEIGHT*cos(adjvec[ROLL]);
      vector.north = 1.0;
      vector.south = 1.0;
    }else{
      vector.north = 1.0 + CORWEIGHT*cos(adjvec[PITCH])*(1-cos(adjvec[YAW]));
      vector.south = 1.0 + CORWEIGHT*cos(adjvec[PITCH])*(cos(adjvec[YAW])-1);
      vector.east =  1.0 + CORWEIGHT*cos(adjvec[ROLL])*(1-cos(adjvec[YAW]));
      vector.west =  1.0 + CORWEIGHT*cos(adjvec[PITCH])*(cos(adjvec[YAW])-1);
    }
    return vector;
}
/*End of contribution*/

