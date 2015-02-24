/**
Processing GRT Example
Version: Development version 1
Author: Nick Gillian

Info: This sketch demonstrates how to use Processing with the GRT GUI. 
You can find out more information about how to use this code here: 

http://www.nickgillian.com/wiki/pmwiki.php/GRT/GUIProcessing
*/

//Import the P5 OSC library
import oscP5.*;
import netP5.*;
import com.shigeodayo.ardrone.processing.*;
import java.lang.*;
import KinectPV2.KJoint;
import KinectPV2.*;

KinectPV2 kinect;
Skeleton [] skeleton;
float zVal = 300;
float rotX = PI;

//by puching
ARDroneForP5 ardrone;
String keyValue="";
long preTime = 0;
long nowTime = 0;
boolean moving = false;
boolean open = false;
boolean allowScreenController = false;
//by puching


//Set the pipeline mode (CLASSIFICATION_MODE or REGRESSION_MODE), the number of inputs and the number of outputs
final int pipelineMode = GRT.CLASSIFICATION_MODE;
final int numInputs = 14;
final int numOutputs = 1;

//Create a new GRT instance, this will initalize everything for us and send the setup message to the GRT GUI
GRT grt = new GRT( pipelineMode, numInputs, numOutputs, "127.0.0.1", 5000, 5001, true );

//Create some global variables to hold our data
float[] data = new float[ numInputs ];
float[] targetVector = new float[ numOutputs ];
PFont font;

void setup() {
  size(960,960, P3D);
  frameRate(30);
  
  //Load the font
  font = loadFont("SansSerif-48.vlw");

  kinect = new KinectPV2(this);
  kinect.enableColorImg(true);
  kinect.enableSkeleton(true );
  //enable 3d Skeleton with (x,y,z) position
  kinect.enableSkeleton3dMap(true);
  kinect.init();
  
  //by puching
//  ardrone=new ARDroneForP5("192.168.1.1", ARDroneVersion.ARDRONE2);
//  // connect to the AR.Drone
//  ardrone.connect();
//  // for getting sensor information
//  ardrone.connectNav();
//  // for getting video informationp
//  ardrone.connectVideo();
//  // start to control AR.Drone and get sensor and video data of it
//  ardrone.start();
  //by puching
}

void draw() {
  background(0);  
  
  image(kinect.getColorImage(), 0, 0, 300, 300);

  skeleton =  kinect.getSkeleton3d();

  //translate the scene to the center 
  pushMatrix();
  translate(width/2, height/2, 0);
  scale(zVal);
  rotateX(rotX);
  
  if( !grt.getInitialized() ){
    background(255,0,0);  
    println("WARNING: GRT Not Initalized. You need to call the setup function!");
    return;
  }
  
  for (int i = 0; i < skeleton.length; i++) {
    if (skeleton[i].isTracked()) {
      KJoint[] joints = skeleton[i].getJoints();
      // print(Float.toString(joints[KinectPV2.JointType_HandRight].getX()) + " ");
      // print(Float.toString(joints[KinectPV2.JointType_HandRight].getY()) + " ");
      // println(Float.toString(joints[KinectPV2.JointType_HandRight].getZ()));
        
      //Draw the info text
      grt.drawInfoText(20,20);
      
      //draw different color for each hand state
      drawHandState(joints[KinectPV2.JointType_HandRight]);
      drawHandState(joints[KinectPV2.JointType_HandLeft]);

      //Draw body
      color col  = getIndexColor(i);
      stroke(col);
      drawBody(joints);
      
      //Draw the info text
      grt.drawInfoText(20,20);
      
      //Grab the mouse data and send it to the GRT backend via OSC
      data[0] = joints[KinectPV2.JointType_HandRight].getX();
      data[1] = joints[KinectPV2.JointType_HandRight].getY();
      data[2] = joints[KinectPV2.JointType_HandRight].getZ();
      data[3] = joints[KinectPV2.JointType_HandLeft].getX();
      data[4] = joints[KinectPV2.JointType_HandLeft].getY();
      data[5] = joints[KinectPV2.JointType_HandLeft].getZ();

      data[6] = joints[KinectPV2.JointType_ElbowRight].getX();
      data[7] = joints[KinectPV2.JointType_ElbowRight].getY();
 
      data[8] = joints[KinectPV2.JointType_ElbowLeft].getX();
      data[9] = joints[KinectPV2.JointType_ElbowLeft].getY();

      data[10] = joints[KinectPV2.JointType_KneeRight].getX();
      data[11] = joints[KinectPV2.JointType_KneeRight].getY();
 
      data[12] = joints[KinectPV2.JointType_KneeLeft].getX();
      data[13] = joints[KinectPV2.JointType_KneeLeft].getY();
      grt.sendData( data );
    }
  }
  popMatrix();


  fill(255, 0, 0);
  text(frameRate, 50, 50);
  
  if(open){
    if (grt.getPredictedClassLabel()==1) {
      print("stop\n");
      nowTime = System.currentTimeMillis() / 1000;
      try {
        if (keyValue.equals("UP")) {
          ardrone.backward(100);
        }
        else if (keyValue.equals("DOWN")) {
          ardrone.forward(100);
        }
        else if (keyValue.equals("LEFT")) {
          ardrone.goRight(100);
        }
        else if (keyValue.equals("RIGHT")) {
          ardrone.goLeft(100);
        }
        else {
          preTime = nowTime;
        } 
        Thread.sleep((nowTime-preTime)*300);
        keyValue = "";
      } catch (Exception e) {}
      
      ardrone.stop();
    }
    else if (grt.getPredictedClassLabel()==2) {
      print("take off\n");
      ardrone.takeOff();
    }
    else if (grt.getPredictedClassLabel()==3) {
      print("landing\n");
      ardrone.landing();
    }
    else if (grt.getPredictedClassLabel()==4) {
      print("up\n");
      ardrone.up(100); 
    }
    else if (grt.getPredictedClassLabel()==5) {
      print("down\n");
      ardrone.down(100);
    }
    else if (grt.getPredictedClassLabel()==6) {
      print("forward\n");
      ardrone.forward(linearSpeed());
    }
    else if (grt.getPredictedClassLabel()==7) {
      print("backward\n");
      ardrone.backward(linearSpeed());
    }
    else if (grt.getPredictedClassLabel()==8) {
      print("left\n");
      ardrone.goLeft(linearSpeed());
    }
    else if (grt.getPredictedClassLabel()==9) {
      print("right\n");
      ardrone.goRight(linearSpeed());
    }
  }
  //by puching

//  PImage img = ardrone.getVideoImage(false);
//  if (img == null)
//    return;
//  image(img, 400, 450);
//
//  // print out AR.Drone information
////  ardrone.printARDroneInfo();
//
//  // getting sensor information of AR.Drone
//  float pitch = ardrone.getPitch();
//  float roll = ardrone.getRoll();
//  float yaw = ardrone.getYaw();
//  float altitude = ardrone.getAltitude();
//  float[] velocity = ardrone.getVelocity();
//  int battery = ardrone.getBatteryPercentage();
//
//  String attitude = "pitch:" + pitch + "\nroll:" + roll + "\nyaw:" + yaw + "\naltitude:" + altitude;
//  text(attitude, 20, 485);
//  String vel = "vx:" + velocity[0] + "\nvy:" + velocity[1];
//  text(vel, 20, 570);
//  String bat = "battery:" + battery + " %";
//  text(bat, 20, 620);
  //by puching

}

int linearSpeed() {
  long deltaTime = System.currentTimeMillis()/1000 - preTime;
  if (deltaTime < 2) {
    return int(deltaTime)*5+20;
    // return int(deltaTime)*3+20;
  }
  else {
    return 30;
  }
}

void move(String s){
  keyValue = s;
  moving = true;
  preTime = System.currentTimeMillis() / 1000;
}

//use different color for each skeleton tracked
color getIndexColor(int index) {
  color col = color(255);
  if (index == 0)
    col = color(255, 0, 0);
  if (index == 1)
    col = color(0, 255, 0);
  if (index == 2)
    col = color(0, 0, 255);
  if (index == 3)
    col = color(255, 255, 0);
  if (index == 4)
    col = color(0, 255, 255);
  if (index == 5)
    col = color(255, 0, 255);

  return col;
}


void drawBody(KJoint[] joints) {
  drawBone(joints, KinectPV2.JointType_Head, KinectPV2.JointType_Neck);
  drawBone(joints, KinectPV2.JointType_Neck, KinectPV2.JointType_SpineShoulder);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_SpineMid);


  drawBone(joints, KinectPV2.JointType_SpineMid, KinectPV2.JointType_SpineBase);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_ShoulderRight);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_ShoulderLeft);
  drawBone(joints, KinectPV2.JointType_SpineBase, KinectPV2.JointType_HipRight);
  drawBone(joints, KinectPV2.JointType_SpineBase, KinectPV2.JointType_HipLeft);

  // Right Arm    
  drawBone(joints, KinectPV2.JointType_ShoulderRight, KinectPV2.JointType_ElbowRight);
  drawBone(joints, KinectPV2.JointType_ElbowRight, KinectPV2.JointType_WristRight);
  drawBone(joints, KinectPV2.JointType_WristRight, KinectPV2.JointType_HandRight);
  drawBone(joints, KinectPV2.JointType_HandRight, KinectPV2.JointType_HandTipRight);
  drawBone(joints, KinectPV2.JointType_WristRight, KinectPV2.JointType_ThumbRight);

  // Left Arm
  drawBone(joints, KinectPV2.JointType_ShoulderLeft, KinectPV2.JointType_ElbowLeft);
  drawBone(joints, KinectPV2.JointType_ElbowLeft, KinectPV2.JointType_WristLeft);
  drawBone(joints, KinectPV2.JointType_WristLeft, KinectPV2.JointType_HandLeft);
  drawBone(joints, KinectPV2.JointType_HandLeft, KinectPV2.JointType_HandTipLeft);
  drawBone(joints, KinectPV2.JointType_WristLeft, KinectPV2.JointType_ThumbLeft);

  // Right Leg
  drawBone(joints, KinectPV2.JointType_HipRight, KinectPV2.JointType_KneeRight);
  drawBone(joints, KinectPV2.JointType_KneeRight, KinectPV2.JointType_AnkleRight);
  drawBone(joints, KinectPV2.JointType_AnkleRight, KinectPV2.JointType_FootRight);

  // Left Leg
  drawBone(joints, KinectPV2.JointType_HipLeft, KinectPV2.JointType_KneeLeft);
  drawBone(joints, KinectPV2.JointType_KneeLeft, KinectPV2.JointType_AnkleLeft);
  drawBone(joints, KinectPV2.JointType_AnkleLeft, KinectPV2.JointType_FootLeft);

  drawJoint(joints, KinectPV2.JointType_HandTipLeft);
  drawJoint(joints, KinectPV2.JointType_HandTipRight);
  drawJoint(joints, KinectPV2.JointType_FootLeft);
  drawJoint(joints, KinectPV2.JointType_FootRight);

  drawJoint(joints, KinectPV2.JointType_ThumbLeft);
  drawJoint(joints, KinectPV2.JointType_ThumbRight);

  drawJoint(joints, KinectPV2.JointType_Head);
}

void drawJoint(KJoint[] joints, int jointType) {
  strokeWeight(2.0f + joints[jointType].getZ()*8);
  point(joints[jointType].getX(), joints[jointType].getY(), joints[jointType].getZ());
}

void drawBone(KJoint[] joints, int jointType1, int jointType2) {
  strokeWeight(2.0f + joints[jointType1].getZ()*8);
  point(joints[jointType2].getX(), joints[jointType2].getY(), joints[jointType2].getZ());
}

void drawHandState(KJoint joint) {
  handState(joint.getState());
  strokeWeight(5.0f + joint.getZ()*8);
  point(joint.getX(), joint.getY(), joint.getZ());
}

void handState(int handState) {
  switch(handState) {
  case KinectPV2.HandState_Open:
    stroke(0, 255, 0);
    break;
  case KinectPV2.HandState_Closed:
    stroke(255, 0, 0);
    break;
  case KinectPV2.HandState_Lasso:
    stroke(0, 0, 255);
    break;
  case KinectPV2.HandState_NotTracked:
    stroke(100, 100, 100);
    break;
  }
}



//getPredictedClassLabel()

void keyPressed() {
//  if (key == CODED) {
//    // preKeyCode = keyCode;
//    if (keyCode == UP && !keyValue.equals("UP")) {
//      move("UP");
//    } 
//    else if (keyCode == DOWN && !keyValue.equals("DOWN")) {
//      move("DO WN");
//    } 
//    else if (keyCode == LEFT && !keyValue.equals("LEFT")) {
//      move("LEFT");
//    } 
//    else if (keyCode == RIGHT && !keyValue.equals("RIGHT")) {
//      move("RIGHT");
//    } 
//    else if (keyCode == SHIFT) {
//      moving = false;
//      ardrone.takeOff(); // take off, AR.Drone cannot move while landing
//      keyValue = "";
//    } 
//    else if (keyCode == CONTROL) {
//      moving = false;
//      ardrone.landing();
//      keyValue = "";
//      // landing
//    }
//  } 
//  else {
//    // preKey = key;
//    moving = false;
//    if (key == 's') {
//      nowTime = System.currentTimeMillis() / 1000;
//      try {
//        if (keyValue.equals("UP")) {
//          ardrone.backward(100);
//        }
//        else if (keyValue.equals("DOWN")) {
//          ardrone.forward(100);
//        }
//        else if (keyValue.equals("LEFT")) {
//          ardrone.goRight(100);
//        }
//        else if (keyValue.equals("RIGHT")) {
//          ardrone.goLeft(100);
//        }
//        else {
//          preTime = nowTime;
//        } 
//        Thread.sleep((nowTime-preTime)*300);
//        keyValue = "";
//      } catch (Exception e) {}
//      
//      ardrone.stop(); // hovering
//
// 
//    }
//       
//    else {
//      keyValue = "";
//      if (key == 'r') {
//        // moving = true;
//        ardrone.spinRight(); // spin right
//      } 
//      else if (key == 'e') {  
//        // moving = true;
//        ardrone.spinLeft(); // spin left
//      } 
//      else if (key == 't') {
//        ardrone.up(100); // go up
//      }
//      else if (key == 'g') {
//        ardrone.down(100); // go down
//      }
//    }
//    
//  }
  switch( key ){
    case '0':
      allowScreenController = true;
    case 'i':
      grt.init( pipelineMode, numInputs, numOutputs, "127.0.0.1", 5000, 5001, true );
      break;
    case '[':
      grt.decrementTrainingClassLabel();
      break;
    case ']':
      grt.incrementTrainingClassLabel();
      break;
    case 'r':
      if( grt.getRecordingStatus() ){
        grt.stopRecording();
      }else grt.startRecording();
      break;
    case 't':
      grt.startTraining();
      break;
    case 's':
      grt.saveTrainingDatasetToFile( "TrainingData.txt" );
      break;
    case 'l':
      grt.loadTrainingDatasetFromFile( "TrainingData.txt" );
      break;
    case 'c':
      grt.clearTrainingDataset();
    break;
    case '{': //Decrease the target vector value by 0.1 (only for REGRESSION_MODE)
      targetVector[0] -= 0.1;
      grt.sendTargetVector( targetVector );
    break;
    case '}': //Increase the target vector value by 0.1 (only for REGRESSION_MODE)
      targetVector[0] += 0.1;
      grt.sendTargetVector( targetVector );
    break;
    case '1': //Set the classifier as ANBC, enable scaling, enable null rejection, and set the null rejection coeff to 5.0
      grt.setClassifier( grt.ANBC, true, true, 5.0 );
    break;
    case '2'://Set the classifier as ADABOOST, enable scaling, disable null rejection, and set the null rejection coeff to 5.0
      grt.setClassifier( grt.ADABOOST, true, false, 5.0 );
    break;
    case 'o':
      if(open)
        open = false;
      else
        open = true;
    break;
    default:
      break;
  }
}
