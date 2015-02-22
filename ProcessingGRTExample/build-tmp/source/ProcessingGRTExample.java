import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import oscP5.*; 
import netP5.*; 
import com.shigeodayo.ardrone.processing.*; 
import java.lang.*; 

import org.slf4j.helpers.*; 
import com.xuggle.xuggler.video.*; 
import org.apache.commons.net.chargen.*; 
import org.apache.commons.net.bsd.*; 
import org.apache.commons.net.discard.*; 
import org.apache.commons.net.nntp.*; 
import examples.unix.*; 
import com.shigeodayo.ardrone.command.*; 
import examples.telnet.*; 
import org.apache.commons.net.ftp.*; 
import org.apache.commons.net.finger.*; 
import org.apache.commons.net.ftp.parser.*; 
import org.apache.commons.net.ntp.*; 
import org.apache.commons.net.smtp.*; 
import com.shigeodayo.ardrone.*; 
import org.apache.commons.net.whois.*; 
import com.xuggle.ferry.*; 
import org.apache.commons.net.*; 
import org.apache.commons.net.io.*; 
import org.apache.commons.net.imap.*; 
import com.xuggle.mediatool.event.*; 
import examples.mail.*; 
import com.xuggle.mediatool.demos.*; 
import org.apache.commons.net.tftp.*; 
import com.xuggle.xuggler.io.*; 
import examples.ftp.*; 
import org.slf4j.impl.*; 
import examples.util.*; 
import org.apache.commons.net.time.*; 
import com.shigeodayo.ardrone.utils.*; 
import examples.ntp.*; 
import org.apache.commons.net.echo.*; 
import com.xuggle.xuggler.*; 
import com.shigeodayo.ardrone.processing.*; 
import com.shigeodayo.ardrone.video.*; 
import org.slf4j.*; 
import com.xuggle.mediatool.*; 
import examples.*; 
import examples.nntp.*; 
import org.apache.commons.net.pop3.*; 
import org.slf4j.spi.*; 
import com.shigeodayo.ardrone.manager.*; 
import org.apache.commons.net.telnet.*; 
import com.shigeodayo.ardrone.navdata.*; 
import org.apache.commons.net.util.*; 
import com.xuggle.xuggler.demos.*; 
import org.apache.commons.net.daytime.*; 
import examples.cidr.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class ProcessingGRTExample extends PApplet {

/**
Processing GRT Example
Version: Development version 1
Author: Nick Gillian

Info: This sketch demonstrates how to use Processing with the GRT GUI. 
You can find out more information about how to use this code here: 

http://www.nickgillian.com/wiki/pmwiki.php/GRT/GUIProcessing
*/

//Import the P5 OSC library





//by puching
ARDroneForP5 ardrone;
String keyValue="";
long preTime = 0;
long nowTime = 0;
//by puching



//Set the pipeline mode (CLASSIFICATION_MODE or REGRESSION_MODE), the number of inputs and the number of outputs
final int pipelineMode = GRT.CLASSIFICATION_MODE;
final int numInputs = 2;
final int numOutputs = 1;

//Create a new GRT instance, this will initalize everything for us and send the setup message to the GRT GUI
GRT grt = new GRT( pipelineMode, numInputs, numOutputs, "127.0.0.1", 5000, 5001, true );

//Create some global variables to hold our data
float[] data = new float[ numInputs ];
float[] targetVector = new float[ numOutputs ];
PFont font;

public void setup() {
  size(960,960);
  frameRate(30);
  
  //Load the font
  font = loadFont("SansSerif-48.vlw");


  //by puching
  ardrone=new ARDroneForP5("192.168.1.1", ARDroneVersion.ARDRONE2);
  // connect to the AR.Drone
  ardrone.connect();
  // for getting sensor information
  ardrone.connectNav();
  // for getting video informationp
  ardrone.connectVideo();
  // start to control AR.Drone and get sensor and video data of it
  ardrone.start();
  //by puching
}

public void draw() {
  background(204);  
  
  if( !grt.getInitialized() ){
    background(255,0,0);  
    println("WARNING: GRT Not Initalized. You need to call the setup function!");
    return;
  }
  
  //Draw the info text
  grt.drawInfoText(20,20);
  
  //Grab the mouse data and send it to the GRT backend via OSC
  data[0] = mouseX;
  data[1] = mouseY;
  grt.sendData( data );


  PImage img = ardrone.getVideoImage(false);
  if (img == null)
    return;
  image(img, 0, 0);

  // print out AR.Drone information
//  ardrone.printARDroneInfo();

  // getting sensor information of AR.Drone
  float pitch = ardrone.getPitch();
  float roll = ardrone.getRoll();
  float yaw = ardrone.getYaw();
  float altitude = ardrone.getAltitude();
  float[] velocity = ardrone.getVelocity();
  int battery = ardrone.getBatteryPercentage();

  String attitude = "pitch:" + pitch + "\nroll:" + roll + "\nyaw:" + yaw + "\naltitude:" + altitude;
  text(attitude, 20, 85);
  String vel = "vx:" + velocity[0] + "\nvy:" + velocity[1];
  text(vel, 20, 140);
  String bat = "battery:" + battery + " %";
  text(bat, 20, 170);



}

public void keyPressed() {
 if (key == CODED) {
    preTime = System.currentTimeMillis() / 1000;
    if (keyCode == UP) {
      keyValue = "UP";
      ardrone.forward(20); // go forward
    } 
    else if (keyCode == DOWN) {
      keyValue = "DOWN";
      ardrone.backward(20); // go backward
    } 
    else if (keyCode == LEFT) {
      keyValue = "LEFT";
      ardrone.goLeft(20); // go left
    } 
    else if (keyCode == RIGHT) {
      keyValue = "RIGHT";
      ardrone.goRight(20); // go right
    } 
    else if (keyCode == SHIFT) {
      ardrone.takeOff(); // take off, AR.Drone cannot move while landing
    } 
    else if (keyCode == CONTROL) {
      ardrone.landing();
      // landing
    }
  } 
else {
  if (key == 's') {
    nowTime = System.currentTimeMillis() / 1000;

    try {
      if (keyValue == "UP") {
        ardrone.backward(100);
      }
      else if (keyValue == "DOWN") {
        ardrone.forward(100);
      }
      else if (keyValue == "LEFT") {
        ardrone.goRight(100);
      }
      else if (keyValue == "RIGHT") {
        ardrone.goLeft(100);
      }
      else {
        preTime = nowTime;
      } 
      Thread.sleep((nowTime-preTime)*300);
      keyValue = "";
    } catch (Exception e) {}

    ardrone.stop(); // hovering
  }
  } 

  
  switch( key ){

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
      targetVector[0] -= 0.1f;
      grt.sendTargetVector( targetVector );
    break;
    case '}': //Increase the target vector value by 0.1 (only for REGRESSION_MODE)
      targetVector[0] += 0.1f;
      grt.sendTargetVector( targetVector );
    break;
    case '1': //Set the classifier as ANBC, enable scaling, enable null rejection, and set the null rejection coeff to 5.0
      grt.setClassifier( grt.ANBC, true, true, 5.0f );
    break;
    case '2'://Set the classifier as ADABOOST, enable scaling, disable null rejection, and set the null rejection coeff to 5.0
      grt.setClassifier( grt.ADABOOST, true, false, 5.0f );
    break;
    default:
      break;
  }
  
}
/**
GRT
Version: 1.0
Author: Nick Gillian

Info: This class acts as an interface to the GRT GUI. It sends data and commands to the GUI via Open Sound Control (OSC). To use this
code you should download and install the Processing OSC library from: http://www.sojamo.de/libraries/oscP5/
*/

class GRT{
  
  private OscP5 oscP5;
  private NetAddress grtBackend;

  private boolean initialized;
  private boolean record;
  private int pipelineMode;
  private int numDimensions;
  private int targetVectorSize;
  private int trainingClassLabel;
  private int predictedClassLabel;
  
  private int grtPipelineMode;
  private boolean grtPipelineTrained;
  private boolean grtRecording;
  private int grtNumTrainingSamples;
  private int grtNumClassesInTrainingData;
  private String grtInfoText;
   
  private float maximumLikelihood;
  private float[] preProcessedData;
  private float[] featureExtractionData;
  private float[] classLikelihoods;
  private float[] classDistances;
  private float[] regressionData;
  private float[] targetVector;
  private int[] classLabels;
  
  //Pipeline Modes
  public static final int CLASSIFICATION_MODE = 0;
  public static final int REGRESSION_MODE = 1;
  public static final int TIMESERIES_MODE = 2;
  
  //Classifier Types
  public static final int ANBC = 0;
  public static final int ADABOOST = 1;
  public static final int GMM = 2;
  public static final int KNN = 3;
  public static final int MINDIST = 4;
  public static final int SOFTMAX = 5;
  public static final int SVM = 6;
  
  /**
   Default constructor
  */
  GRT(){
    initialized = false;
  }
  
  /**
   Main constructor used to initialize the instance.
   
   @param int pipelineMode: sets the mode that the pipeline will run in. This should be a valid pipeline mode, either CLASSIFICATION_MODE, REGRESSION_MODE, or TIMESERIES_MODE
   @param int numInputs: sets the size of your data vector, this is the data that you will send to the GRT GUI
   @param int numOutputs: this parameter is only used for REGRESSION_MODE, in which case it sets the target vector size
   @param String grtIPAddress: the IP address of the machine running the GRT GUI. If it is running on the same machine as this Processing Sketch this should be "127.0.0.1"
   @param int grtPort: the network port that the GRT GUI is listening for connections on. This is set by the OSC Receive Port setting in the GRT GUI
   @param int listenerPort: the network port that this Processing Sketch should listen for OSC messages from the GRT GUI
   @param bool sendSetupMessage: if true then the Setup message will be sent to the GRT GUI, if false then the message will not be sent
  */
  GRT(int pipelineMode,int numInputs,int numOutputs,String grtIPAddress,int grtPort,int listenerPort, boolean sendSetupMessage){
    initialized = init( pipelineMode, numInputs, numOutputs, grtIPAddress, grtPort, listenerPort, sendSetupMessage );
  }
  
  /**
   This function initalizes the GRT backend. It will set up any memory required for running the GRT backend and will then send a Setup message to the GRT GUI via OSC (if the
   sendSetupMessage parameter is true).
   
   @param int pipelineMode: sets the mode that the pipeline will run in. This should be a valid pipeline mode, either CLASSIFICATION_MODE, REGRESSION_MODE, or TIMESERIES_MODE
   @param int numInputs: sets the size of your data vector, this is the data that you will send to the GRT GUI
   @param int numOutputs: this parameter is only used for REGRESSION_MODE, in which case it sets the target vector size
   @param String grtIPAddress: the IP address of the machine running the GRT GUI. If it is running on the same machine as this Processing Sketch this should be "127.0.0.1"
   @param int grtPort: the network port that the GRT GUI is listening for connections on. This is set by the OSC Receive Port setting in the GRT GUI
   @param int listenerPort: the network port that this Processing Sketch should listen for OSC messages from the GRT GUI
   @param boolean sendSetupMessage: if true then the Setup message will be sent to the GRT GUI, if false then the message will not be sent
   @return returns true if the initalization was successful, false otherwise
  */
  public boolean init(int pipelineMode,int numInputs,int numOutputs,String grtIPAddress,int grtPort,int listenerPort, boolean sendSetupMessage){
    
    initialized = false;
    
    if( pipelineMode != CLASSIFICATION_MODE && pipelineMode != REGRESSION_MODE && pipelineMode != TIMESERIES_MODE ){
      return false;
    }
    
    this.pipelineMode = pipelineMode;
    this.numDimensions = numInputs;
    
    if( pipelineMode == REGRESSION_MODE ) targetVectorSize = numOutputs;
    else targetVectorSize = 0;
    
    //Init the grt status values, this will be updated each time we get a status message from the GRT
    grtPipelineMode = 0;
    grtPipelineTrained = false;
    grtRecording = false;
    grtNumTrainingSamples = 0;
    grtNumClassesInTrainingData = 0;
    grtInfoText = "";
    
    //Init the prediction data, this will be resized when any new prediction data is received
    preProcessedData = new float[ numDimensions ];
    featureExtractionData = new float[ 1 ]; 
    classLikelihoods = new float[ 1 ];
    classDistances = new float[ 1];
    regressionData = new float[ targetVectorSize ];
    targetVector = new float[ targetVectorSize ];
    classLabels = new int[ 1 ];
    
    trainingClassLabel = 1;
    record = false;
    predictedClassLabel = 0;
    maximumLikelihood = 0;
    
    //Setup the I/O networks
    oscP5 = new OscP5(this,listenerPort);
    grtBackend = new NetAddress(grtIPAddress,grtPort);
  
    //Send the setup message
    if( sendSetupMessage ){
      OscMessage msg = new OscMessage("/Setup");
      msg.add( pipelineMode );
      msg.add( numInputs );
      msg.add( numOutputs );
      oscP5.send(msg, grtBackend); 
    }
    
    initialized = true;
    
    return true;
  }
  
  /**
   This function sends the data to the GRT GUI. The size of the data vector must match the numDimensions parameter.
   
   You need to initalize the GRT backend before you use this function.
   
   @param float[] data: the data you want to send to the GRT GUI
   @return returns true if the data was sent successful, false otherwise
  */
  public boolean sendData(float[] data){
    
    if( !initialized ) return false;
    
    if( data.length != numDimensions ) return false;
      
    OscMessage msg = new OscMessage("/Data");
      
    for(int i=0; i<numDimensions; i++){
      msg.add( data[i] );
    }
    oscP5.send(msg, grtBackend); 
      
    return true;
  }
  
  /**
   This function sends the target data to the GRT GUI. The size of the target vector must match the targetVectorSize parameter.
   
   This function is only useful if you have the GRT in REGRESSION_MODE.
   You need to initalize the GRT backend before you use this function.
   
   @param float[] targetVector: the targetVector you want to send to the GRT GUI
   @return returns true if the targetVector was sent successful, false otherwise
  */
  public boolean sendTargetVector(float[] targetVector){
    
    if( !initialized ) return false;
    
    if( targetVector.length != targetVectorSize ) return false;
    
    OscMessage msg = new OscMessage("/TargetVector");
    for(int i=0; i<targetVectorSize; i++){
      msg.add( targetVector[i] );
    }
    oscP5.send(msg, grtBackend); 
    
    //Save the target vector so we can draw it
    this.targetVector = targetVector;
    
    return true;
  }

  /**
   This function increments the current training class label and then sends the updated training class label to the GRT GUI.
   
   This function is only useful if you have the GRT in CLASSIFICATION_MODE or TIMESERIES_MODE.
   You need to initalize the GRT backend before you use this function.
   
   @return returns true if the targetVector was sent successful, false otherwise
  */
  public boolean incrementTrainingClassLabel(){
    
    if( !initialized ) return false;
    
    trainingClassLabel++;
    
    OscMessage msg = new OscMessage("/TrainingClassLabel");
    msg.add( trainingClassLabel );
    oscP5.send(msg, grtBackend); 
      
    return true;
  }
  
  /**
   This function decrements the current training class label and then sends the updated training class label to the GRT GUI.
   
   The training class label can not be less than 1.
   
   This function is only useful if you have the GRT in CLASSIFICATION_MODE or TIMESERIES_MODE.
   You need to initalize the GRT backend before you use this function.
   
   @return returns true if the targetVector was sent successful, false otherwise
  */
  public boolean decrementTrainingClassLabel(){
    
    if( !initialized ) return false;
    
    if( trainingClassLabel <= 1 ) return false;
    trainingClassLabel--;
    
    OscMessage msg = new OscMessage("/TrainingClassLabel");
    msg.add( trainingClassLabel );
    oscP5.send(msg, grtBackend); 
      
    return true;
  }
  
  /**
   This function sends a start recording message to the GRT GUI.
   
   You need to initalize the GRT backend before you use this function.
   
   @return returns true if the start recording message was sent successful, false otherwise
  */
  public boolean startRecording(){
    
    if( !initialized ) return false;
    
    record = true;
    OscMessage msg = new OscMessage("/Record");
    msg.add( 1 );
    oscP5.send(msg, grtBackend); 
      
    return true; 
  }
  
  /**
   This function sends a stop recording message to the GRT GUI.
   
   You need to initalize the GRT backend before you use this function.
   
   @return returns true if the stop recording message was sent successful, false otherwise
  */
  public boolean stopRecording(){
    
    if( !initialized ) return false;
    
    record = false;
    OscMessage msg = new OscMessage("/Record");
    msg.add( 0 );
    oscP5.send(msg, grtBackend); 
      
    return true; 
  }
  
  /**
   This function sends a train message to the GRT GUI.
   
   You need to initalize the GRT backend before you use this function.
   
   @return returns true if the train message was sent successful, false otherwise
  */
  public boolean startTraining(){
    
    if( !initialized ) return false;
    OscMessage msg = new OscMessage("/Train");
    msg.add( 1 );
    oscP5.send(msg, grtBackend); 
      
    return true;
  }
  
  /**
   This function sends a message to the GRT GUI indicating that it should save the current training dataset to a file.
   
   You need to initalize the GRT backend before you use this function.
   
   @param String filename: the name of the file you want to save the training data to
   @return returns true if the SaveTrainingDatasetToFile message was sent successful, false otherwise
  */
  public boolean saveTrainingDatasetToFile( String filename ){
    
    if( !initialized ) return false;
    
    OscMessage msg = new OscMessage("/SaveTrainingDatasetToFile");
    msg.add( filename );
    oscP5.send(msg, grtBackend);
    
    return true;
  }
  
  /**
   This function sends a message to the GRT GUI indicating that it should load a training dataset from a file.
   
   You need to initalize the GRT backend before you use this function.
   
   @param String filename: the name of the file you want to load the training data from
   @return returns true if the loadTrainingDatasetFromFile message was sent successful, false otherwise
  */
  public boolean loadTrainingDatasetFromFile( String filename ){
    
      if( !initialized ) return false;
     
      OscMessage msg = new OscMessage("/LoadTrainingDatasetFromFile");
      msg.add( filename );
      oscP5.send(msg, grtBackend);
      
      return true;
  }
  
  /**
   This function sends a message to the GRT GUI indicating that it should clear any training data.
   
   You need to initalize the GRT backend before you use this function.
   
   @return returns true if the ClearTrainingDataset message was sent successful, false otherwise
  */
  public boolean clearTrainingDataset( ){
    
      if( !initialized ) return false;
     
      OscMessage msg = new OscMessage("/ClearTrainingDataset");
      oscP5.send(msg, grtBackend);
      
      return true;
  }
  
  /**
   This function sends a message to the GRT GUI to set a specific classifier. You can also use this function to select if
   the classifier should use scaling or null rejection and to update the null rejection threshold.
   
   Note that calling this function will automatically update your classifier, so you will need to retrain any classification
   model after doing this before you can use the new classifier.
   
   You need to initalize the GRT backend before you use this function.
   
   @param int classifierType: this should be one of the classifier types (see the list of defined classifier types at the top of this class)
   @param boolean useScaling: if true, then the GRT will automatically scale your data
   @param boolean useNullRejection: if true, then the GRT will automatically try to reject new data that has a low probability
   @param double nullRejectionThreshold: this value controls the null rejection threshold. See http://www.nickgillian.com/wiki/pmwiki.php/GRT/AutomaticGestureSpotting for more information.
   @return returns true if the setClassifier message was sent successful, false otherwise
  */
  public boolean setClassifier(int classifierType,boolean useScaling,boolean useNullRejection,double nullRejectionThreshold ){
    
      if( !initialized ) return false;
      
      if( classifierType < ANBC || classifierType > ADABOOST ) return false;
      
      if( nullRejectionThreshold < 0 ) return false;
      
      OscMessage msg = new OscMessage("/SetClassifier");
      msg.add( classifierType );
      msg.add( useScaling ? 1 : 0 );
      msg.add( useNullRejection ? 1 : 0 );
      msg.add( nullRejectionThreshold );
      oscP5.send(msg, grtBackend);
      
      return true;
  }
  
  /**
   This function draws some useful info to the main processing draw window. 
   
   @param int x: the x position that the info text will be drawn at
   @param int y: the y position that the info text will be drawn at
   @return returns true if the info text was drawn successful, false otherwise
  */
  public boolean drawInfoText( int x, int y ){
    
      if( !initialized ) return false;
      
      //Draw the info text
      stroke( 255 ); 
      textFont(font, 16);
    
      //Draw the status
      int spacer = 20;
  
      //Draw the pipeline mode
      switch( getPipelineMode() ){
        case  CLASSIFICATION_MODE:
          text("Pipeline Mode: CLASSIFICATION", x, y);
        break;
        case  REGRESSION_MODE:
          text("Pipeline Mode: REGRESSION", x, y);
        break;
        case  TIMESERIES_MODE:
          text("Pipeline Mode: TIMESERIES", x, y);
        break;
      }
      y += spacer*2;
      
      
      //Draw the status info
      text("-----------GRT Status Info-----------", x, y); 
      y += spacer;
      switch( grtPipelineMode ){
        case  CLASSIFICATION_MODE:
          text("GRT Pipeline Mode: CLASSIFICATION", x, y);
        break;
        case  REGRESSION_MODE:
          text("GRT Pipeline Mode: REGRESSION", x, y);
        break;
        case  TIMESERIES_MODE:
          text("GRT Pipeline Mode: TIMESERIES", x, y);
        break;
      }
      y += spacer;
      text("Pipeline Trained: " + (grtPipelineTrained==true?"YES":"NO"), x, y);
      y += spacer;
      text("GRT Message Log: " + grtInfoText, x, y);
      y += spacer;
      
    
      y += spacer*2;
      //Draw the training info
      text("-----------Training Info-----------", x, y); 
  
      y += spacer;
      text("Recording: " + (getRecordingStatus()==true?"YES":"NO"), x, y);
  
      y += spacer;
      switch( getPipelineMode() ){
        case  CLASSIFICATION_MODE:
          text("TrainingClassLabel: " + getTrainingClassLabel(), x, y);
        break;
        case  REGRESSION_MODE:
          String targetVectorText = "TargetVector: ";
          for(int i=0; i<targetVector.length; i++)
            targetVectorText += " " + targetVector[i];
          text(targetVectorText, x, y);
        break;
        case  TIMESERIES_MODE:
        break;
      }
      
      //Draw the prediction info
      y += spacer*2;
      text("-----------Prediction Info-----------", x, y); 
      
      //Draw the prediction data
      y += spacer;
      
      float[] regressionResults = grt.getRegressionData();
      switch( getPipelineMode() ){
        case  CLASSIFICATION_MODE:
          text("PredictedClassLabel: " + getPredictedClassLabel(), x, y);
          y += spacer;
          
          text("MaximumLikelihood: " + getMaximumLikelihood(), x, y);
          y += spacer;
          
          String classLikelihoodsText = "ClassLikelihoods: ";
          for(int i=0; i<classLikelihoods.length; i++)
            classLikelihoodsText += "    " + (classLikelihoods[i]>1.0e-5f?classLikelihoods[i]:0);
          text(classLikelihoodsText, x, y);
          y += spacer;
          
          String classDistancesText = "ClassDistances: ";
          for(int i=0; i<classDistances.length; i++)
            classDistancesText += "    " + classDistances[i];
          text(classDistancesText, x, y);
          y += spacer;
          
        break;
        case  REGRESSION_MODE:
          if( regressionResults.length > 0 ){
            text("PredictedRegressionData: " + regressionResults[0], x, y);
            y += spacer;
          }
        break;
        case  TIMESERIES_MODE:
        break;
      }
     
      return true;
  }
  
  /**
   Returns if the instance has been initialized.
   
   @return returns true if the instance has been initialized, false otherwise
  */
  public boolean getInitialized(){
    return initialized;
  }
  
  /**
   Returns if a start recording message has been sent to the GRT GUI.
   
   @return returns true if a start recording message has been sent to the GRT GUI, false otherwise
  */
  public boolean getRecordingStatus(){
    return record;
  }
  
  /**
   @return returns the current pipelineMode
  */
  public int getPipelineMode(){
    return pipelineMode;
  }
  
  /**
   @return returns the current trainingClassLabel
  */
  public int getTrainingClassLabel(){
    return trainingClassLabel; 
  }
  
  /**
   @return returns the most recent predictedClassLabel, received from the GRT GUI
  */
  public int getPredictedClassLabel(){
    return predictedClassLabel;
  }
  
  /**
   @return returns the number of classes in the training data, received from the GRT GUI
  */
  public int getNumClassesInTrainingData(){
    return grtNumClassesInTrainingData; 
  }
  
  /**
   @return returns the most recent maximumLikelihood, received from the GRT GUI
  */
  public double getMaximumLikelihood(){
    return maximumLikelihood; 
  }
  
  /**
   @return returns the most recent preProcessedData, received from the GRT GUI
  */
  public float[] getPreProcessedData(){
    return preProcessedData; 
  }
  
  /**
   @return returns the most recent featureExtractionData, received from the GRT GUI
  */
  public float[] getFeatureExtractionData(){
    return featureExtractionData; 
  }
  
  /**
   @return returns the most recent classLikelihoods, received from the GRT GUI
  */
  public float[] getClassLikelihoods(){
    return classLikelihoods; 
  }
  
  /**
   @return returns the most recent classDistances, received from the GRT GUI
  */
  public float[] getClassDistances(){
    return classDistances; 
  }
  
  /**
   @return returns the most recent regressionData, received from the GRT GUI
  */
  public float[] getRegressionData(){
    return regressionData; 
  }
  
  /**
   @return returns the most recent classLabels, received from the GRT GUI
  */
  public int[] getClassLabels(){
    return classLabels; 
  }
  
  /**
   This function is called anytime a new OSC message is received. It will then try and parse the message.
  */
  public void oscEvent(OscMessage theOscMessage) {
  
    if( parseMessage( theOscMessage ) ) return;
 
  }
  
  /**
   This function parses OSC messages from the GRT GUI. 
   
   @param OscMessage theOscMessage: the osc message to parse
   @return returns true if the message was successfully parsed, false otherwise
  */
  public boolean parseMessage(OscMessage theOscMessage) {
    
    if( theOscMessage.checkAddrPattern("/Status")==true) {
      grtPipelineMode = theOscMessage.get(0).intValue();
      grtPipelineTrained = theOscMessage.get(1).intValue() == 1 ? true : false;
      grtRecording = theOscMessage.get(2).intValue() == 1 ? true : false;
      grtNumTrainingSamples = theOscMessage.get(3).intValue();
      grtNumClassesInTrainingData = theOscMessage.get(4).intValue();
      grtInfoText = theOscMessage.get(5).stringValue();
      return true;
    }
    
    if(theOscMessage.checkAddrPattern("/PreProcessedData")==true) {
      int N = theOscMessage.get(0).intValue();
      if( preProcessedData.length != N ){
        preProcessedData = new float[N]; 
      }
      for(int i=0; i<N; i++){
        preProcessedData[i] =  theOscMessage.get(i+1).floatValue();
      }
      return true;
    }
    
    if(theOscMessage.checkAddrPattern("/FeatureExtractionData")==true) {
      int N = theOscMessage.get(0).intValue();
      if( featureExtractionData.length != N ){
        featureExtractionData = new float[N]; 
      }
      for(int i=0; i<N; i++){
        featureExtractionData[i] =  theOscMessage.get(i+1).floatValue();
      }
      return true;
    }
    
    if(theOscMessage.checkAddrPattern("/Prediction")==true) {
      if(theOscMessage.checkTypetag("if")) {
        predictedClassLabel = theOscMessage.get(0).intValue();
        maximumLikelihood = theOscMessage.get(1).floatValue();
        return true;
      }   
    }
    
    if(theOscMessage.checkAddrPattern("/ClassLikelihoods")==true) {
        int N = theOscMessage.get(0).intValue();
        if( classLikelihoods.length != N ) classLikelihoods = new float[N];
        
        for(int i=0; i<N; i++){
          classLikelihoods[i] = theOscMessage.get(1+i).floatValue();
        }
        return true;
    }
    
    if(theOscMessage.checkAddrPattern("/ClassDistances")==true) {
        int N = theOscMessage.get(0).intValue();
        if( classDistances.length != N ) classDistances = new float[N];
        
        for(int i=0; i<N; i++){
          classDistances[i] = theOscMessage.get(1+i).floatValue();
        }
        return true;
    }
    
    if(theOscMessage.checkAddrPattern("/ClassLabels")==true) {
      int N = theOscMessage.get(0).intValue();
      if( classLabels.length != N ) classLabels = new int[N]; 

      for(int i=0; i<N; i++){
        classLabels[i] = theOscMessage.get(i+1).intValue();
      }
      return true;
    }
    
     if(theOscMessage.checkAddrPattern("/RegressionData")==true) {
        int N = theOscMessage.get(0).intValue();
        if( regressionData.length != N ) regressionData = new float[N];
        for(int i=0; i<N; i++){
          regressionData[i] = theOscMessage.get(1+i).floatValue();
        }
        return true;
    }
    
    return false;
 
  }

};
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "ProcessingGRTExample" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
