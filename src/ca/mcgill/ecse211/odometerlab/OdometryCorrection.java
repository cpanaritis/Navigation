/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometerlab;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.*;

public class OdometryCorrection extends Thread {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private SampleProvider colorSample;
  private float scaledColor;
  private float[] LData;
  private boolean firstX = true;
  private boolean firstY = true;
  private boolean firstNegativeX = true;
  private boolean firstNegativeY = true;
  private double lastX;
  private double lastY;




  // constructor
  public OdometryCorrection(Odometer odometer, SampleProvider colorSample) {
    this.odometer = odometer;
    this.colorSample = colorSample;
  }

  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    LData = new float[colorSample.sampleSize()];
    while (true) {
      correctionStart = System.currentTimeMillis();
      //TODO Place correction implementation here
      colorSample.fetchSample(LData, 0); // Get data from color sensor
      scaledColor = LData[0]*1000;
      
    	if (scaledColor <= 300) {	//makes a sound when its not a regular tile (very primitive only used
  	  	Sound.beep();		//for testing. If there's not enough data read for the line it will not beep.
  	  	if(odometer.getTheta() <= Math.PI/4){ //Update values for first side of square
  	  		if(firstY){	             		// if first line of y then y = 0
  	  			odometer.setY(0);
  	  			firstY = false;
  	  			lastY=0;
  	  		}
  	  		else{	//otherwise add a grid length to the previous y value 
  	  			lastY += OdometryLab.GRID_LENGTH;
	  			odometer.setY(lastY); // update y value
  	  		}
  	  	}
  	  	else if(odometer.getTheta() <= 3*Math.PI/4){
  	  		if(firstX){			// update values for 2nd side of square
  	  			odometer.setX(0);		// if first line of x then x = 0
  	  			firstX = false; 
  	  			lastX = 0;
  	  		}
  	  		else{
  	  			lastX += OdometryLab.GRID_LENGTH;	//otherwise add the grid length to the previous x value
  	  			odometer.setX(lastX); // update x value
  	  		}
  	  	}
  	  	else if(odometer.getTheta() <= 5*Math.PI/4){ //3rd side of square
  	  		if(firstNegativeY) {		//if it is the first y then reset the y value to the last y from the other side
  	  			odometer.setY(lastY);	
  	  			firstNegativeY = false;	
  	  		}
  	  		else {		//otherwise remove a grid length from the y value.
  	  		lastY -= OdometryLab.GRID_LENGTH;
			odometer.setY(lastY);
  	  		}
  	  	}
  	  	else{		//4th side of the square
  	  		if(firstNegativeX) {		//if it is the first x then reset the x to the last x value assigned from side 2
  	  			odometer.setX(lastX);
  	  			firstNegativeX = false;
  	  		}
  	  		else {		//otherwise remove a grid length from the x value
  	  		lastX -= OdometryLab.GRID_LENGTH;
  	  		odometer.setX(lastX);
  	  		}
  	  	}
  	  	
      } 
      
      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here because it is not
          // expected that the odometry correction will be
          // interrupted by another thread
        }
      }
    }
  }
}
