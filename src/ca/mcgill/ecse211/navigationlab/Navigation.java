/*
 * Navigation.java
 */
package ca.mcgill.ecse211.navigationlab;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation extends Thread {
  private static final int FORWARD_SPEED = 200;
  private static final int ROTATE_SPEED = 100;
  private static double[][] waypoints = new double[][] {
	  										{0, 1, 2, 2, 1 },  // Row 0 is x coordinates
	  										{2, 1, 2, 1, 0 } };// Row 1 is y coordinates
  public EV3LargeRegulatedMotor leftMotor;
  public EV3LargeRegulatedMotor rightMotor;
  private double radius;
  private double width;
  private Odometer odometer;
  private static final long CORRECTION_PERIOD = 10;
  public boolean navigating; 
  private BangBangController bangbang;
  private UltrasonicPoller usPoller;
  private int whichPoint = 0;

  public Navigation (EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
	      double leftRadius, double rightRadius, double width, Odometer odometer, BangBangController bangbang) {
	  this.leftMotor = leftMotor;
	  this.rightMotor = rightMotor;
	  this.radius = rightRadius;
	  this.width = width;
	  this.odometer = odometer;
	  this.bangbang = bangbang;
  }
  public void run() {
	long correctionStart, correctionEnd;
    // reset the motors
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
      motor.stop();
      motor.setAcceleration(3000);
    }
    while (true) {
        correctionStart = System.currentTimeMillis();
    
    if(NavigationLab.demo) {
    		for(int i = 0; waypoints[0].length > i ; i++){
    			navigating = true;
    			travelTo(waypoints[0][i],waypoints[1][i]);
    			while(leftMotor.isMoving()&&rightMotor.isMoving()) {
    			}
    		}
    }
    else {
    		navigating = true;
    		for(int i = 0; waypoints[0].length > i ; i++){
    				i= whichPoint;
    				travelTo(waypoints[0][i],waypoints[1][i]);
    				while(leftMotor.isMoving()&&rightMotor.isMoving()) {
    					 if(bangbang.readUSDistance() <= 9) {
    				    	  	leftMotor.stop();
    				    	 	rightMotor.stop();
    				    	  	rightMotor.setAcceleration(3000);
    				    	  	leftMotor.setAcceleration(3000);
    				    	  	leftMotor.rotate(90);
    				    	  	rightMotor.rotate(-90);
    				    	  	NavigationLab.sensorMotor.rotate(-90);
    				    	  	navigating = false;
    				    	  	bangbang.activate();
    					 }
    					 
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

  void travelTo(double x, double y) {

	  navigating =true;
	  double deltaY = (y*30.48) - odometer.getY();
	  double deltaX = (x*30.48) - odometer.getX();
	  
		  double thetaD = Math.atan2(deltaX,deltaY);
		  double thetaTurn = thetaD - odometer.getTheta();
		  if(thetaTurn < -Math.PI) {
			  turnTo(2*Math.PI + thetaTurn);
			  
		  }
		  else if(thetaTurn > Math.PI) {
			  turnTo(thetaTurn - 2*Math.PI); 
		  }
		  else {
			  turnTo(thetaTurn);
		  }
	  
	  leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      double distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
      leftMotor.rotate(convertDistance(radius, distance), true);
      rightMotor.rotate(convertDistance(radius, distance), true);
      whichPoint++;
      navigating = false;
	
  }
  
  void turnTo(double theta) {
	  navigating = true;
	  leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED);
      leftMotor.rotate(convertAngle(radius, width, Math.toDegrees(theta)), true);
    	  rightMotor.rotate(-convertAngle(radius, width,  Math.toDegrees(theta)), false);  
  }
  
  boolean isNavigating() {
	  return this.navigating;
  }
  
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
}
