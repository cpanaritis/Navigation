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
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private double radius;
  private double width;
  private Odometer odometer;

  public Navigation (EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
	      double leftRadius, double rightRadius, double width, Odometer odometer) {
	  this.leftMotor = leftMotor;
	  this.rightMotor = rightMotor;
	  this.radius = rightRadius;
	  this.width = width;
	  this.odometer = odometer;
  }
  public void drive() {
    // reset the motors
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
      motor.stop();
      motor.setAcceleration(3000);
    }
    
    for(int i = 0; waypoints[0].length > i ; i++){
    	travelTo(waypoints[0][i],waypoints[1][i]);
    }
    // wait 5 seconds
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      // there is nothing to be done here because it is not expected that
      // the odometer will be interrupted by another thread
    }

      /*for (int i = 0; i < 4; i++) {
      // drive forward three tiles
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);

      leftMotor.rotate(convertDistance(leftRadius, 91.44), true);
      rightMotor.rotate(convertDistance(rightRadius, 91.44), false);

      // turn 90 degrees clockwise
      leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED);

      leftMotor.rotate(convertAngle(leftRadius, width, 90.0), true);
      rightMotor.rotate(-convertAngle(rightRadius, width, 90.0), false);
    } */
  } 

  void travelTo(double x, double y) {
	  double deltaY = y - odometer.getY();
	  double deltaX = x - odometer.getX();
	  double thetaD = Math.atan(deltaY/deltaX);
	  turnTo(thetaD - odometer.getTheta());
	  leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      double distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
      leftMotor.rotate(convertDistance(radius, distance*30.48), true);
      rightMotor.rotate(convertDistance(radius, distance*30.48), false);

	  
  }
  
  void turnTo(double theta) {
	  leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED);
      
      leftMotor.rotate(convertAngle(radius, width, Math.toDegrees(theta)), true);
      rightMotor.rotate(-convertAngle(radius, width,  Math.toDegrees(theta)), false);
	  
  }
  
  boolean isNavigating() {
	  return true;
  }
  
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
}
