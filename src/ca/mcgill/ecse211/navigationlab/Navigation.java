/*
 * Navigation.java
 */
package ca.mcgill.ecse211.navigationlab;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * @author Christos Panaritis Kevin Chuong
 * Navigation class
 *
 */
public class Navigation extends Thread implements UltrasonicController {
	
  private static final int FORWARD_SPEED = 200;
  private static final int ROTATE_SPEED = 100;
  private static double[][] waypoints = new double[][] {
	  										{2, 1, 1, 2 },  // Row 0 is x coordinates
	  										{1, 1, 2, 0 } };// Row 1 is y coordinates
  public EV3LargeRegulatedMotor leftMotor;
  public EV3LargeRegulatedMotor rightMotor;
  private double radius;
  private double width;
  private Odometer odometer;
  public boolean navigating; 
  private int whichPoint = 0;
  private double lastTheta;
  private int distanceFromBlock;
  public boolean active = false;  // Checks if avoidance should run or not (bang bang)
  public static final int bandCenter = 8; // Offset from the wall (cm)
  private static final int bandWidth = 2; // Width of dead band (cm)

  private int filterControl;
  private static final int FILTER_OUT = 20;

  

  /**
   * constructor
 * @param leftMotor
 * @param rightMotor
 * @param leftRadius
 * @param rightRadius
 * @param width
 * @param odometer
 */
public Navigation (EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
	      double leftRadius, double rightRadius, double width, Odometer odometer) {
	  this.leftMotor = leftMotor;
	  this.rightMotor = rightMotor;
	  this.radius = rightRadius;
	  this.width = width;
	  this.odometer = odometer;
  }
  
  /* (non-Javadoc)
 * @see java.lang.Thread#run()
 * runs the thread depending on what mode you are on.
 */
public void run() {
    // reset the motors
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
      motor.stop();
      motor.setAcceleration(3000);
    }
    
    if(NavigationLab.demo) {
    		for(int i = 0; waypoints[0].length > i ; i++){
    			navigating = true;
    			travelTo(waypoints[0][i],waypoints[1][i]);
    			while(leftMotor.isMoving()&&rightMotor.isMoving()) {
    			}
    		}
    }
    else {
    		for(int i = 0; waypoints[0].length > i ; i++){
    				i=whichPoint;
    				navigating = true;
    				travelTo(waypoints[0][i],waypoints[1][i]);
    		}
    }
   //check if this is needed
  } 

  /**
 * @param x
 * @param y
 * The first part travels the robot to the point
 * The second part is used to avoid the obstacle
 */
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
      boolean thing = true;
      
      if(!(NavigationLab.demo)) {
      	while(leftMotor.isMoving()&&rightMotor.isMoving() || active) {
      		
      		if(active) {
      			
      		double error = this.distanceFromBlock - bandCenter;  // Computer value of error.
      		if(lastTheta - odometer.getTheta() >= Math.PI/2) {
				leftMotor.stop();
				rightMotor.stop();
				deactivate();
				NavigationLab.sensorMotor.rotate(90);
				whichPoint--;
			}
      		else {
      			if(Math.abs(error) <= bandWidth){       // Error within limits, keep going straight.
      	    			leftMotor.setSpeed(FORWARD_SPEED);
      	    			rightMotor.setSpeed(FORWARD_SPEED);
      	    			leftMotor.forward();
      	    			rightMotor.forward();
      			}
      			else if(error < 0){ // Negative error means too close to wall.
      				if(error < -3){ // Emergency turn used for convex angles.
      		    			leftMotor.setSpeed(ROTATE_SPEED);
      		    			rightMotor.setSpeed(ROTATE_SPEED);
      		    			leftMotor.forward();
      		    			rightMotor.backward();
      				}
      				else{
      			    		leftMotor.setSpeed(FORWARD_SPEED);
      			    		rightMotor.setSpeed(ROTATE_SPEED);
      			    		leftMotor.forward();
      			   		rightMotor.forward();
    			    		}
      			}
      			else if(error > 0) { // Positive error means too far from wall.
      				leftMotor.setSpeed(ROTATE_SPEED);
      				rightMotor.setSpeed(FORWARD_SPEED);
      				leftMotor.forward();
      				rightMotor.forward();
      			}
      		}
      	}
      	System.out.println(readUSDistance());	
		if(distanceFromBlock < bandCenter && !(active)) {
			lastTheta = odometer.getTheta();
		    turnTo(Math.PI/2);
	  	  	NavigationLab.sensorMotor.setAcceleration(1000);
	    	  	NavigationLab.sensorMotor.rotate(-90);
	    	  	activate();
		}
      }
    }
  }
  
  /**
 * @param theta
 * finds what the robot should rotate at.
 */
void turnTo(double theta) {
	  navigating = true;
	  leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED);
      leftMotor.rotate(convertAngle(radius, width, Math.toDegrees(theta)), true);
    	  rightMotor.rotate(-convertAngle(radius, width,  Math.toDegrees(theta)), true);  
  }
  
  /**
 * @return the navigating boolean 
 */
boolean isNavigating() {
	  return this.navigating;
  }
  
  /**
 * @return the boolean
 */
public boolean getStatus() {
	  return this.active;
  }
  /**
 * sets the boolean to true
 */
public void activate() {
	  this.active = true;
  }
 
/**
 * sets the boolean to false
 */
public void deactivate() {
	  this.active = false;
  }
  
  /**
 * @param radius
 * @param distance
 * @return
 */
private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
 * @param radius
 * @param width
 * @param angle
 * @return
 */
private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
/* (non-Javadoc)
 * @see ca.mcgill.ecse211.navigationlab.UltrasonicController#processUSData(int)
 */
@Override
public void processUSData(int distanceFromBlock) {
	if ((distanceFromBlock >= 255 || distanceFromBlock == 0) && filterControl < FILTER_OUT) {
	      // bad value, do not set the distance var, however do increment the
	      // filter value
	      filterControl++;
	    } 
		else if (distanceFromBlock >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distanceFromBlock = distanceFromBlock;
		} 
		else {
	      // distance went below 255: reset filter and leave
	      // distance alone.
	      filterControl = 0;
	      this.distanceFromBlock = distanceFromBlock;
	    }
}

/* (non-Javadoc)
 * @see ca.mcgill.ecse211.navigationlab.UltrasonicController#readUSDistance()
 */
@Override
public int readUSDistance() {
	return this.distanceFromBlock;
}
}
