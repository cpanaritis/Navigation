package ca.mcgill.ecse211.navigationlab;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {
	
  /* Constants */
  private static final int FILTER_OUT = 20;

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;
  private int filterControl;
  private boolean active = false; 
 

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    NavigationLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    NavigationLab.rightMotor.setSpeed(motorHigh);
  }

  @Override
  public void processUSData(int distance) {
	if(active) {
	  if (distance >= 255 && filterControl < FILTER_OUT) {
	      // bad value, do not set the distance var, however do increment the
	      // filter value
	      filterControl++;
	    } else if (distance >= 255) {
	      // We have repeated large values, so there must actually be nothing
	      // there: leave the distance alone
	      this.distance = distance;
	    } else {
	      // distance went below 255: reset filter and leave
	      // distance alone.
	      filterControl = 0;
	      this.distance = distance;
	    }
    int error = this.distance - bandCenter;  // Computer value of error.
    if(Math.abs(error) <= bandwidth){       // Error within limits, keep going straight.
    		NavigationLab.leftMotor.setSpeed(motorHigh);
    		NavigationLab.rightMotor.setSpeed(motorHigh);
    		NavigationLab.leftMotor.forward();
    		NavigationLab.rightMotor.forward();
    }
    else if(error < 0){ // Negative error means too close to wall.
    	if(error < -3){ // Emergency turn used for convex angles.
    		NavigationLab.leftMotor.setSpeed(motorLow);
    		NavigationLab.rightMotor.setSpeed(motorLow);
    		NavigationLab.leftMotor.forward();
    		NavigationLab.rightMotor.backward();
    	}
    	else{
    		NavigationLab.leftMotor.setSpeed(motorHigh);
    		NavigationLab.rightMotor.setSpeed(motorLow);
    		NavigationLab.leftMotor.forward();
    		NavigationLab.rightMotor.forward();
    	}
    }
    else if(error > 0) { // Positive error means too far from wall.
    		NavigationLab.leftMotor.setSpeed(motorLow);
    		NavigationLab.rightMotor.setSpeed(motorHigh);
    		NavigationLab.leftMotor.forward();
    		NavigationLab.rightMotor.forward();
    }
	  }
    
  }

  public void activate() {
	  active = true;
  }
  public void deactivate() {
	  active = false;
  }
  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
