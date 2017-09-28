package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;
  private static final int MAXCORRECTION = 50;
  private static final double proportionConstant = 1.5;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initialize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
	// rudimentary filter - toss out invalid samples corresponding to null
	    // signal.
	    // (n.b. this was not included in the Bang-bang controller, but easily
	    // could have).
	    //
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
	int deltaChange;
    int error = this.distance-bandCenter;
    //Maintain speed if the error is within the allowed error.
    if(Math.abs(error) <= bandWidth) {
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
    	}
    //Do the opposite if the robot is too close to the wall.
    else if(error < 0) {
    	if(error < -3){ // Emergency turn used for convex angles.
        	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
        	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    		WallFollowingLab.leftMotor.forward();
    		WallFollowingLab.rightMotor.backward();
    		}
    	else{
    		deltaChange = calcCorrection(error);
    		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + deltaChange);
    		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - deltaChange);
    		WallFollowingLab.leftMotor.forward();
    		WallFollowingLab.rightMotor.forward();
    		}
    	}
    //Increase the outside wheel speed and decrease the inside wheel speed if the robot is too far from the wall.
    else if (error > 0) {   
    	deltaChange = calcCorrection (error);
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - MAXCORRECTION); //Using the deltaChnage on the left wheel would consistently make it crash so we set it to a fixed number.
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + deltaChange);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
    	}
  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }
  //Method used to calculate the proportional change in speed of the motor.
  int calcCorrection (int diff) {
	  int correction; 
	  if (diff<0) {
		  diff= Math.abs(diff);
	  }
	  correction = (int)(proportionConstant*(double)(diff));
	  if(correction >= MOTOR_SPEED) {
		  correction = MAXCORRECTION;
	  }
	  return correction; 
  }
}
