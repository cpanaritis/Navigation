package ca.mcgill.ecse211.navigationlab;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * @author Christos Panaritis Kevin Chuong
 *
 */
public class Odometer extends Thread {
  // robot position
  private double x;
  private double y;
  private double theta;
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private static final long ODOMETER_PERIOD = 25; /*odometer update period, in ms*/

  private Object lock; /*lock object for mutual exclusion*/

  /**
 * @param leftMotor
 * @param rightMotor
 * Constructor
 */
public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.x = 0.0;
    this.y = 0.0;
    this.theta = 0.0;
    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;
    lock = new Object();
  }

  // run method (required for Thread)
  /* (non-Javadoc)
 * @see java.lang.Thread#run()
 * run method (required for Thread)
 */
public void run() {
    long updateStart, updateEnd;
    
    while (true) {
      updateStart = System.currentTimeMillis();
      // TODO put (some of) your odometer code here
      double distLeft, distRight, deltaDistance, deltaTheta, dX, dY; /*Respectively, the distance the LWheel traveled,
      															   					 the distance the RWheel traveled,
      															   					 the change in position of the vehicle,
      															   					 the change in heading,
      															   					 the change in X position,
      															   					 the change in Y position,
       																				 */
      distLeft = Math.PI*NavigationLab.WHEEL_RADIUS*(leftMotor.getTachoCount() - leftMotorTachoCount)/180; // Calculated using arclength
      distRight = Math.PI*NavigationLab.WHEEL_RADIUS*(rightMotor.getTachoCount() - rightMotorTachoCount)/180;
      leftMotorTachoCount = leftMotor.getTachoCount(); // Save current TachoCount (change in angle) for next iteration
      rightMotorTachoCount = rightMotor.getTachoCount();
      deltaDistance = (distLeft+distRight)/2;	//Compute displacement of vehicle
      deltaTheta = (distLeft-distRight)/NavigationLab.TRACK;	//Compute heading
      
      
      
      synchronized (lock) {
        /**
         * Don't use the variables x, y, or theta anywhere but here! Only update the values of x, y,
         * and theta in this block. Do not perform complex math
         * 
         */
    	theta += deltaTheta; //Update the heading
    	dX = deltaDistance*Math.sin(theta);	// Compute change in X using current heading
    	dY = deltaDistance*Math.cos(theta); // Compute change in Y using current heading
    	
    	x += dX;	//Updated x pos
    	y += dY;	//Updated y pos
      }

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here because it is not
          // expected that the odometer will be interrupted by
          // another thread
        }
      }
    }
  }

  /**
 * @param position
 * @param update
 */
public void getPosition(double[] position, boolean[] update) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        position[0] = x;
      if (update[1])
        position[1] = y;
      if (update[2])
        position[2] = theta;
    }
  }

  /**
 * @return x
 */
public double getX() {
    double result;

    synchronized (lock) {
      result = x;
    }

    return result;
  }

  /**
 * @return y
 */
public double getY() {
    double result;

    synchronized (lock) {
      result = y;
    }

    return result;
  }

  /**
 * @return theta
 */
public double getTheta() {
    double result;

    synchronized (lock) {
      result = theta;
    }

    return result;
  }

  /**
 * @param position
 * @param update
 * mutator
 */
public void setPosition(double[] position, boolean[] update) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        x = position[0];
      if (update[1])
        y = position[1];
      if (update[2])
        theta = position[2];
    }
  }

  public void setX(double x) {
    synchronized (lock) {
      this.x = x;
    }
  }

  public void setY(double y) {
    synchronized (lock) {
      this.y = y;
    }
  }

  public void setTheta(double theta) {
    synchronized (lock) {
      this.theta = theta;
    }
  }

  /**
   * @return the leftMotorTachoCount
   */
  public int getLeftMotorTachoCount() {
    return leftMotorTachoCount;
  }

  /**
   * @param leftMotorTachoCount the leftMotorTachoCount to set
   */
  public void setLeftMotorTachoCount(int leftMotorTachoCount) {
    synchronized (lock) {
      this.leftMotorTachoCount = leftMotorTachoCount;
    }
  }

  /**
   * @return the rightMotorTachoCount
   */
  public int getRightMotorTachoCount() {
    return rightMotorTachoCount;
  }

  /**
   * @param rightMotorTachoCount the rightMotorTachoCount to set
   */
  public void setRightMotorTachoCount(int rightMotorTachoCount) {
    synchronized (lock) {
      this.rightMotorTachoCount = rightMotorTachoCount;
    }
  }
}
