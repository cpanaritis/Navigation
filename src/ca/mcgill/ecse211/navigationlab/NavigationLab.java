// Lab3.java

package ca.mcgill.ecse211.navigationlab;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class NavigationLab {

  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  public static final EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final Port usPort = LocalEV3.get().getPort("S4");
  public static final double WHEEL_RADIUS = 2.2;
  public static final double TRACK = 12.22;
  public static final double GRID_LENGTH = 30.48;
  public static boolean demo;
  private static final int bandCenter = 10; // Offset from the wall (cm)
  private static final int bandWidth = 2; // Width of dead band (cm)
  private static final int motorLow = 100; // Speed of slower rotating wheel (deg/sec)
  private static final int motorHigh = 200; // Speed of the faster rotating wheel (deg/seec)

  public static void main(String[] args) {
    int buttonChoice;

    final TextLCD t = LocalEV3.get().getTextLCD();
    Odometer odometer = new Odometer(leftMotor, rightMotor);
    OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);
    BangBangController bangbang = new BangBangController(bandCenter, bandWidth, motorLow, motorHigh, leftMotor, rightMotor);
    Navigation navigation = new Navigation(leftMotor, rightMotor, WHEEL_RADIUS, WHEEL_RADIUS, TRACK, odometer, bangbang);

    SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
    SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from this instance
    float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are returned

 // Setup Ultrasonic Poller // This thread samples the US and invokes
    UltrasonicPoller usPoller = null; // the selected controller on each cycle
    usPoller = new UltrasonicPoller(usDistance, usData, navigation, bangbang);
    
    do {
      // clear the display
      t.clear();

      // ask the user whether the motors should drive in a square or float
      t.drawString("< Left | Right >", 0, 0);
      t.drawString("       |        ", 0, 1);
      t.drawString(" Object| Start  ", 0, 2);
      t.drawString(" avoid | the   ", 0, 3);
      t.drawString(" path  | path ", 0, 4);

      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    if (buttonChoice == Button.ID_LEFT) {
    	  demo = false;
    	  usPoller.start();
      odometer.start();
      odometryDisplay.start();
      navigation.start();
    } 
    else {
    	  demo = true;
      odometer.start();
      odometryDisplay.start();
      navigation.start();
    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
