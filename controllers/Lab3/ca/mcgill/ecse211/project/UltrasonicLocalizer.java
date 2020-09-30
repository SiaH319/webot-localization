package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.waitUntilNextStep;
import simlejos.robotics.SampleProvider;


/** 
 * class for the handling the ultrasonic localization. * 
 * @author Sia Ham
 *
 */
public class UltrasonicLocalizer {

  private static int d = 30;

  /** Buffer (array) to store US samples. */
  private static float[] usData = new float[usSensor.sampleSize()];

  // parameters for filter() methods.
  /** The distance remembered by the filter() method. */
  private static int prevDistance;
  /** The number of invalid samples seen by filter() so far. */
  private static int invalidSampleCount;



  private static double backAngle;
  private static double leftAngle;
  private static double deltaT;


  public static void localize() {
    System.out.println("UltrasonicLocalizer is localizing the robot");

    // turns robot to face inside board
    while (readUsDistance() < 100) {
      clockwise();
    }
    System.out.println("Robot facing inside board");

    // start finding back angle
    while (readUsDistance() > d) {
      clockwise();
    }
    backAngle = odometer.getXyt()[2];
    System.out.println("backAngle = " + backAngle);

    //start rotating the opposite direction
    //this prevents the sensor from immediately 
    //detecting back angle again and thinking its the left angle
    counterclockwise();
    for(int i = 0; i < 50; i++) {
      waitUntilNextStep();
    }

    //you can do the same thing again for left angle


    while (true) {
      counterclockwise();

      if (readUsDistance() < d && Math.abs(backAngle - odometer.getXyt()[2]) > 30) {
        break;
      }
    }
    leftAngle = odometer.getXyt()[2];
    System.out.println("leftAngle = " + leftAngle);


    double average = (leftAngle + backAngle) / 2;
    deltaT = Math.abs((backAngle - average) / 2);
    turnBy(deltaT);

  }

  private static void clockwise() {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    leftMotor.forward();
    rightMotor.backward();
  }

  private static void counterclockwise() {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    leftMotor.backward();
    rightMotor.forward();
  }


  /** Returns the filtered distance between the US sensor and an obstacle in cm. */
  public static int readUsDistance() {
    usSensor.fetchSample(usData, 0);
    return filter((int) (usData[0] * 100.0));
  }

  /**
   * Rudimentary filter from Lab1.java - toss out invalid samples corresponding to null signal
   * @param distance raw distance measured by the sensor in cm
   * @return the filtered distance in cm
   */
  static int filter(int distance) {
    if (distance >= MAX_SENSOR_DIST && invalidSampleCount < INVALID_SAMPLE_LIMIT) {
      // bad value, increment the filter value and return the distance remembered from before
      invalidSampleCount++;
      return prevDistance;
    } 


    else {
      if (distance < MAX_SENSOR_DIST) {
        invalidSampleCount = 0; // reset filter and remember the input distance.
      }
      prevDistance = distance;
      return distance;
    }
  }

  public static void turnBy(double angle) {
    leftMotor.rotate(convertAngle(angle), true);
    rightMotor.rotate(-convertAngle(angle), false);
  }

  public static int convertAngle(double angle) {
    return convertDistance((BASE_WIDTH * Math.PI * angle) / 360);
  }

  public static void stopMotors() {
    leftMotor.stop();
    rightMotor.stop();
  }
  public static int convertDistance(double distance) {
    return (int) ((distance * 180) / (Math.PI * WHEEL_RAD));
  }

}