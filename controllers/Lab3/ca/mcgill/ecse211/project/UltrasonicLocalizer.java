package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;

import simlejos.robotics.SampleProvider;


/** 
 * class for the handling the ultrasonic localization. * 
 * @author Sia Ham
 *
 */
public class UltrasonicLocalizer {

  private static int d = 30;
  private static int dist;
  private static int k = 1;
  private static double[] fallingAngle = new double[2];

  /** Buffer (array) to store US samples. */
  private static float[] usData = new float[usSensor.sampleSize()];

  // parameters for filter() methods.
  /** The distance remembered by the filter() method. */
  private static int prevDistance;
  /** The number of invalid samples seen by filter() so far. */
  private static int invalidSampleCount;

  /**
   * Consturctor for UltrasonicLocalizer().
   */

  private static double backAngle;
  private static double leftAngle;
  private static double deltaT;
  private static int intPoistion = filter((int) (usData[usSensor.sampleSize()-1] * 100.0));
  public static void localize() {
    System.out.println("UltrasonicLocalizer is localizing the robot");

    while (readUsDistance() < 100) {
      leftMotor.forward();
      rightMotor.backward();
    }

    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    
    while (readUsDistance() > d) {
      leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED);
      
      leftMotor.forward();
      rightMotor.backward();
    }
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    backAngle = odometer.getXyt()[2];
    System.out.println("backAngle = "+ backAngle);
    while (true) {
      leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED);
      leftMotor.backward();
      rightMotor.forward();
      if (readUsDistance() <= d && Math.abs(((int)backAngle) - ((int)odometer.getXyt()[2]))>2) {
        break;
      }
    }
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    leftAngle = odometer.getXyt()[2];
    System.out.println("leftAngle = "+ leftAngle);


   double average = (leftAngle + backAngle)/2;
  deltaT = Math.abs((backAngle - average)/2);
    odometer.setXyt(0.0, 0.0, deltaT);
    turnBy(deltaT);

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