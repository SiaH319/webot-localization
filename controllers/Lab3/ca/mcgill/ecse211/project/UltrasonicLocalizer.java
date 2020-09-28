package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;

import simlejos.robotics.SampleProvider;


/** 
 * class for the handling the ultrasonic localization. * 
 * @author Sia Ham
 *
 */
public class UltrasonicLocalizer {

  private static int dropoff = 30;
  private static int dist;
  private static double[] fallingAngle = new double[2];

  /** Buffer (array) to store US samples. */
  private static SampleProvider usDistance = usSensor.getMode("Distance");
  private static float[] usData = new float[usDistance.sampleSize()];

  // parameters for filter() methods.
  /** The distance remembered by the filter() method. */
  private static int prevDistance;
  /** The number of invalid samples seen by filter() so far. */
  private static int invalidSampleCount;

  public enum Direction { LEFT, BACK };


  /**
   * Consturctor for UltrasonicLocalizer().
   */

  public static void localize() {
    double deltaT;
    double angleA = 0; 
    double angleB= 0; 
    double[] Angles = new double[2];
    
    while (angleB != 0) {
      if (getDist()>dropoff) { // facing away from the wall
        while (getDist()<dropoff) {
          rotateClockwise();
        }
        angleA = odometer.getXyt()[2];
      }

      else { // facing the wall
        while (getDist()>dropoff) {
          rotateCounterClockwise();
        }
        angleB = odometer.getXyt()[2];
      }
    }



    if (angleA > angleB) {
      deltaT = 225.0 - 0.5 * (angleA - angleB);
    }

    else  {
      deltaT = 45.0 - 0.5 * (angleA - angleB);
    }

    double newAngle = deltaT + odometer.getXyt()[2];

    odometer.setXyt(0.0, 0.0, deltaT);
    turnBy(360 - (newAngle));

  }

  private static int getDist() {
    int distance;
    usDistance.fetchSample(usData, 0);
    distance = (int) (usData[0] * 100.0);
    return filter(distance); 
  }

  /**
   * Rotate the robot clockwise.
   */
  public static void rotateClockwise() {
    leftMotor.forward();
    rightMotor.backward();
  }

  /**
   * Rotate the robot counter clockwise.
   */ 
  public static void rotateCounterClockwise() {
    leftMotor.backward();
    rightMotor.forward();
  }

  /**
   * Stop the movement of the robot.
   */ 
  public static void stopMoving() {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
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
    } else {
      if (distance < MAX_SENSOR_DIST) {
        invalidSampleCount = 0; // reset filter and remember the input distance.
      }
      prevDistance = distance;
      return distance;
    }
  }
  public static void turnBy(double angle) {
    // TODO Hint: similar to moveStraightFor(), but use a minus sign

    leftMotor.rotate(convertAngle(angle), true);
    rightMotor.rotate(-convertAngle(angle), false);

  }
  public static int convertAngle(double angle) {

    // TODO Compute and return the correct value. Hint: you can reuse convertDistance()
    return convertDistance((BASE_WIDTH * Math.PI * angle) / 360);
  }
  public static int convertDistance(double distance) {
    // TODO Compute and return the correct value
    return (int) ((distance * 180) / (Math.PI * WHEEL_RAD));
  }

}