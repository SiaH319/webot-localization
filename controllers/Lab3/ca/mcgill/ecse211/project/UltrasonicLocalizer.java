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
  // parameters for localiz()
  private static double backAngle;
  private static double leftAngle;
  private static double deltaT;

  /**
   * Localize the position of the robot using ultrasonic sensor localizer.
   ** @author Sia Ham
   */

  public static void localize() {
    System.out.println("UltrasonicLocalizer is localizing the robot");

    // turns robot to face inside board
    while (readUsDistance() < 100) {
      clockwise();
    }
    System.out.println("Robot facing inside board");

    // start finding back angle by rotating clockwise
    while (readUsDistance() > d) {
      clockwise();
    }
    backAngle = odometer.getXyt()[2];
    System.out.println("backAngle = " + backAngle);

    //start rotating the opposite direction
    //this prevents the sensor from immediately 
    //detecting back angle again and thinking its the left angle
    counterclockwise();
    for (int i = 0; i < 50; i++) {
      waitUntilNextStep();
    }

    // rotate counter clockwise
    while (true) {
      counterclockwise();
      if (readUsDistance() < d && Math.abs(backAngle - odometer.getXyt()[2]) > 35) { 
        // make sure backAngle and leftAngle are found on the back wall and left wall
        break;
      }
    }
    leftAngle = odometer.getXyt()[2];
    System.out.println("leftAngle = " + leftAngle);

    // find deltaT to turn the robot at 0 dgree
    double average = (leftAngle + backAngle) / 2;
    if (backAngle > leftAngle) {
      deltaT = average - odometer.getXyt()[2] - 45;// modified formula from Tian Han Jiang
    }

    else { 
      deltaT = average - odometer.getXyt()[2] + 135; // formula from Tian Han Jiang
    }

    turnBy(deltaT);
  }
  
  /**
   * rotate in clockwise
   * */
  private static void clockwise() {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    leftMotor.forward();
    rightMotor.backward();
  }

  /**
   * rotate in counter-clockwise
   * */
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

  /**
   * Turns the robot by a specified angle. Note that this method is different from
   * {@code Navigation.turnTo()}. For example, if the robot is facing 90 degrees, calling
   * {@code turnBy(90)} will make the robot turn to 180 degrees, but calling
   * {@code Navigation.turnTo(90)} should do nothing (since the robot is already at 90 degrees).
   * 
   * @param angle the angle by which to turn, in degrees
   */
  public static void turnBy(double angle) {
    leftMotor.rotate(convertAngle(angle), true);
    rightMotor.rotate(-convertAngle(angle), false);
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that
   * angle.
   * @param angle the input angle in degrees
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    return convertDistance((BASE_WIDTH * Math.PI * angle) / 360);
  }
  
  /**
   * Stops both motors.
   */
  public static void stopMotors() {
    leftMotor.stop();
    rightMotor.stop();
  }
  
  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that
   * angle.
   * * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertDistance(double distance) {
    return (int) ((distance * 180) / (Math.PI * WHEEL_RAD));
  }
}