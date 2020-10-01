package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.sleepFor;
import static simlejos.ExecutionController.waitUntilNextStep;
import simlejos.robotics.SampleProvider;

import simlejos.hardware.port.SensorPort;
import simlejos.hardware.sensor.EV3ColorSensor;

/** 
 * class for the handling the light sensor localization. * 
 * @author Sia Ham
 *
 */
public class LightLocalizer {
  public static float[] CSR = new float[colorSensorR.sampleSize()];
  public static float[] CSL = new float[colorSensorL.sampleSize()];

  /**
   * Localize the position of the robot using light sensor localizer.
   ** @author Sia Ham
   */

  public static void localize() {
    System.out.println("LightLocalizer is localizing the robot");
    turnBy(90);

    while (true) {
      goForward();
      System.out.println(
          "Color sensor: right = " + readColorRight() 
          + ", left = " + readColorLeft());
      if (readColorRight() < 30) {
        break;
      }
    }

    moveStraightFor(0.15);
    turnBy(-90);

    while (true) {
      goForward();
      System.out.println(
          "Color sensor: right = " + readColorRight() 
          + ", left = " + readColorLeft());
      if (readColorLeft() < 50) {
        break;
      }
    }
    moveStraightFor(0.15);

    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0); 

  }

  /** read left color sensor readings. */
  public static float readColorLeft() {
    colorSensorL.fetchSample(CSL, 0);
    float readL = CSL[0];
    return readL;
  }

  /** read right color sensor readings. */
  public static float readColorRight() {
    colorSensorR.fetchSample(CSR, 0);
    float readR = CSR[0];
    return readR;
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
   * Go forward straight.
   */
  public static void goForward() {
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    leftMotor.forward();
    rightMotor.forward();
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that
   * angle.
   * * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertDistance(double distance) {
    return (int) ((distance * 180) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Moves the robot straight for the given distance.
   * 
   * @param distance in feet (tile sizes), may be negative
   */
  public static void moveStraightFor(double distance) {
    leftMotor.rotate(convertDistance(TILE_SIZE * distance), true);
    rightMotor.rotate(convertDistance(TILE_SIZE * distance), false);
  }

}
