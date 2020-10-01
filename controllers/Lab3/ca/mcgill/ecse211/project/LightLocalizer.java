package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.sleepFor;
import static simlejos.ExecutionController.waitUntilNextStep;
import simlejos.robotics.SampleProvider;

import simlejos.hardware.port.SensorPort;
import simlejos.hardware.sensor.EV3ColorSensor;

public class LightLocalizer {
  public static final SampleProvider colorSensorR = new EV3ColorSensor(SensorPort.S2).getRedMode();
  public static final SampleProvider colorSensorL = new EV3ColorSensor(SensorPort.S3).getRedMode();
  public static float[] CSR = new float[colorSensorR.sampleSize()];
  public static float[] CSL = new float[colorSensorL.sampleSize()];

  public static void localize() {
    System.out.println("LightLocalizer is localizing the robot");
    while (readColorRight() < 150 && readColorLeft() < 150) {
      goForward();
      readColorRight();
      readColorLeft();
      System.out.println("Color sensor: right = " + readColorRight() + ", left = " + readColorLeft());
    }

    turnBy(90);
    /*
    for (int i = 0; i < 50; i++) {
      waitUntilNextStep();
    }

    while (true) {
      goForward();
      System.out.println("Color sensor: right = " + readColorRight() + ", left = " + readColorLeft());
      if (readColorRight() >= 157 || readColorLeft() >= 157) {
        break;
      }

    }

    for (int i = 0; i < 50; i++) {
      waitUntilNextStep();
    }

    turnBy(-90);
     */

  }

  /** Returns the filtered distance between the US sensor and an obstacle in cm. */
  public static float readColorLeft() {
    colorSensorL.fetchSample(CSL, 0);
    float readL = CSL[0] ;
    return readL;
  }
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
}
