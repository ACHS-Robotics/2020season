/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  // neo sparkmaxes -- the ids on the side of the pdp with lower numbers has their
  // can id set to 1+pdpNumber
  public static final int NEOlf = 14;
  public static final int NEOlb = 15;
  public static final int NEOrf = 1;
  public static final int NEOrb = 2;

  public static final int NEO1 = 4; // seperate testing

  public static final double driveGearRatio = 10.71;

  // conversion factors
  public static final double neoRevs2meters = 0.1524 * Math.PI / driveGearRatio;
  public static final double meters2NeoRevs = 1 / neoRevs2meters;

  // trajectory related value (in SI units) TODO: POPULATE VALUES
  public static final double trackWidth = 0;
  public static final double kS = 0;
  public static final double kV = 0;
  public static final double kA = 0;
  public static final double kPTraj = 0;
  public static final double maxTrajVelocity = 0;
  public static final double maxTrajAcceleration = 0;

  // old

  // controllers
  public static final int logitechDriveCont = 0;
  public static final int gamecubeWeaponsCont = 1;
  // logitech button
  public static final int buttonA = 1;
  public static final int buttonB = 2;
  public static final int buttonX = 3;
  public static final int buttonY = 4;
  public static final int leftBumper = 5;
  public static final int rightBumper = 6;
  public static final int backButton = 7;
  public static final int startButton = 8;
  public static final int dpad = RobotContainer.driveController.getPOV(0);
  public static final int dpadUp = 0;
  public static final int dpadDown = 180;
  public static final int dpadLeft = 270;
  public static final int dpadRight = 90;

  // joystick axes
  public static final int leftAxisX = 0;
  public static final int leftAxisY = 1;
  public static final int rightAxisX = 4;
  public static final int rightAxisY = 5;
  // triggers
  public static final int leftTrigger = 2;
  public static final int rightTrigger = 3;

  // gamecube buttons
  public static final int gcButtonA = 2;
  public static final int gcButtonB = 3;
  public static final int gcButtonX = 1;
  public static final int gcButtonY = 4;
  public static final int gcRightBumper = 8; // z button
  public static final int gcStartButton = 10;
  // dpad buttons
  public static final int gcUp = 13;
  public static final int gcDown = 15;
  public static final int gcLeft = 16;
  public static final int gcRight = 14;
  // joystick axes
  public static final int gcLeftAxisX = 0; // control stick
  public static final int gcLeftAxisY = 1;
  public static final int gcRightAxisX = 5; // c stick
  public static final int gcRightAxisY = 2;
  // triggers analog (probs not used) -- ignore
  public static final int gcLeftTrigger = 3;
  public static final int gcRightTrigger = 4;
  // trigger digital (buttons)
  public static final int gcLeftTriggerButton = 5;
  public static final int gcRightTriggerButton = 6;

  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
