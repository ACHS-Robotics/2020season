/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  //neo sparkmaxes - TODO: fix later
  public static int NEOlf = 0;
  public static int NEOlb = 1;
  public static int NEOrf = 12;
  public static int NEOrb = 13;

  public static int NEO1 = 4; // seperate testing


  //old 


  //controllers
  public static final int logitechDriveCont = 0;
  public static final int gamecubeWeaponsCont = 1;
  //logitech button
  public static final int buttonA = 1;
  public static final int buttonB = 2;
  public static final int buttonX = 3;
  public static final int buttonY = 4;
  public static final int leftBumper = 5;
  public static final int rightBumper = 6;
  public static final int backButton = 7;
  public static final int startButton = 8;
  //joystick axes
  public static final int leftAxisX = 0;
  public static final int leftAxisY = 1;
  public static final int rightAxisX = 4;
  public static final int rightAxisY = 5;
  //triggers
  public static final int leftTrigger = 2;
  public static final int rightTrigger = 3;

  //gamecube buttons
  public static final int gcButtonA = 2;
  public static final int gcButtonB = 3;
  public static final int gcButtonX = 1;
  public static final int gcButtonY = 4;
  public static final int gcRightBumper = 8; // z button
  public static final int gcStartButton = 10;
  //dpad buttons
  public static final int gcUp = 13;
  public static final int gcDown = 15;
  public static final int gcLeft = 16;
  public static final int gcRight = 14;
  //joystick axes
  public static final int gcLeftAxisX = 0; // control stick
  public static final int gcLeftAxisY = 1;
  public static final int gcRightAxisX = 5; // c stick
  public static final int gcRightAxisY = 2;
  //triggers analog (probs not used) -- ignore
  public static final int gcLeftTrigger = 3; 
  public static final int gcRightTrigger = 4;
  //trigger digital (buttons)
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
