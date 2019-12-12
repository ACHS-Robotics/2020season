/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  
  public Joystick driveController = new Joystick(RobotMap.logitechDriveCont);
  public Button driveButtonA = new JoystickButton(driveController, RobotMap.buttonA);
  public Button driveButtonB = new JoystickButton(driveController, RobotMap.buttonB);
  public Button driveButtonX = new JoystickButton(driveController, RobotMap.buttonX);
  public Button driveButtonY = new JoystickButton(driveController, RobotMap.buttonY);
  public Button driveRightBumper = new JoystickButton(driveController, RobotMap.rightBumper);
  public Button driveLeftBumper = new JoystickButton(driveController, RobotMap.leftBumper);


  public Joystick weaponsController = new Joystick(RobotMap.gamecubeWeaponsCont);
  public Button weaponsButtonA = new JoystickButton(weaponsController, RobotMap.gcButtonA);
  public Button weaponsButtonB = new JoystickButton(weaponsController, RobotMap.gcButtonB);
  public Button weaponsButtonX = new JoystickButton(weaponsController, RobotMap.gcButtonX);
  public Button weaponsButtonY = new JoystickButton(weaponsController, RobotMap.gcButtonY);
  public Button weaponsRightBumper = new JoystickButton(weaponsController, RobotMap.gcRightBumper); // z button
  public Button weaponsStartButton = new JoystickButton(weaponsController, RobotMap.gcStartButton);
  public Button weaponsUp = new JoystickButton(weaponsController, RobotMap.gcUp);
  public Button weaponsDown = new JoystickButton(weaponsController, RobotMap.gcDown);
  public Button weaponsLeft = new JoystickButton(weaponsController, RobotMap.gcLeft);
  public Button weaponsRight = new JoystickButton(weaponsController, RobotMap.gcRight);
  public Button weaponsLeftTrigger = new JoystickButton(weaponsController, RobotMap.gcLeftTriggerButton);
  public Button weaponsRightTrigger = new JoystickButton(weaponsController, RobotMap.gcRightTriggerButton);



  public OI(){
//driveCont
// driveButtonX.whenPressed(new ToggleFlaps()); //deploy flaps (toggle)
//  driveButtonY.whenPressed(new MoveLift()); //toggle lift
    
  }
  

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
