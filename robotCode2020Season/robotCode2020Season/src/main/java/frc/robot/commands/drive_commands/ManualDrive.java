/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.S_Drive;

public class ManualDrive extends CommandBase {
  /**
   * Creates a new ManualDrive.
   */
  
  S_Drive sub;

  public ManualDrive(S_Drive sub) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sub);
    this.sub = sub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: make a suffleboard selector for ease of switching so drivers can try the different modes easier
    //--tank - linear
    sub.runMotor(RobotContainer.driveController.getRawAxis(Constants.leftAxisY), RobotContainer.driveController.getRawAxis(Constants.rightAxisY));
    //--tank - exponential joystick input conversion
    //double kExpCurve = SmartDashboard.getNumber("kExpCurve", 3); // higher the number the faster the exponential function grows (can see desmos image)
    //double inL = RobotContainer.driveController.getRawAxis(Constants.leftAxisY);
    //double inR = RobotContainer.driveController.getRawAxis(Constants.rightAxisY);
    //double outL = Math.signum(inL)*(Math.pow(2,kExpCurve*Math.abs(inL))-1)/(Math.pow(2,kExpCurve)-1);
    //double outR = Math.signum(inR)*(Math.pow(2,kExpCurve*Math.abs(inR))-1)/(Math.pow(2,kExpCurve)-1);
    //sub.runMotor(outL, outR);
    //--arcade drive - one stick
    //sub.arcadeDrive(RobotContainer.driveController.getRawAxis(Constants.leftAxisY), RobotContainer.driveController.getRawAxis(Constants.leftAxisX));
    //---GTA drive
    //sub.arcadeDrive(RobotContainer.driveController.getRawAxis(Constants.rightTrigger) -
    //                RobotContainer.driveController.getRawAxis(Constants.leftTrigger), //idk if trigger is an axis
    //                RobotContainer.driveController.getRawAxis(Constants.rightAxisX));
    //--curvature drive - 1 stick
    //sub.curveDrive(RobotContainer.driveController.getRawAxis(Constants.leftAxisY), RobotContainer.driveController.getRawAxis(Constants.leftAxisX));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
