/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive_commands;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
  double startFwdConstant = 3; //above 1
  double startBackConstant = 0.5; //between 0 and 1
  double startTurnConstant = 3; //above 1


  public ManualDrive(S_Drive sub) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sub);
    SmartDashboard.putNumber("kExpCurveFwd", startFwdConstant); // higher the number the faster the exponential function grows (can see desmos image)
    SmartDashboard.putNumber("kLineSlopeBack", startBackConstant); // also stands as max magnitude of output for backward movement
    SmartDashboard.putNumber("kExpCurveTurn", startTurnConstant);

    this.sub = sub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //ensuring positive values where wanted/expected
    double lyAxis = S_Drive.getTranslatedJoystickAxisValue(RobotContainer.driveController, Constants.leftAxisY, 0.0);
    double ryAxis = S_Drive.getTranslatedJoystickAxisValue(RobotContainer.driveController, Constants.rightAxisY, 0.0);
    double lxAxis = S_Drive.getTranslatedJoystickAxisValue(RobotContainer.driveController, Constants.leftAxisX, -90.0);
    double rxAxis = S_Drive.getTranslatedJoystickAxisValue(RobotContainer.driveController, Constants.rightAxisX, -90.0);
    double lTrigger = Math.abs(RobotContainer.driveController.getRawAxis(Constants.leftTrigger));
    double rTrigger = Math.abs(RobotContainer.driveController.getRawAxis(Constants.rightTrigger));
    //TODO: make a suffleboard selector for ease of switching so drivers can try the different modes easier
    //String controlMode = Shuffleboard.getTab("Drive")
    //  .add("Control Mode", "Linear Tank")
    //  .withWidget(BuiltInWidgets.kComboBoxChooser)
    //  .withProperties(Map.of(""))
    //switch(controlMode)
    //--tank - linear
    //System.out.println("lyAxis: "+ lyAxis + " ryAxis: "+ ryAxis);
    //sub.runMotor(lyAxis,ryAxis);
    //--tank - exponential joystick input conversion
    //double kExpCurve = SmartDashboard.getNumber("kExpCurve", startExpConstant); // higher the number the faster the exponential function grows (can see desmos image)
    //double outL = Math.signum(inL)*(Math.pow(2,kExpCurve*Math.abs(lyAxis))-1)/(Math.pow(2,kExpCurve)-1);
    //double outR = Math.signum(inR)*(Math.pow(2,kExpCurve*Math.abs(ryAxis))-1)/(Math.pow(2,kExpCurve)-1);
    //sub.runMotor(outL, outR);
    //--arcade drive - one stick
    //sub.arcadeDrive(lyAxis, lxAxis);
    //---GTA drive
    //sub.arcadeDrive(rTrigger - lTrigger, lxAxis);
    double kExpCurveFwd = SmartDashboard.getNumber("kExpCurveFwd", startFwdConstant); // higher the number the faster the exponential function grows (can see desmos image)
    double kLineSlopeBack = SmartDashboard.getNumber("kLineSlopeBack", startBackConstant); // higher the number the faster the exponential function grows (can see desmos image)
    double kExpCurveTurn = SmartDashboard.getNumber("kExpCurveTurn", startTurnConstant);
    double triggersValue = rTrigger - lTrigger;
    double adjustedOutput;
    if (triggersValue >= 0) {
      adjustedOutput = Math.signum(triggersValue)*(Math.pow(2,kExpCurveFwd*Math.abs(triggersValue))-1)/(Math.pow(2,kExpCurveFwd)-1);
    }
    else {
      adjustedOutput = triggersValue*kLineSlopeBack;
    }
    double transformedAxisValue = Math.signum(lxAxis)*(Math.pow(2,kExpCurveTurn*Math.abs(lxAxis))-1)/(Math.pow(2,kExpCurveTurn)-1);
    sub.arcadeDrive(adjustedOutput, transformedAxisValue);
    //--curvature drive - 1 stick
    //sub.curveDrive(lyAxis, lxAxis);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      sub.runMotor(0, 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
