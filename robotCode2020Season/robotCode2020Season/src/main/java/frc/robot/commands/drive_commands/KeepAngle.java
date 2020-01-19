/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.S_Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class KeepAngle extends PIDCommand {
  /**
   * Creates a new KeepAngle.
   */
  private static double turnP = 0.013, turnI = 0, turnD =0.00115;
  private S_Drive sub;

  public KeepAngle(S_Drive sub) {

    super(
      // The controller that the command will use
      new PIDController(turnP, turnI, turnD),
      // This should return the measurement
      () -> sub.gyro.getYaw(), //maybe Math.IEEtAngleEremainder(sub.gyro.getYaw(), 360) instead
      // This should return the setpoint (can also be a constant)
      0, //go straight
      // This uses the output
      output -> {
        // Use the output here
        sub.arcadeDrive(RobotContainer.driveController.getRawAxis(Constants.rightAxisY),output);
        //System.out.println("yaw: " + sub.gyro.getYaw());
      });
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(sub);
    this.sub = sub;
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180); // maybe change depending on the navx
    getController().setTolerance(0, 0); //copied constants (may need adjusting and put into constants.java)
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void initialize() {
    sub.gyro.reset();
  }

}