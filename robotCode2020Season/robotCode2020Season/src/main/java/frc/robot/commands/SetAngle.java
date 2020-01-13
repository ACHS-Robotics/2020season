/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.S_Neo;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetAngle extends PIDCommand {
  /**
   * Creates a new SetAngle.
   */
  //private static double turnP = 0.05, turnI = 0, turnD =0, setpoint = 10;

  public SetAngle(S_Neo sub) {

    super(
      // The controller that the command will use
      //new PIDController(sub.kTurnP, sub.kTurnI, sub.kTurnD),
      new PIDController( // for tuning
        SmartDashboard.getNumber("turnP", 0),
        SmartDashboard.getNumber("turnI", 0),
        SmartDashboard.getNumber("turnD", 0)
      ),
      // This should return the measurement
      () -> sub.gyro.getYaw(), //maybe Math.IEEtAngleEremainder(sub.gyro.getYaw(), 360) instead
      // This should return the setpoint (can also be a constant)
      SmartDashboard.getNumber("turnSetpoint", 0), //for tuning
      // This uses the output
      output -> {
        // Use the output here
        sub.arcadeDrive(0,output);
        //System.out.println("yaw: " + sub.gyro.getYaw());
      });
    // Use addRequirements() here to declare subsystem dependencies.
    System.out.println("turnP: " + SmartDashboard.getNumber("turnP", 0));
    System.out.println("turnI: " + SmartDashboard.getNumber("turnI", 0));
    System.out.println("turnD: " + SmartDashboard.getNumber("turnD", 0));
    System.out.println("turnSetpoint: " + SmartDashboard.getNumber("turnSetpoint", 0));
    addRequirements(sub);
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180); // maybe change depending on the navx
    getController().setTolerance(5, 10); //copied constants (may need adjusting and put into constants.java)
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
