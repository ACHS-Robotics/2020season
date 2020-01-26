/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.S_PushClimb;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetLinearActuatorLength extends PIDCommand {
  /**
   * Creates a new SetLinearActuatorLength.
   */
  S_PushClimb sub;
  
  public SetLinearActuatorLength(S_PushClimb sub) {
    super(
        // The controller that the command will use
        new PIDController(0.75, 0, 0),
        // This should return the measurement
        () -> sub.getExtendedLength(),
        // This should return the setpoint (can also be a constant)
        6,
        // This uses the output
        output -> {
          // Use the output here
          sub.setMotorOutput(output);
          System.out.println("LinAct length" + sub.getExtendedLength());
        });
    addRequirements(sub);
    this.sub = sub;
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180); // maybe change depending on the navx
    getController().setTolerance(0.2, 0);
    //TODO: ? getController().setIntegratorRange(minimumIntegral, maximumIntegral);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
