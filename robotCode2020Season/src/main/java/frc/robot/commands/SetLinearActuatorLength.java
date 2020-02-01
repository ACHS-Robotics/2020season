/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.S_Climb;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetLinearActuatorLength extends PIDCommand {
  /**
   * Creates a new SetLinearActuatorLength.
   */
  S_Climb sub;
  double setpoint;
  
  public SetLinearActuatorLength(S_Climb sub, double setpointInInches) {
    super(
        // The controller that the command will use
        new PIDController(20, 0, 0), //may need to raise P or add I for when there is a load on the actuator
        // This should return the measurement
        () -> sub.getPercentExtended(),
        // This should return the setpoint (can also be a constant)
        setpointInInches/12.0,
        // This uses the output
        output -> {
          // Use the output here
          sub.setMotorOutput(output);
          //System.out.println("Actuator length" + sub.getPercentExtended());
        });
    addRequirements(sub);
    this.sub = sub;
    this.setpoint = setpointInInches;
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.01, 0); //at sepoint within 1 percent of a foot
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean illegalSetpointRange = false;
    if (setpoint < 0.0 && setpoint > 12.0){
      System.out.println("problematic linear acturator setpoint!");
      illegalSetpointRange = true;
    }
    return getController().atSetpoint() || illegalSetpointRange;
  }
}
