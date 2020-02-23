/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.limelight.LimeLight;
import frc.robot.subsystems.S_Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class LemonAlign extends PIDCommand {
  /**
   * Creates a new LemonAlign.
   */
  
  private static double turnP = 0.023, turnI = 0, turnD =0.00115;
  private S_Drive sub;
  private LimeLight limelight;

  public LemonAlign(S_Drive sub, LimeLight limelight) {
    super(
        // The controller that the command will use
        new PIDController(turnP, turnI, turnD),
        // This should return the measurement
        () -> limelight.getHorizontalOffset(),
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          // Use the output here
          //TODO: may need contraints for output as well as change PD gain values - also may need to negate a value
          sub.arcadeDrive(S_Drive.getTranslatedJoystickAxisValue(RobotContainer.driveController, Constants.leftAxisY, 0.0), output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sub);
    this.sub = sub;
    this.limelight = limelight;
    limelight.setPipeline(Constants.lemonPipeline);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
