/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.spinner_commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.S_Spinner;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RotationControl extends PIDCommand {
  /**
   * Creates a new RotationControl.
   */

  private S_Spinner sub;

  public RotationControl(S_Spinner sub) {
    super(
        // The controller that the command will use
        new PIDController(0.3, 0, 0), 
        // This should return the measurement
        () -> {
          int[] list = sub.getColorList();
          return list[0]+list[1]+list[2]+list[3]; // this may just straigt up just not work thanks to input not being continuous
        }, // TODO:may want to have some dectection for any errors in sensing colors in order
        // This should return the setpoint (can also be a constant)
        32, // number of colors
        // This uses the output
        output -> {
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sub);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0);
    this.sub = sub;
    
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
    sub.resetColorListCounters();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
