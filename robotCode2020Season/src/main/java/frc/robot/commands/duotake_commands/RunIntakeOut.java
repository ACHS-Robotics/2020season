/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.duotake_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.S_Duotake;

public class RunIntakeOut extends CommandBase {
  /**
   * Creates a new RunIntakeOut.
   */
    S_Duotake sub;

  public RunIntakeOut(S_Duotake sub) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sub);
    this.sub = sub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sub.runIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted){
      sub.stopIntake();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
