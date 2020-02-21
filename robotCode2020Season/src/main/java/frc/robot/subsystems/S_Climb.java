/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class S_Climb extends SubsystemBase {
  /**
   * Creates a new S_Climb.
   */
  private TalonFX leftFalcon;

  public S_Climb() {
    leftFalcon = new TalonFX(Constants.winchLeftFalcon);
    leftFalcon.set(ControlMode.PercentOutput, 0);
    leftFalcon.configFactoryDefault();
    leftFalcon.setNeutralMode(NeutralMode.Brake);
  
  }

  public void setFalcons(double percent){
    leftFalcon.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
