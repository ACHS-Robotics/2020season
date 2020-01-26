/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class S_PushClimb extends SubsystemBase {
  /**
   * Creates a new S_PushClimb.
   */
  private CANSparkMax motor;
  private AnalogPotentiometer pot;

  public S_PushClimb() {
    motor = new CANSparkMax(Constants.climbMotoPort, MotorType.kBrushed);
    motor.restoreFactoryDefaults();
    motor.setInverted(true);
    motor.setIdleMode(IdleMode.kBrake);
    pot = new AnalogPotentiometer(3, 12.0/5.0);
  }

  public double getExtendedLength(){ // returns inches
    return pot.get();
  }

  public void setMotorOutput(double percent){
    motor.set(percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
