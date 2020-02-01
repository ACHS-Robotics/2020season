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

public class S_Climb extends SubsystemBase {
  /**
   * Creates a new S_Climb.
   */
  private CANSparkMax motor;
  private AnalogPotentiometer pot;

  public S_Climb() {
    motor = new CANSparkMax(Constants.climbMotoPort, MotorType.kBrushed);
    motor.restoreFactoryDefaults();
    motor.setInverted(true);
    motor.setIdleMode(IdleMode.kBrake);
    pot = new AnalogPotentiometer(3);
  }

  public double getPercentExtended(){ // returns range ~0 to ~1
    return (-(pot.get()-1))/0.87; // 0.87 is experimental constant based on the pot return values from min and max length
  }

  public void setMotorOutput(double percent){
    motor.set(percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
