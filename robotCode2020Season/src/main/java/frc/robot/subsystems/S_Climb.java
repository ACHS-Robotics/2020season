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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class S_Climb extends SubsystemBase {
  /**
   * Creates a new S_Climb.
   */
  private CANSparkMax motor;
  private AnalogPotentiometer pot;
  public static DoubleSolenoid leftSol = new DoubleSolenoid(Constants.climbLeftSolenoidPort, Constants.climbLeftSolenoidForward, Constants.climbLeftSolenoidReverse);
  public static DoubleSolenoid rightSol = new DoubleSolenoid(Constants.climbRightSolenoidPort, Constants.climbRightSolenoidForward, Constants.climbRightSolenoidReverse);
  private TalonFX leftFalcon = new TalonFX(Constants.winchLeftFalcon);
  private TalonFX rightFalcon = new TalonFX(Constants.winchRightFalcon);

  public S_Climb() {
    motor = new CANSparkMax(Constants.climbMotoPortLA, MotorType.kBrushed);
    motor.restoreFactoryDefaults();
    motor.setInverted(true);
    motor.setIdleMode(IdleMode.kBrake);
    pot = new AnalogPotentiometer(3);
    
    leftFalcon.set(ControlMode.PercentOutput, 0);
    rightFalcon.set(ControlMode.PercentOutput, 0);
    leftFalcon.configFactoryDefault();
    rightFalcon.configFactoryDefault();
    leftFalcon.setNeutralMode(NeutralMode.Brake);
    rightFalcon.setNeutralMode(NeutralMode.Brake);

  }

  public double getPercentExtended(){ // returns range ~0 to ~1
    return (-(pot.get()-1))/0.87; // 0.87 is experimental constant based on the pot return values from min and max length
  }

  public void setMotorOutput(double percent){
    motor.set(percent);
  }
  public void setFalcons(double percent){
    leftFalcon.set(ControlMode.PercentOutput, percent);
    rightFalcon.set(ControlMode.PercentOutput, percent);
  }
  public void togglePneumatics(){
    if (rightSol.get() == Value.kForward && leftSol.get() == Value.kForward){
      rightSol.set(Value.kReverse);
      leftSol.set(Value.kReverse);
    }
    else if (rightSol.get() == Value.kReverse && leftSol.get() == Value.kReverse){
      rightSol.set(Value.kForward);
      leftSol.set(Value.kForward);
    }
    else { // fixes if pneumatics are in opposite states
      rightSol.set(Value.kReverse);
      leftSol.set(Value.kReverse);
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
