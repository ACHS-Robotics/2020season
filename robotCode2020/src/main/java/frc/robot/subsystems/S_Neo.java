/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.RunNeo;

/**
 * Add your docs here.
 */
public class S_Neo extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax lfmoto, lbmoto, rfmoto, rbmoto;
  
  private CANSparkMax motor1;
  private CANEncoder encoder1;
  private CANPIDController pidcontroller1;
  double tareEncPosition = 0;
  double kP = 0.1, kI = 1e-4, kD = 1,kFF = 0, kMinOutput = -1, kMaxOutput = 1;
  //double kSetpoint = 0; //in revolutions

  public S_Neo(){
    lfmoto = new CANSparkMax(RobotMap.NEOlf, MotorType.kBrushless);
    lfmoto.restoreFactoryDefaults();
    lfmoto.setInverted(false);

    lbmoto = new CANSparkMax(RobotMap.NEOlb, MotorType.kBrushless);
    lbmoto.restoreFactoryDefaults();
    lbmoto.setInverted(false);

    rfmoto = new CANSparkMax(RobotMap.NEOrf, MotorType.kBrushless);
    rfmoto.restoreFactoryDefaults();
    rfmoto.setInverted(true);

    rbmoto = new CANSparkMax(RobotMap.NEOrb, MotorType.kBrushless);
    rbmoto.restoreFactoryDefaults();
    rbmoto.setInverted(true);
    

    //pid testing
    motor1 = new CANSparkMax(RobotMap.NEO1, MotorType.kBrushless);
    motor1.restoreFactoryDefaults();
    motor1.setInverted(false);

    //  encoder1 = motor1.getEncoder(); //no params uses integrated controller
    encoder1 = motor1.getEncoder(EncoderType.kHallSensor, 42);
    
    pidcontroller1 = motor1.getPIDController();
    pidcontroller1.setFeedbackDevice(encoder1);

    /*
    pidcontroller1.setP(kP);
    pidcontroller1.setI(kI);
    pidcontroller1.setD(kD);
    pidcontroller1.setFF(kFF);
    pidcontroller1.setOutputRange(kMinOutput, kMaxOutput);
    */

    SmartDashboard.putNumber("P", kP);
    SmartDashboard.putNumber("I", kI);
    SmartDashboard.putNumber("D", kD);
    SmartDashboard.putNumber("FF", kFF);
    SmartDashboard.putNumber("MinOutput", kMinOutput);
    SmartDashboard.putNumber("MaxOutput", kMaxOutput);
    SmartDashboard.putNumber("Setpoint", 0);

    setPID();

  }
  

  public void setPID(){
    double p = SmartDashboard.getNumber("P", 0);
    double i = SmartDashboard.getNumber("I", 0);
    double d = SmartDashboard.getNumber("D", 0);
    double ff = SmartDashboard.getNumber("FF", 0);
    double minOutput = SmartDashboard.getNumber("MinOutput", 0);
    double maxOutput = SmartDashboard.getNumber("MaxOutput", 0);
    double setpoint = SmartDashboard.getNumber("Setpoint", 0);


    if (p != kP) {
      kP = p;
      pidcontroller1.setP(kP);
    }
    if (i != kI) {
      kI = i;
      pidcontroller1.setI(kI);
    }
    if (d != kD) {
      kD = d;
      pidcontroller1.setD(kD);
    }
    if (ff != kFF) {
      kFF = ff;
      pidcontroller1.setFF(kFF);
    }
    if (minOutput != kMinOutput || maxOutput != kMaxOutput) {
      kMinOutput = minOutput;
      kMaxOutput = maxOutput;
      pidcontroller1.setOutputRange(kMinOutput, kMaxOutput);
    }
    
    System.out.println("setpoint: "+ setpoint);
    System.out.println("P: "+ kP);
    System.out.println("I: "+ kI);
    System.out.println("D: "+ kD);

    pidcontroller1.setReference(setpoint, ControlType.kPosition); //note that this should be adjusted based on relative position

  }

  public void getSDInfo(){ //send info to smart dashboard
    SmartDashboard.putNumber("encoder position", getRelativePosition());
    SmartDashboard.putNumber("encoder velocity", encoder1.getVelocity());

  }

  public void resetEncPosition(){
    tareEncPosition = encoder1.getPosition();
  }
  public double getRelativePosition(){ // TODO: change to be parameterized for any encoder
    return encoder1.getPosition(); //- tareEncPosition; TODO: make setpoints relative based
  }



  public void runMotor(double left, double right){
   
    lfmoto.set(left);
    lbmoto.set(left);
    
    rfmoto.set(right);
    rbmoto.set(right);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new RunNeo());
  }
}
