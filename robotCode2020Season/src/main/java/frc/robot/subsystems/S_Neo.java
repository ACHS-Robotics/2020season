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
import com.revrobotics.CANSparkMax.IdleMode;
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
  
  //private CANSparkMax motor1;
  private CANEncoder encoderRight;
  private CANEncoder encoderLeft;
  private CANPIDController pidcontrollerR;
  private CANPIDController pidcontrollerL;
  double tareEncPositionR = 0;
  double tareEncPositionL = 0;
  double kP = 0.075, kI = 0, kD = 0,kFF = 0, kMinOutput = -1, kMaxOutput = 1;
  //final double rev2dist = 6*Math.PI/10.7/12;
  final double dist2rev = 12/(6*Math.PI)*10.71; // conversion factor from distance in feet of robot movement to neo revolutions
  //double kSetpoint = 0; //in revolutions

  public S_Neo(){
    
    lfmoto = new CANSparkMax(RobotMap.NEOlf, MotorType.kBrushless);
    lfmoto.restoreFactoryDefaults();
    lfmoto.setInverted(false);
    lfmoto.setIdleMode(IdleMode.kCoast);

    lbmoto = new CANSparkMax(RobotMap.NEOlb, MotorType.kBrushless);
    lbmoto.restoreFactoryDefaults();
    lbmoto.setInverted(false);
    lbmoto.setIdleMode(IdleMode.kCoast);
    lbmoto.follow(lfmoto);

    rfmoto = new CANSparkMax(RobotMap.NEOrf, MotorType.kBrushless);
    rfmoto.restoreFactoryDefaults();
    rfmoto.setInverted(true);
    rfmoto.setIdleMode(IdleMode.kCoast);

    rbmoto = new CANSparkMax(RobotMap.NEOrb, MotorType.kBrushless);
    rbmoto.restoreFactoryDefaults();
    rbmoto.setInverted(true);
    rbmoto.setIdleMode(IdleMode.kCoast);
    rbmoto.follow(rfmoto);

    encoderRight = rfmoto.getEncoder();
    encoderRight = rfmoto.getEncoder(EncoderType.kHallSensor, 42);

    encoderLeft = lfmoto.getEncoder();
    encoderLeft = lfmoto.getEncoder(EncoderType.kHallSensor, 42);

    pidcontrollerR = rfmoto.getPIDController();
    pidcontrollerR.setFeedbackDevice(encoderRight);

    pidcontrollerL = lfmoto.getPIDController();
    pidcontrollerL.setFeedbackDevice(encoderLeft);

    SmartDashboard.putNumber("P", kP);
    SmartDashboard.putNumber("I", kI);
    SmartDashboard.putNumber("D", kD);
    SmartDashboard.putNumber("FF", kFF);
    SmartDashboard.putNumber("MinOutput", kMinOutput);
    SmartDashboard.putNumber("MaxOutput", kMaxOutput);
    SmartDashboard.putNumber("Setpoint", 0);

    pidcontrollerR.setP(kP);
    pidcontrollerL.setP(kP);
    pidcontrollerR.setI(kI);
    pidcontrollerL.setI(kI);
    pidcontrollerR.setD(kD);
    pidcontrollerL.setD(kD);
    pidcontrollerR.setOutputRange(kMinOutput, kMaxOutput);
    pidcontrollerL.setOutputRange(kMinOutput, kMaxOutput);

/* no motor for now
    //pid testing
    motor1 = new CANSparkMax(RobotMap.NEO1, MotorType.kBrushless);
    motor1.restoreFactoryDefaults();
    motor1.setInverted(false);

    //  encoder1 = motor1.getEncoder(); //no params uses integrated controller
    encoder1 = motor1.getEncoder(EncoderType.kHallSensor, 42);
    
    pidcontroller1 = motor1.getPIDController();
    pidcontroller1.setFeedbackDevice(encoder1);


    SmartDashboard.putNumber("P", kP);
    SmartDashboard.putNumber("I", kI);
    SmartDashboard.putNumber("D", kD);
    SmartDashboard.putNumber("FF", kFF);
    SmartDashboard.putNumber("MinOutput", kMinOutput);
    SmartDashboard.putNumber("MaxOutput", kMaxOutput);
    SmartDashboard.putNumber("Setpoint", 0);

    setPID();
*/
  }
  
 //TODO: put back
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
      pidcontrollerR.setP(kP);
      pidcontrollerL.setP(kP);
    }
    if (i != kI) {
      kI = i;
      pidcontrollerR.setI(kI);
      pidcontrollerL.setI(kI);
    }
    if (d != kD) {
      kD = d;
      pidcontrollerR.setD(kD);
      pidcontrollerL.setD(kD);
    }
    if (ff != kFF) {
      kFF = ff;
      pidcontrollerR.setFF(kFF);
      pidcontrollerL.setFF(kFF);
    }
    if (minOutput != kMinOutput || maxOutput != kMaxOutput) {
      kMinOutput = minOutput;
      kMaxOutput = maxOutput;
      pidcontrollerR.setOutputRange(kMinOutput, kMaxOutput);
      pidcontrollerL.setOutputRange(kMinOutput, kMaxOutput);
    }
    /*
    System.out.println("setpoint: "+ setpoint);
    System.out.println("P: "+ kP);
    System.out.println("I: "+ kI);
    System.out.println("D: "+ kD);
    System.out.println("max output: " + pidcontrollerL.getOutputMax());
    SmartDashboard.putNumber("applied motor output lf", lfmoto.getAppliedOutput());
    SmartDashboard.putNumber("applied motor output rf", rfmoto.getAppliedOutput());
    SmartDashboard.putNumber("applied motor output lb", lbmoto.getAppliedOutput());
    SmartDashboard.putNumber("applied motor output rb", rbmoto.getAppliedOutput());

    System.out.println("applied motor output lf " + lfmoto.getAppliedOutput());
    System.out.println("applied motor output rf " + rfmoto.getAppliedOutput());
    System.out.println("applied motor output lb " + lbmoto.getAppliedOutput());
    System.out.println("applied motor output rb " + rbmoto.getAppliedOutput());
*/

    // note multiply by rev 2 makes setpoint in terms of feet driven by robot
    System.out.println(setpoint*dist2rev);
    pidcontrollerR.setReference((setpoint*dist2rev)+ tareEncPositionR, ControlType.kPosition); //note that this should be adjusted based on relative position
    pidcontrollerL.setReference((setpoint*dist2rev)+ tareEncPositionL, ControlType.kPosition); 

  }

  public void getSDInfo(){ //send info to smart dashboard
    SmartDashboard.putNumber("encoder position", getRelativePosition());
    SmartDashboard.putNumber("encoder velocity Right", encoderRight.getVelocity());
  }

  public void resetEncPosition(){ // if this is called it kills the cpu usage due to get position so don't put in loop
    tareEncPositionR = encoderRight.getPosition();
    tareEncPositionL = encoderLeft.getPosition();
  }

  //this is kinda useless right now
  public double getRelativePosition(){ // TODO: change to be parameterized for any encoder
    return encoderRight.getPosition(); //- tareEncPosition; TODO: make setpoints relative based
  }
/*
  public double convertEncRelative2Raw(double endpoint, String encoderID){ //TODO: don't use strings like a dumb boi :P
    if (encoderID.equals("R")){
      endpoint = endpoint + tareEncPositionR;
    }
    else if (encoderID.equals("L")){
      endpoint = endpoint + tareEncPositionL;
    }
    else {
      System.out.println("encoderID is not valid!!!"); //idk how to throw errors in java
      throw new Error("encoderID is not valid!!!");
    }

    return endpoint;
  }
*/

/*
  public void runMotor(double left, double right){
   
    lfmoto.set(left);
    lbmoto.set(left);
    
    rfmoto.set(right);
    rbmoto.set(right);

  }
*/
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new RunNeo());
  }
}
