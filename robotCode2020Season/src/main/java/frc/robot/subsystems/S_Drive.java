/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;

public class S_Drive extends SubsystemBase {
  /**
   * Creates a new S_Drive.
   */
  private CANSparkMax lfmoto, lbmoto, rfmoto, rbmoto;
  
  //private CANSparkMax motor1;
  public AHRS gyro = new AHRS(Port.kUSB); // multiple usb options usure if this one is correct

  public DifferentialDrive diffDrive;
  
  //trajectory stuff
  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odometry;
  private Pose2d pose;
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
  private PIDController leftPIDController_WPI = new PIDController(Constants.kPTraj, 0, 0); //velocity controllers only have P
  private PIDController rightPIDController_WPI = new PIDController(Constants.kPTraj, 0, 0);

  private CANEncoder encoderRight;
  private CANEncoder encoderLeft;
  private CANPIDController pidcontrollerR;
  private CANPIDController pidcontrollerL;
  double tareEncPositionR = 0;
  double tareEncPositionL = 0;
  public double kTurnP = .1, kTurnI = 0, kTurnD = 0; // probs a better way to do this than make it public
  double kP = 0.075, kI = 0, kIzone = 0, kD = 1.5,kFF = 0, kMinOutput = -1, kMaxOutput = 1;

  public S_Drive(){
    
    lfmoto = new CANSparkMax(Constants.NEOlf, MotorType.kBrushless);
    lfmoto.restoreFactoryDefaults();
    lfmoto.setInverted(false);
    lfmoto.setIdleMode(IdleMode.kBrake);

    lbmoto = new CANSparkMax(Constants.NEOlb, MotorType.kBrushless);
    lbmoto.restoreFactoryDefaults();
    lbmoto.setIdleMode(IdleMode.kBrake);
    lbmoto.follow(lfmoto);

    rfmoto = new CANSparkMax(Constants.NEOrf, MotorType.kBrushless);
    rfmoto.restoreFactoryDefaults();
    rfmoto.setInverted(true);
    rfmoto.setIdleMode(IdleMode.kBrake);

    rbmoto = new CANSparkMax(Constants.NEOrb, MotorType.kBrushless);
    rbmoto.restoreFactoryDefaults();
    rbmoto.setIdleMode(IdleMode.kBrake);
    rbmoto.follow(rfmoto);

    encoderRight = rfmoto.getEncoder();
    encoderRight.setPositionConversionFactor(Constants.neoRevs2meters); //native is neo rotations
    encoderRight.setVelocityConversionFactor(Constants.neoRevs2meters/60); //native is neo rotations per minute

    encoderLeft = lfmoto.getEncoder();
    encoderLeft.setPositionConversionFactor(Constants.neoRevs2meters);
    encoderLeft.setVelocityConversionFactor(Constants.neoRevs2meters/60);

    pidcontrollerR = rfmoto.getPIDController();
    pidcontrollerR.setFeedbackDevice(encoderRight);

    pidcontrollerL = lfmoto.getPIDController();
    pidcontrollerL.setFeedbackDevice(encoderLeft);

    SmartDashboard.putNumber("P", kP);
    SmartDashboard.putNumber("I", kI);
    SmartDashboard.putNumber("Izone", kIzone);
    SmartDashboard.putNumber("D", kD);
    SmartDashboard.putNumber("FF", kFF);
    SmartDashboard.putNumber("MinOutput", kMinOutput);
    SmartDashboard.putNumber("MaxOutput", kMaxOutput);
    SmartDashboard.putNumber("Setpoint", 0);
    SmartDashboard.putNumber("turnP", kTurnP);
    SmartDashboard.putNumber("turnI", kTurnI);
    SmartDashboard.putNumber("turnD", kTurnD);
    SmartDashboard.putNumber("turnSetpoint", 0);

    pidcontrollerR.setP(kP);
    pidcontrollerL.setP(kP);
    pidcontrollerR.setI(kI);
    pidcontrollerL.setI(kI);
    pidcontrollerR.setIZone(kIzone);
    pidcontrollerL.setIZone(kIzone);
    pidcontrollerR.setD(kD);
    pidcontrollerL.setD(kD);
    pidcontrollerR.setOutputRange(kMinOutput, kMaxOutput);
    pidcontrollerL.setOutputRange(kMinOutput, kMaxOutput);

    diffDrive = new DifferentialDrive(lfmoto, rfmoto);
    diffDrive.setSafetyEnabled(false);
    diffDrive.setRightSideInverted(false); //all inversion is done beforehand
    //diffDrive.setDeadband(0.02); // default deadband for differentialDrive is 0.02
    //diffDrive.setDeadband(0); // maybe more useful to have that value for pid
    
    gyro.reset();
    resetEncPosition();
    kinematics = new DifferentialDriveKinematics(Constants.trackWidth);
    pose = new Pose2d(); // i think this should be the same as that which is used in DifferentialDriveOdometry?
    odometry = new DifferentialDriveOdometry(getHeading()); // may want other constructor for enabling a starting position
    


  }
  
  public void setPID(){
    double p = SmartDashboard.getNumber("P", 0);
    double i = SmartDashboard.getNumber("I", 0);
    double izone = SmartDashboard.getNumber("Izone", 0);
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
    if (i != kIzone){
      kIzone = izone;
      pidcontrollerR.setIZone(kIzone);
      pidcontrollerL.setIZone(kIzone);
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

    // setpoints should be in meters
    pidcontrollerR.setReference(setpoint + tareEncPositionR, ControlType.kPosition);
    pidcontrollerL.setReference(setpoint + tareEncPositionL, ControlType.kPosition); 

  }

  public void enableCoastMode(){
    lfmoto.setIdleMode(IdleMode.kCoast);
    lbmoto.setIdleMode(IdleMode.kCoast);
    rfmoto.setIdleMode(IdleMode.kCoast);
    rbmoto.setIdleMode(IdleMode.kCoast);
  }

  public void enableBrakeMode(){
    lfmoto.setIdleMode(IdleMode.kBrake);
    lbmoto.setIdleMode(IdleMode.kBrake);
    rfmoto.setIdleMode(IdleMode.kBrake);
    rbmoto.setIdleMode(IdleMode.kBrake);
  }


  public void arcadeDrive(double fwd, double rot){
    diffDrive.arcadeDrive(fwd, rot);
  }
  public void curveDrive(double fwd, double rot){
    diffDrive.curvatureDrive(fwd, rot, true);
  }

  public void resetEncPosition(){
    tareEncPositionR = encoderRight.getPosition();
    tareEncPositionL = encoderLeft.getPosition();
  }

  public void runMotor(double left, double right){
    if (left > 0.02 || left < -0.02){
      lfmoto.set(left);
      //lbmoto.set(left);
    }
    else {
      lfmoto.set(0);
    }

    if (right > 0.02 || right < -0.02){
      rfmoto.set(right);
      //rbmoto.set(right);
    }
    else {
      rfmoto.set(0);
    }

  }

  public void put_SB_Dist_PID_Info(){
    //TODO: make pid graphs
  }

    /**
   * Translates the sign of the joystick value to fit a 180 degree range of allowed positive degrees.
   * Likely won't actually have this in the competition code, I just got annoyed with the format of
   * negative values on the f310 (also coding for fun).
   * 
   * @param joystick The controller from which the axis is gotten.
   * @param axis The axis from which the axis magnitude value from [-1, 1] is gotten.
   * @param minPosDegrees The reference point on a unit circle which indicates the lower bound
   *                      of a 180 degree interval that should be marked as positive.
   *                      only actual sensible values would be 0, 90, 180, or 270 (or periodic equivalents).
   * @return the raw axis magnitude with clarified sign/direction in the dimention of the given axis.
   * */
  public double getTranslatedJoystickAxisValue(Joystick joystick, int axis, double minPosDegrees){ 
    //these axes port tranlations only work for the F310 Logitech gamepad
    if (axis%2 == 0) {
      joystick.setXChannel(axis);
      joystick.setYChannel(axis+1);
    }
    else {
      joystick.setXChannel(axis-1);
      joystick.setYChannel(axis);
    }
    double axisAngle = Math.IEEEremainder(-joystick.getDirectionDegrees()+90,360); //given angle starting from positive y-axis positive in clockwise direction
    SmartDashboard.putNumber("axisAngle", axisAngle);
    double maxPosDegrees = minPosDegrees+180.0;
    minPosDegrees = Math.IEEEremainder(minPosDegrees, 360);
    maxPosDegrees = Math.IEEEremainder(maxPosDegrees, 360);
    double output;
    //there's probs a better way to state and format this if statement
    if ((minPosDegrees < maxPosDegrees && (axisAngle < maxPosDegrees && axisAngle > minPosDegrees))
        || (minPosDegrees > maxPosDegrees && ((axisAngle > maxPosDegrees && axisAngle > minPosDegrees) 
                                              || (axisAngle < maxPosDegrees && axisAngle < minPosDegrees)))) {
      output = Math.abs(joystick.getRawAxis(axis));
    }
    else {
      output = -Math.abs(joystick.getRawAxis(axis));
    }
    
    return output;
  }


//trajecotry stuff

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  public Pose2d getPose(){
    return pose;
  }

  public SimpleMotorFeedforward getFeedforward(){
    return feedforward;
  }

  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      encoderLeft.getVelocity(),
      encoderRight.getVelocity()
    );
  }

  public PIDController getLeftTrajPIDController(){
    return leftPIDController_WPI;
  }

  public PIDController getRightTrajPIDController(){
    return rightPIDController_WPI;
  }

  public void setOutput(double leftVolts, double rightVolts){
    lfmoto.set(leftVolts/12);
    rfmoto.set(rightVolts/12);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update( // note: if we want odometry after autonomyous then would need a seperate tare for robotInit
      getHeading(),
      encoderLeft.getPosition()-tareEncPositionL,
      encoderRight.getPosition()-tareEncPositionR
    );
  }
}
