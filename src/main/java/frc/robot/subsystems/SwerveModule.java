/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.HoloDrive;


public class SwerveModule extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax driveMotor;
  private TalonSRX angleMotor;
  private double encoderOffset, setpoint, lastAngle;
  private Notifier pidLoop; 
  private boolean isReversed;
  // talon constants
  private static final int TIMEOUT_MS = 10;  // set to zero if skipping confirmation
  private static final int PIDLOOP_IDX = 0;  // set to zero if primary loop
  private static final int PROFILE_SLOT = 0;

  // PIDF values - drive
  private static final double drive_kP = 5e-5;
  private static final double drive_kI = 1e-6;  
  private static final double drive_kD = 0.0;
  private static final double drive_kF = 0.0; 
  private static final int drive_kIZone = 0; //18

  // PIDF values - turn qweresd
  private static final double turn_kP = 2.0;
  private static final double turn_kI = 0.0;  
  private static final double turn_kD = 0;
  private static final int turn_kIZone = 0; //18

  
  private static final double dt = 0.05;  //this is how fast we run our PID loop.
  private static final double maxRPM = 5700;
  private int kPositiveRotationMin = 41;  //we measured this
  private int kPositiveRotationMax = 990;  //and this

  private int kNegativeRotationMin = 41;  //we measured this
  private int kNegativeRotationMax = 990;  //and this

  CANPIDController drivePid;

  private volatile double sumError, errorChange, lastError, currentError, pidOutput;

  
  
  public SwerveModule(CANSparkMax driveMotor, TalonSRX turnMotor, double encoderOffset, boolean isReversed, double kP, boolean angleReverse) {
    lastAngle = 0;
    this.driveMotor = driveMotor;
    this.angleMotor = turnMotor;
    this.encoderOffset = encoderOffset;
    drivePid = driveMotor.getPIDController();
    drivePid.setOutputRange(-1, 1);
    drivePid.setP(drive_kP);
    drivePid.setI(drive_kI);
    drivePid.setD(drive_kD);
    drivePid.setFF(drive_kF);
  
    //driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PIDLOOP_IDX, TIMEOUT_MS);
    //driveMotor.setSelectedSensorPosition(0, PIDLOOP_IDX, TIMEOUT_MS);

    angleMotor.config_kP(PROFILE_SLOT, kP, TIMEOUT_MS);
    angleMotor.config_kI(PROFILE_SLOT, turn_kI, TIMEOUT_MS);
    angleMotor.config_kD(PROFILE_SLOT, turn_kD, TIMEOUT_MS);
    angleMotor.configOpenloopRamp(0, TIMEOUT_MS);
    angleMotor.configPeakCurrentLimit(40, TIMEOUT_MS);
    angleMotor.configContinuousCurrentLimit(40, TIMEOUT_MS);
    angleMotor.configPeakCurrentDuration(100, TIMEOUT_MS);
    angleMotor.enableCurrentLimit(true);
    angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, PIDLOOP_IDX, TIMEOUT_MS);
    angleMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 10);
    angleMotor.setNeutralMode(NeutralMode.Brake);
    angleMotor.setInverted(angleReverse);

    pidLoop = new Notifier(() -> {
      currentError = getModifiedError();  //update the current error to the most recent one
      
      sumError += currentError * dt;
      errorChange = (currentError-lastError)/dt;
      
      pidOutput = kP * currentError + turn_kD * errorChange;; //+ turn_kI * sumError // 
      angleMotor.set(ControlMode.PercentOutput, pidOutput);
      //lastError = currentError;   //update the last error to be the current error
  });
  this.isReversed = isReversed;
  pidLoop.startPeriodic(dt);

  }

  
  @Override
  public void initDefaultCommand() {
    
    // setDefaultCommand(new MySpecialCommand());
  }
  public TalonSRX getAngleMotor() {
    return angleMotor;
}
public CANSparkMax getDriveMotor() {
  return driveMotor;
}

public void killMotors() {
  angleMotor.set(ControlMode.PercentOutput, 0);
  driveMotor.set(0);
}
private double normalizeEncoder(int minVal, int maxVal, int encPos){
  return ((Math.abs(encPos) % 1023) - minVal) * Math.abs((360.0/(maxVal-minVal)));
}

  public double getSetpointDegrees(){
    return setpoint - encoderOffset;
}

  public int degreesToEncUnits(double degrees){
    return (int) (degrees/360.0*4096);
  }
  
  public double encUnitsToDegrees(int encUnits){
    return encUnits/4096*360.0;
  }
  
  public double getSteeringDegrees(){
    int steeringPosition = angleMotor.getSelectedSensorPosition(0);

    if(steeringPosition >= 0){
        return normalizeEncoder(kPositiveRotationMin, kPositiveRotationMax, steeringPosition)-180;
    }
    else
        return (360-normalizeEncoder(kNegativeRotationMin, kNegativeRotationMax, steeringPosition))-180;


}


public void configEncValues(int posMin, int posMax, int negMin, int negMax){
  kPositiveRotationMin = posMin;
  kPositiveRotationMax = posMax;

  kNegativeRotationMin = negMin;
  kNegativeRotationMax = negMax;
}

public static double boundHalfDegrees(double angle_degrees) {
  while (angle_degrees >= 180.0) angle_degrees -= 360.0;
  while (angle_degrees < -180.0) angle_degrees += 360.0;
  return angle_degrees;
}

public double getActualSteeringDegrees(){
  return boundHalfDegrees(getSteeringDegrees() - encoderOffset);
}

public void setSteeringDegrees(double deg){
  setpoint = boundHalfDegrees(deg + encoderOffset);
}

public double getError(){
  return setpoint - getSteeringDegrees();
}

public double getVelocity() {
  return driveMotor.getEncoder().getVelocity();
}
public double getModifiedError(){
  return (boundHalfDegrees(getError()))/180;
}


public void setDrivePower(double power){
  if(isReversed)
  driveMotor.set(-power);
else
  driveMotor.set(power);
}

public void setVelocity(double power){
  if(isReversed)
  drivePid.setReference(-power*maxRPM, ControlType.kVelocity);
else
  drivePid.setReference(power*maxRPM, ControlType.kVelocity);
}

public void set(double degrees, double power){
  double supplement = degrees > 0 ? degrees - 180 : 180 + degrees;

  if(Math.abs(supplement-lastAngle) <= 90){
      setSteeringDegrees(supplement);
      setDrivePower(-power);
      lastAngle = supplement;
  }
  else {
      setSteeringDegrees(degrees);
      setDrivePower(power);
      lastAngle = degrees;
  }
}

public void setVelocity(double degrees, double power){
  double supplement = degrees > 0 ? degrees - 180 : 180 + degrees;

  if(Math.abs(supplement-lastAngle) <= 90){
      setSteeringDegrees(supplement);
      setVelocity(-power);
      lastAngle = supplement;
  }
  else {
      setSteeringDegrees(degrees);
      setVelocity(power);
      lastAngle = degrees;
  }
}


}