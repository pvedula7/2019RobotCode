/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.HoloDrive;
import frc.robot.commands.RotateSwerveModule;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.SwerveModifier;


public class Drivetrain extends Subsystem {

    public static final double WHEELBASE = 20;  
    public static final double TRACKWIDTH = 22.5; 
    public boolean fieldOriented=false;
    public double speedMultiplier = 0.5;
    public double rotMultiplier = 0.5;

    public static final double WIDTH = 29.5;  
    public static final double LENGTH = 29.5; 
    private Trajectory trajectory;
    private SwerveModifier modifier;
    private Trajectory fl;
    private Trajectory fr;
    private Trajectory bl;
    private Trajectory br;
    private SwerveModifier.Mode pathMode = SwerveModifier.Mode.SWERVE_DEFAULT;
    private EncoderFollower flFollower = new EncoderFollower();
    private EncoderFollower frFollower = new EncoderFollower();
    private EncoderFollower blFollower = new EncoderFollower();
    private EncoderFollower brFollower = new EncoderFollower();
    CANSparkMax flDriveMotor = new CANSparkMax(RobotMap.frontLeftDriveMotor,MotorType.kBrushless);
    TalonSRX flAngleMotor = new TalonSRX(RobotMap.frontLeftAngleMotor);
    CANSparkMax frDriveMotor = new CANSparkMax(RobotMap.frontRightDriveMotor, MotorType.kBrushless);
    TalonSRX frAngleMotor = new TalonSRX(RobotMap.frontRightAngleMotor);
    CANSparkMax blDriveMotor = new CANSparkMax(RobotMap.backLeftDriveMotor, MotorType.kBrushless);
    TalonSRX blAngleMotor = new TalonSRX(RobotMap.backLefttAngleMotor);
    CANSparkMax brDriveMotor = new CANSparkMax(RobotMap.backRightDriveMotor, MotorType.kBrushless);
    TalonSRX brAngleMotor = new TalonSRX(RobotMap.backRightAngleMotor);
   

    public SwerveModule frontLeft = new SwerveModule(flDriveMotor, flAngleMotor, -138, true, 3.5, true); //true
    public SwerveModule frontRight = new SwerveModule(frDriveMotor, frAngleMotor, -149, true,1.5,true); // true
    public SwerveModule backLeft = new SwerveModule(blDriveMotor, blAngleMotor, -69.6, false, 2.0, false);
    public SwerveModule backRight = new SwerveModule(brDriveMotor, brAngleMotor, -162.3, true, 1.1, false);
    public AHRS navx = new AHRS(SPI.Port.kMXP);
    @Override
    protected void initDefaultCommand() {
      setDefaultCommand(new HoloDrive());
    }
    public Drivetrain(){
      frontLeft.configEncValues(35, 987, 35, 987);
      backRight.configEncValues(43, 987, 43, 987); 
      backLeft.configEncValues(54, 990, 54, 990); 
      frontRight.configEncValues(41, 985, 41, 985);
      frontLeft.getAngleMotor().setSensorPhase(false); 
      backRight.getAngleMotor().setSensorPhase(true);
      backLeft.getAngleMotor().setSensorPhase(true);
      frontRight.getAngleMotor().setSensorPhase(false); 
      
    }
 
    public double[] calculateSwerveModuleAngles(double forward, double strafe, double rotation, boolean isFieldOriented) {
      if (isFieldOriented) {
          double heading = Math.toRadians(navx.getAngle());
          double temp = forward * Math.cos(heading) + strafe * Math.sin(heading);
          strafe = -forward * Math.sin(heading) + strafe * Math.cos(heading);
          forward = temp;
      }

      double a = strafe - rotation * (WHEELBASE / TRACKWIDTH);
      double b = strafe + rotation * (WHEELBASE / TRACKWIDTH);
      double c = forward - rotation * (TRACKWIDTH / WHEELBASE);
      double d = forward + rotation * (TRACKWIDTH / WHEELBASE);

      return new double[]{
              Math.atan2(b, c) * 180 / Math.PI,
              Math.atan2(b, d) * 180 / Math.PI,
              Math.atan2(a, d) * 180 / Math.PI,
              Math.atan2(a, c) * 180 / Math.PI,
              
      };
  }

  
  public void holonomicDrive(double forward, double strafe, double rotation) {
      forward *= speedMultiplier;
      strafe *= speedMultiplier;
      rotation *= rotMultiplier;
      if (fieldOriented) {
          double heading = Math.toRadians(getAngle());
          double temp = forward * Math.cos(heading) + strafe * Math.sin(heading);
          strafe = -forward * Math.sin(heading) + strafe * Math.cos(heading);
          forward = temp;
      }

      double a = strafe - rotation * (WHEELBASE / TRACKWIDTH);
      double b = strafe + rotation * (WHEELBASE / TRACKWIDTH);
      double c = forward - rotation * (TRACKWIDTH / WHEELBASE);
      double d = forward + rotation * (TRACKWIDTH / WHEELBASE);

     
      double frontRightAng = Math.atan2(b, c) * 180 / Math.PI;
      double frontLeftAng = Math.atan2(b, d) * 180 / Math.PI;
      double backLeftAng = Math.atan2(a, d) * 180 / Math.PI;
      double backRightAng = Math.atan2(a, c) * 180 / Math.PI;

      double frontRightSpeed = Math.sqrt(b * b + c * c);
      double frontLeftSpeed = Math.sqrt(b * b + d * d);
      double backLeftSpeed = Math.sqrt(a * a + d * d);
      double backRightSpeed = Math.sqrt(a * a + c * c);

      double max = backLeftSpeed;

      if(frontRightSpeed > max)
        max = backLeftSpeed;
      else if(frontLeftSpeed > max)
        max = backLeftSpeed;
      else if(backRightSpeed > max)
        max = backRightSpeed;
      if(max > 1){
        frontLeftSpeed /= max;
        frontRightSpeed /= max;
        backLeftSpeed /= max;
        backRightSpeed /= max;
      }

      /*
      backLeft.set(frontLeftAng, frontLeftSpeed);
      backRight.set(frontRightAng, frontRightSpeed);
      frontLeft.set(backLeftAng, backLeftSpeed);
      frontRight.set(backRightAng, backRightSpeed);
      */
            
      backLeft.setVelocity(frontLeftAng, frontLeftSpeed);
      backRight.setVelocity(frontRightAng, frontRightSpeed);
      frontLeft.setVelocity(backLeftAng, backLeftSpeed);
      frontRight.setVelocity(backRightAng, backRightSpeed);
    }

    public double getAngle() {
      double angle = navx.getAngle();
      angle %= 360;
      if (angle<0) angle+=360;
      return angle;
    }
    public void stopMotors() {
      frontLeft.killMotors();
      frontRight.killMotors();
      backLeft.killMotors();
      backRight.killMotors();
    }

    public void turnModule (double angle) {
      frontLeft.set(angle,0);
      backLeft.set(angle,0);
      backRight.set(angle,0);
      frontRight.set(angle,0);
      
    }

    public void lockTheseMofos() {
      frontLeft.set(-45,0);
      backLeft.set(45,0);

      frontRight.set(45,0);
      backRight.set(-45,0);

    }

    public void zeroModules() {
      frontLeft.zeroEncoder();
      frontRight.zeroEncoder();
      backLeft.zeroEncoder();
      backRight.zeroEncoder();
    }
    public void toggleFieldOriented() {
      if(fieldOriented)
        fieldOriented=false;
      else
        fieldOriented=true;
    }


    public void setSpeedMultiplier(double speedMultiplier, double rotMultiplier) {
      this.speedMultiplier = speedMultiplier;
      this.rotMultiplier = rotMultiplier;

    }

    public void followPath(Waypoint[] path, boolean isReversed) {
      Trajectory.Config config = new Trajectory.Config(
            Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 12, 4, 30
        );
        trajectory = Pathfinder.generate(path, config);
        modifier.modify(WHEELBASE, TRACKWIDTH, pathMode);
        fl = modifier.getFrontLeftTrajectory();     
        fr = modifier.getFrontRightTrajectory();      
        bl = modifier.getBackLeftTrajectory();      
        br = modifier.getBackRightTrajectory();       
        flFollower.setTrajectory(fl);
        frFollower.setTrajectory(fr);
        blFollower.setTrajectory(bl);
        brFollower.setTrajectory(br);

        flFollower.configureEncoder((int)flDriveMotor.getEncoder().getPosition(), 8, 48);
        frFollower.configureEncoder((int)frDriveMotor.getEncoder().getPosition(), 8, 48);
        blFollower.configureEncoder((int)blDriveMotor.getEncoder().getPosition(), 8, 48);
        brFollower.configureEncoder((int)brDriveMotor.getEncoder().getPosition(), 8, 48);

        double flOutput = flFollower.calculate((int)flDriveMotor.getEncoder().getPosition());
        double flHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(flFollower.getHeading()));

        double frOutput = frFollower.calculate((int)frDriveMotor.getEncoder().getPosition());
        double frHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(frFollower.getHeading()));

        double blOutput = blFollower.calculate((int)blDriveMotor.getEncoder().getPosition());
        double blHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(blFollower.getHeading()));

        double brOutput = brFollower.calculate((int)brDriveMotor.getEncoder().getPosition());
        double brHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(brFollower.getHeading()));

        frontLeft.set(flHeading, flOutput);
        frontRight.set(frHeading, frOutput);
        backLeft.set(blHeading, blOutput);
        backRight.set(brHeading, brOutput);
        


    }
    

    public void generatePaths() {
      Waypoint[] baseline = new Waypoint[]{
				new Waypoint(0,0,0),
				new Waypoint(5.0, -1.85, Pathfinder.d2r(20)),
      };
      
      
    }



    public void updateDashboard() {
      SmartDashboard.putNumber("Raw Analog Encoder Value", frontLeft.getAngleMotor().getSelectedSensorPosition(0) % 1023);
      SmartDashboard.putNumber("Normalized Steering Angle", frontLeft.getSteeringDegrees());
      SmartDashboard.putNumber("Actual Steering Angle", frontLeft.getActualSteeringDegrees());

      SmartDashboard.putNumber("back Right Enc", backRight.getAngleMotor().getSelectedSensorPosition(0) % 1023);
      SmartDashboard.putNumber("back right Angle", backRight.getSteeringDegrees());
      SmartDashboard.putNumber("actual back right Angle", backRight.getActualSteeringDegrees());

      
      SmartDashboard.putNumber("Front Right Enc", frontRight.getAngleMotor().getSelectedSensorPosition(0) % 1023);
      SmartDashboard.putNumber("Front Right normalize", frontRight.getSteeringDegrees());
      SmartDashboard.putNumber("FR Actual", frontRight.getActualSteeringDegrees());
      

      SmartDashboard.putNumber("Back Left Enc", backLeft.getAngleMotor().getSelectedSensorPosition(0) % 1023);
      SmartDashboard.putNumber("Back Left normalize", backLeft.getSteeringDegrees());
      SmartDashboard.putNumber("BL Actual", backLeft.getActualSteeringDegrees());

      SmartDashboard.putNumber("NavX", navx.getAngle());

      SmartDashboard.putNumber("BackLeft Setpoint", backLeft.getSetpointDegrees());
      SmartDashboard.putNumber("FrontLeft Setpoint", frontLeft.getSetpointDegrees());

      SmartDashboard.putBoolean("isFieldOriented", fieldOriented);

      SmartDashboard.putNumber("Drive Velocity", frontLeft.getVelocity());


    } 
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


}