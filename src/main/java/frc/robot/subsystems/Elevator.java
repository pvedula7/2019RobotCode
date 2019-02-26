package frc.robot.subsystems;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.LiftDrive;
import frc.robot.commands.SetLift;
import frc.robot.commands.SetLiftMotion;

public class Elevator extends Subsystem {

    public CANSparkMax elevator = new CANSparkMax(RobotMap.elevator,MotorType.kBrushless);
    public CANPIDController liftPid = new CANPIDController(elevator);
    public CANEncoder liftEnc = elevator.getEncoder();
    private double maxVel, minVel, maxAccel, minAccel;
    private DigitalInput liftSwitch = new DigitalInput(0);

    public Elevator(){
        maxVel = 800;
        maxAccel = 400;
        minVel=100;
        elevator.setInverted(false);
        elevator.setOpenLoopRampRate(0.2);
        this.liftPid.setOutputRange(-1, 1);
        this.liftPid.setP(5e-5);
        this.liftPid.setI(1e-4);
        this.liftPid.setD(1.0);
        this.liftPid.setFF(0);
        this.liftPid.setSmartMotionMaxVelocity(maxVel, 0);
        this.liftPid.setSmartMotionMaxVelocity(minVel, 0);
        this.liftPid.setSmartMotionMaxAccel(maxAccel,0); 
        this.liftPid.setSmartMotionAllowedClosedLoopError(1, 0);
        liftEnc.setPosition(0);

    }
 
    @Override
    protected void initDefaultCommand() {
        //setDefaultCommand(new SetLift(-10));
         setDefaultCommand(new LiftDrive());
    }

 
    public void set(double speed) {
        elevator.set(speed);
    }

    public void setPosition(double pos) {
        liftPid.setReference(pos, ControlType.kPosition);
    }
    public void setMotionMagic(double pos) {
        liftPid.setReference(pos, ControlType.kSmartMotion);
    }

    public void updateElevator () {
        SmartDashboard.putNumber("Lift encoder readings", liftEnc.getPosition());
        SmartDashboard.putNumber("Lift velocity", liftEnc.getVelocity());
        if(liftSwitch.get()) {
            liftEnc.setPosition(0);
        }

    }
  
  
 
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


}