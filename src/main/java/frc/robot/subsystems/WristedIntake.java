package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.JoystickIntake;


public class WristedIntake extends Subsystem {

    VictorSPX intake = new VictorSPX(5);
    
    //TalonSRX wrist = new TalonSRX(8);
   
    public WristedIntake() {
     
    }
    
    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new JoystickIntake());
    }

    public void set(double pow) {
        intake.set(ControlMode.PercentOutput, pow);
    }
    public void wrist(double pow) {
        //wrist.set(ControlMode.PercentOutput, pow);
    }
 
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


}