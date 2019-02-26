package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.LiftDrive;
import frc.robot.commands.SetLift;
import frc.robot.commands.YeetHatch;

public class Hatch extends Subsystem {

    public DoubleSolenoid extendor = new DoubleSolenoid(2, 3);
    public DoubleSolenoid pivot = new DoubleSolenoid(0, 1);
    private DigitalInput limitSwitch = new DigitalInput(1);

   
    public Hatch() {
      extendor.set(Value.kForward);
      //pivot.set(Value.kForward);
    }
 
    @Override
    protected void initDefaultCommand() {
        //setDefaultCommand(new YeetHatch());
    }

    public void update() {
        if(limitSwitch.get()) {
            yeet();
        }
    }
 
    public void yeet () {
        if(extendor.get()==Value.kForward) {
            extendor.set(Value.kReverse);
        }
        else   
            extendor.set(Value.kForward);
    }

    public void pivot () {
        if(pivot.get()==Value.kForward) {
            pivot.set(Value.kReverse);
        }
        else   
            pivot.set(Value.kForward);
    }
  
 


}