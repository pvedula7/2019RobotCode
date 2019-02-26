/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Vision extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public Solenoid ledRing = new Solenoid(7);

  public Vision() {

    ledRing.set(true);
    setMode(1);
    
  }
  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("ProtocolVision");
  public static NetworkTableEntry tapeDetected = table.getEntry("tapeDetected");
  public static NetworkTableEntry cargoDetected = table.getEntry("cargoDetected");
  public static NetworkTableEntry tapeYawEntry = table.getEntry("tapeYaw");
  public static NetworkTableEntry cargoYawEntry = table.getEntry("cargoYaw");

  public static boolean tapeInView;
  public static boolean cargoInView;
  public static double tapeYaw;
  public static double cargoYaw;
// 0 -> Driver 1 -> Tape 2-> Ball
  public void setMode(int mode) {
    if(mode==1) {
        NetworkTableInstance.getDefault().getTable("Protocol Vision").getEntry("Driver").setBoolean(true);
        NetworkTableInstance.getDefault().getTable("Protocol Vision").getEntry("Driver").setBoolean(false);
    }
    else if (mode==2) {
        NetworkTableInstance.getDefault().getTable("Protocol Vision").getEntry("Driver").setBoolean(false);
        NetworkTableInstance.getDefault().getTable("Protocol Vision").getEntry("Driver").setBoolean(true);
    }
    else if (mode==3) {
        NetworkTableInstance.getDefault().getTable("Protocol Vision").getEntry("Driver").setBoolean(false);
        NetworkTableInstance.getDefault().getTable("Protocol Vision").getEntry("Driver").setBoolean(false);

    }
    else
      mode = 2;
  }

  public void updateVisionTelemtry() {
    SmartDashboard.putNumber("tapeYaw", tapeYaw);
    SmartDashboard.putNumber("cargoYaw", cargoYaw);
  }

  public void tapeAlign() {
    tapeYaw = tapeYawEntry.getDouble(0);
    tapeInView = tapeDetected.getBoolean(false);
    
  }

  public void cargoAlign() {
    cargoYaw = cargoYawEntry.getDouble(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}