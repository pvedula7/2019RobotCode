/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class HoloDrive extends Command {
  public HoloDrive() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivetrain);
    Robot.drivetrain.navx.zeroYaw();
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  private double deadband(double input) {
		if (Math.abs(input) < 0.05) return 0;
		return input;
	}
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double forward = -Robot.getOI().getPrimaryController().getLeftYValue();
		double strafe = Robot.getOI().getPrimaryController().getLeftXValue();
    double rotation = -Robot.getOI().getPrimaryController().getRightXValue();
    
    forward*=   Math.abs(forward);
		strafe *=   Math.abs(strafe);
    rotation *=  Math.abs(rotation);

    forward = deadband(forward);
		strafe = deadband(strafe);
		rotation = deadband(rotation);
    
    SmartDashboard.putNumber("Forward", forward);
		SmartDashboard.putNumber("Strafe", strafe);
    SmartDashboard.putNumber("Rotation", rotation);
    
    Robot.drivetrain.holonomicDrive(forward, strafe, rotation);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.stopMotors();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
