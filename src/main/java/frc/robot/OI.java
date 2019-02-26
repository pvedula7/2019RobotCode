/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import frc.robot.commands.YeetHatch;
import frc.robot.commands.ZeroEncoderModules;
import frc.robot.util.IGamepad;
import frc.robot.util.XboxGamepad;
import frc.robot.util.DPadButton.Direction;
import frc.robot.commands.SetLift;
import frc.robot.commands.SetSpeedMultiplier;
import frc.robot.commands.SetWristPower;
import frc.robot.commands.ToggleFieldOriented;
import frc.robot.commands.LockModules;
import frc.robot.commands.PivotHatch;
import frc.robot.commands.RotateSwerveModule;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 * 
 * 
 */
public class OI {

  private IGamepad primaryController = new XboxGamepad(2);
  private IGamepad secondaryController = new XboxGamepad(3);

  
  public IGamepad getPrimaryController() {
    return primaryController;
}

public IGamepad getSecondaryController() {
  return secondaryController;
}


public OI() {
  secondaryController.getBButton().whenPressed(new YeetHatch());
  secondaryController.getXButton().whenPressed(new PivotHatch());
  secondaryController.getDPadButton(Direction.DOWN).whileHeld(new SetWristPower(1));
  secondaryController.getDPadButton(Direction.UP).whileHeld(new SetWristPower(-1));
  //secondaryController.getYButton().whenPressed(new SetLift(-10));

  primaryController.getLeftBumperButton().whenPressed(new ToggleFieldOriented());
  primaryController.getRightBumperButton().whenPressed(new SetSpeedMultiplier(0.1, 0.1));
  primaryController.getAButton().whenPressed(new ZeroEncoderModules());
  primaryController.getXButton().whenPressed(new LockModules());

}

}
