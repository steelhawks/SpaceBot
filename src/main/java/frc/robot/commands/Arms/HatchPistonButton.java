/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Arms;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class HatchPistonButton extends Command {

  public HatchPistonButton() {
    requires(Robot.arms);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.arms.hatchPiston();
  }

  @Override
  protected boolean isFinished() {
    return true;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
