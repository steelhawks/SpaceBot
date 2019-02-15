/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ArmPivot;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class AutoPivot extends Command {

  Constants constants = Constants.getInstance();
  public double setPos;
  public boolean setDir;

  public AutoPivot(double pos, boolean dir) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.pivot);
    setPos = pos;
    setDir = dir;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.pivot.pivotM.getSensorCollection().setQuadraturePosition(0, 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.pivot.autoPivot(setPos, setDir);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return constants.autoArm;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.pivot.stopPivot();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
