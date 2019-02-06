/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class PathWeaverTest extends Command {

  Constants constants = Constants.getInstance();

  //PATHWEAVER IMPLEMENTATION
  Trajectory leftTraj;
  Trajectory rightTraj;
  EncoderFollower leftEncFollower = new EncoderFollower(leftTraj);
  EncoderFollower rightEncFollower = new EncoderFollower(rightTraj);
  Notifier notifier;
  //SETTING CONSTANTS
  double kP = 1.0;
  double kI = 0.0;
  double kD = 0.0;
  double kV = 1 / constants.maxVel;
  double kA = 0.0;

  public PathWeaverTest(String pathName) {
    requires(Robot.drivetrain);
    //LEFT SWAPPED WITH RIGHT DUE TO A BUG IN PATHWEAVER
    leftTraj = PathfinderFRC.getTrajectory("/home/lvuser/deploy/" + pathName + ".right");
    rightTraj = PathfinderFRC.getTrajectory("/home/lvuser/deploy/" + pathName + ".left");
    notifier = new Notifier(this::followPath);
    //MAKING SURE IT GETS PAST THIS STEP
    System.out.println("Path loaded and ready to reploy.");
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    leftEncFollower = new EncoderFollower(leftTraj);
    rightEncFollower = new EncoderFollower(rightTraj);
     
    leftEncFollower.reset();
    rightEncFollower.reset();

    leftEncFollower.configureEncoder(Robot.drivetrain.leftEnc.get(), constants.ticksPerRev, constants.wheelDiameter);
    rightEncFollower.configureEncoder(Robot.drivetrain.rightEnc.get(), constants.ticksPerRev, constants.wheelDiameter);

    leftEncFollower.configurePIDVA(kP, kI, kD, kV, kA);
    rightEncFollower.configurePIDVA(kP, kI, kD, kV, kA);
    
    notifier.startPeriodic(leftTraj.get(0).dt);
    //MAKING SURE THAT IT PASSES THIS STEP
    System.out.println("Initialized and ready to go.");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //MAKING SURE IT FINISHES
    System.out.println("FINISHED!");
    return leftEncFollower.isFinished() && rightEncFollower.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    notifier.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
  public void followPath() {
    if (leftEncFollower.isFinished() || rightEncFollower.isFinished()) {
      notifier.stop();
    } else {
      double leftSpeed = leftEncFollower.calculate(Robot.drivetrain.leftEnc.get());
      double rightSpeed = rightEncFollower.calculate(Robot.drivetrain.rightEnc.get());
      double heading = Robot.drivetrain.gyro.getAngle();
      double desiredHeading = -Pathfinder.r2d(leftEncFollower.getHeading());
      double headingDifference = Pathfinder.boundHalfDegrees(desiredHeading - heading);
      double turn = 0.8 * (-1.0/80.0) * headingDifference;
      Robot.drivetrain.leftGroup.set(leftSpeed + turn);
      Robot.drivetrain.rightGroup.set(rightSpeed - turn);
  }
}
}
