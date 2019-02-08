/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutonCommands.PathWeaverTest;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class Robot extends TimedRobot {

  Constants constants = Constants.getInstance();

  public static final Drivetrain drivetrain = new Drivetrain();
  public static final Arms arms = new Arms();
  public static final Climber climber = new Climber();
  public static final Elevator elevator = new Elevator();
  public static OI oi = new OI();

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    Robot.drivetrain.diffDrive.setSafetyEnabled(false);
    Robot.drivetrain.diffDrive.setExpiration(120);
    SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
        //OUTPUTTING LEFT SIDE DRIVETRAIN ENCODER VALUES
        SmartDashboard.putNumber("Left Enc Rate", Robot.drivetrain.getLeftEncRate());
        SmartDashboard.putNumber("Left Enc Distance", Robot.drivetrain.getLeftEncDist());
        //OUTPUTTING RIGHT SIDE DRIVETRAIN ENCODER VALUES
        SmartDashboard.putNumber("Right Enc Rate", Robot.drivetrain.getRightEncRate());
        SmartDashboard.putNumber("Right Enc Distance", Robot.drivetrain.getRightEncDist());
        //OUTPUTTING LEFT SIDE DRIVETRAIN VOLTAGE VALUES
        SmartDashboard.putNumber("Front Left Motor Voltage", Robot.drivetrain.frontLeftM.getBusVoltage());
        SmartDashboard.putNumber("Mid Left Motor Voltage", Robot.drivetrain.midLeftM.getBusVoltage());
        SmartDashboard.putNumber("Rear Left Motor Voltage", Robot.drivetrain.rearLeftM.getBusVoltage());
        //OUTPUTTING RIGHT SIDE DRIVETRAIN VOLTAGE VALUES
        SmartDashboard.putNumber("Front Right Motor Voltage", Robot.drivetrain.frontRightM.getBusVoltage());
        SmartDashboard.putNumber("Mid Right Motor Voltage", Robot.drivetrain.midRightM.getBusVoltage());
        SmartDashboard.putNumber("Rear Right Motor Voltage", Robot.drivetrain.rearRightM.getBusVoltage());
        //OUTPUTTING LEFT SIDE DRIVETRAIN OUTPUT VALUES
        SmartDashboard.putNumber("Front Left Motor Output", Robot.drivetrain.frontLeftM.getAppliedOutput());
        SmartDashboard.putNumber("Mid Left Motor Output", Robot.drivetrain.midLeftM.getAppliedOutput());
        SmartDashboard.putNumber("Rear Left Motor Output", Robot.drivetrain.rearLeftM.getAppliedOutput());
        //OUTPUTTING RIGHT SIDE DRIVETRAIN OUTPUT VALUES
        SmartDashboard.putNumber("Front Right Motor Output", Robot.drivetrain.frontRightM.getAppliedOutput());
        SmartDashboard.putNumber("Mid Right Motor Output", Robot.drivetrain.midRightM.getAppliedOutput());
        SmartDashboard.putNumber("Rear Right Motor Output", Robot.drivetrain.rearRightM.getAppliedOutput());
        //OUTPUTTING GYRO VALUES
        SmartDashboard.putNumber("Gyro Angle", Robot.drivetrain.getGyroAngle());
        SmartDashboard.putNumber("Gyro Axis", Robot.drivetrain.getGyroAxis());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = new PathWeaverTest("TestPath");

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
