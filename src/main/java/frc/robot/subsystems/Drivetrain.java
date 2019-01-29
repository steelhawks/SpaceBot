/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class Drivetrain extends Subsystem {

  static Constants constants = Constants.getInstance();
  
  /*TALON CODE THAT IS UNUSED BECAUSE WE ARE USING SPARKS
  //TALON SRX LEFT MOTORS
  public WPI_TalonSRX frontLeftM = new WPI_TalonSRX(constants.frontLeftMPort);
  public WPI_TalonSRX midLeftM = new WPI_TalonSRX(constants.midLeftMPort);
  public WPI_TalonSRX rearLeftM = new WPI_TalonSRX(constants.rearLeftMPort);
  //TALON SRX RIGHT MOTORS
  public WPI_TalonSRX frontRightM = new WPI_TalonSRX(constants.frontRightMPort);
  public WPI_TalonSRX midRightM = new WPI_TalonSRX(constants.midRightMPort);
  public WPI_TalonSRX rearRightM = new WPI_TalonSRX(constants.rearRightMPort);*/

  //SPARK MAX LEFT MOTORS
  public CANSparkMax frontLeftM = new CANSparkMax(constants.frontLeftMPort, MotorType.kBrushless);
  public CANSparkMax midLeftM = new CANSparkMax(constants.midLeftMPort, MotorType.kBrushless);
  public CANSparkMax rearLeftM = new CANSparkMax(constants.rearLeftMPort, MotorType.kBrushless);
  
  //SPARK MAX RIGHT MOTORS
  public CANSparkMax frontRightM = new CANSparkMax(constants.frontRightMPort, MotorType.kBrushless);
  public CANSparkMax midRightM = new CANSparkMax(constants.midRightMPort, MotorType.kBrushless);
  public CANSparkMax rearRightM = new CANSparkMax(constants.rearRightMPort, MotorType.kBrushless);

  //SPEED CONTROLLER GROUPS
  public SpeedControllerGroup leftGroup = new SpeedControllerGroup(frontLeftM, midLeftM, rearLeftM);
  public SpeedControllerGroup rightGroup = new SpeedControllerGroup(frontRightM, midRightM, rearRightM);

  //DIFFERENTIAL DRIVE
  public DifferentialDrive diffDrive = new DifferentialDrive(leftGroup, rightGroup);

  //SHIFTING SOLENOIDS
  public DoubleSolenoid shiftSol = new DoubleSolenoid(constants.shiftSolPortOn, constants.shiftSolPortOff);

  //NAVX MXP GYRO
  public static AHRS gyro = new AHRS(SPI.Port.kMXP);

  //NEO MOTOR ENCODERS
  public CANEncoder leftSideEnc = frontLeftM.getEncoder();
  public CANEncoder rightSideEnc = frontRightM.getEncoder();

  public Drivetrain(){
    //SETTING DEFAULTS FOR THE ROBOT
    resetGyro();
    //OUTPUTTING LEFT SIDE DRIVETRAIN ENCODER VALUES
    SmartDashboard.putNumber("Left NEO Encoder Position", getLeftEncPos());
    SmartDashboard.putNumber("Left NEO Encoder Velocity", getLeftEncVel());
    //OUTPUTTING RIGHT SIDE DRIVETRAIN ENCODER VALUES
    SmartDashboard.putNumber("Right NEO Encoder Position", getRightEncPos());
    SmartDashboard.putNumber("Right NEO Encoder Velocity", getRightEncVel());
    //OUTPUTTING LEFT SIDE DRIVETRAIN VOLTAGE VALUES
    SmartDashboard.putNumber("Front Left Motor Voltage", frontLeftM.getBusVoltage());
    SmartDashboard.putNumber("Mid Left Motor Voltage", midLeftM.getBusVoltage());
    SmartDashboard.putNumber("Rear Left Motor Voltage", rearLeftM.getBusVoltage());
    //OUTPUTTING RIGHT SIDE DRIVETRAIN VOLTAGE VALUES
    SmartDashboard.putNumber("Front Right Motor Voltage", frontRightM.getBusVoltage());
    SmartDashboard.putNumber("Mid Right Motor Voltage", midRightM.getBusVoltage());
    SmartDashboard.putNumber("Rear Right Motor Voltage", rearRightM.getBusVoltage());
    //OUTPUTTING LEFT SIDE DRIVETRAIN TEMPERATURE VALUES
    SmartDashboard.putNumber("Front Left Motor Temperature", frontLeftM.getMotorTemperature());
    SmartDashboard.putNumber("Mid Left Motor Temperature", midLeftM.getMotorTemperature());
    SmartDashboard.putNumber("Rear Left Motor Temperature", rearLeftM.getMotorTemperature());
    //OUTPUTTING RIGHT SIDE DRIVETRAIN TEMPERATURE VALUES
    SmartDashboard.putNumber("Front Right Motor Temperature", frontRightM.getMotorTemperature());
    SmartDashboard.putNumber("Mid Right Motor Temperature", midRightM.getMotorTemperature());
    SmartDashboard.putNumber("Rear Right Motor Temperature", rearRightM.getMotorTemperature());
    //OUTPUTTING LEFT SIDE DRIVETRAIN OUTPUT VALUES
    SmartDashboard.putNumber("Front Left Motor Output", frontLeftM.getAppliedOutput());
    SmartDashboard.putNumber("Mid Left Motor Output", midLeftM.getAppliedOutput());
    SmartDashboard.putNumber("Rear Left Motor Output", rearLeftM.getAppliedOutput());
    //OUTPUTTING RIGHT SIDE DRIVETRAIN OUTPUT VALUES
    SmartDashboard.putNumber("Front Right Motor Output", frontRightM.getAppliedOutput());
    SmartDashboard.putNumber("Mid Right Motor Output", midRightM.getAppliedOutput());
    SmartDashboard.putNumber("Rear Right Motor Output", rearRightM.getAppliedOutput());
    //OUTPUTTING GYRO VALUES
    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
    SmartDashboard.putNumber("Gyro Axis", getGyroAxis());
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  //DRIVING METHOD
  public void arcadeDrive(Joystick stick) {
    double y = stick.getY();
    double rotate = stick.getTwist();
    diffDrive.arcadeDrive(y, rotate);
  }

  //SHIFTING METHOD
  public void shiftGears(){
    if(shiftSol.get() == DoubleSolenoid.Value.kForward) {
      shiftSol.set(DoubleSolenoid.Value.kReverse);
    } else {
      shiftSol.set(DoubleSolenoid.Value.kForward);
    }
  }

  //PATH METHODS
  public void path(String Path1){
    //LEFT SWAPPED WITH RIGHT DUE TO A BUG IN PATHWEAVER
    Trajectory leftTraj = PathfinderFRC.getTrajectory(Path1 + ".right");
    Trajectory rightTraj = PathfinderFRC.getTrajectory(Path1 + ".left");

    EncoderFollower leftEncFollower = new EncoderFollower(leftTraj);
    EncoderFollower rightEncFollower = new EncoderFollower(rightTraj);
  }

  public double getLeftEncPos(){
    return leftSideEnc.getPosition();
  }
  public double getLeftEncVel(){
    return leftSideEnc.getVelocity();
  }
  public double getRightEncPos(){
    return rightSideEnc.getPosition();
  }
  public double getRightEncVel(){
    return rightSideEnc.getVelocity();
  }
  public double getGyroAngle(){
    return gyro.getAngle();
  }
  public double getGyroAxis(){
    return gyro.getBoardYawAxis().board_axis.getValue();
  }
  public static void resetGyro(){
    gyro.reset();
    gyro.zeroYaw();
  }
}
