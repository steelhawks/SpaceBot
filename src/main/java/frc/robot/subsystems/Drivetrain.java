/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.commands.Drivetrain.DiffDrive;

public class Drivetrain extends Subsystem {

  static Constants constants = Constants.getInstance();

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
  public CANEncoder leftNeoEnc = frontLeftM.getEncoder();
  public CANEncoder rightNeoEnc = frontRightM.getEncoder();

  //GRAYHILL OPTICAL ENCODERS
  public Encoder leftEnc = new Encoder(constants.leftEncPortA, constants.leftEncPortB, false, EncodingType.k4X);
  public Encoder rightEnc = new Encoder(constants.rightEncPortA, constants.rightEncPortB, false, EncodingType.k4X);

  //DRIVETRAIN CONSTRUCTOR
  public Drivetrain() {
    resetGyro();
    leftEnc.reset();
    rightEnc.reset();
    shiftSol.set(DoubleSolenoid.Value.kForward);
    frontLeftM.setIdleMode(IdleMode.kCoast);
    midLeftM.setIdleMode(IdleMode.kCoast);
    rearLeftM.setIdleMode(IdleMode.kBrake);
    frontRightM.setIdleMode(IdleMode.kCoast);
    midRightM.setIdleMode(IdleMode.kCoast);
    rearRightM.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DiffDrive());
  }

  //DRIVING METHOD
  public void arcadeDrive(Joystick stick) {
    double y = stick.getY();
    double rotate = stick.getTwist();
    diffDrive.arcadeDrive(y, -rotate);
  }

  //SHIFTING METHOD
  public void shiftGearButton() {
    if(shiftSol.get() == DoubleSolenoid.Value.kForward) {
      shiftSol.set(DoubleSolenoid.Value.kReverse);
    } else {
      shiftSol.set(DoubleSolenoid.Value.kForward);
    }
  }

  public double getLeftEncRate() {
    return leftEnc.getRate();
  }

  public double getLeftEncDist() {
    return leftEnc.getDistance();
  }

  public double getRightEncRate() {
    return rightEnc.getRate();
  }

  public double getRightEncDist() {
    return rightEnc.getDistance();
  }

  public double getGyroAngle() {
    return gyro.getAngle(); 
  }

  public double getGyroAxis() {
    return gyro.getBoardYawAxis().board_axis.getValue();
  }

  public static void resetGyro() {
    gyro.reset();
    gyro.zeroYaw();
  }
}
