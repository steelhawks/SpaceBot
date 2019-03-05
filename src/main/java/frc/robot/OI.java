/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.Arms.ArmIntakeButton;
import frc.robot.commands.Arms.ArmOuttakeButton;
import frc.robot.commands.Arms.ArmPistonButton;
import frc.robot.commands.Arms.ArmStopButton;
import frc.robot.commands.Arms.HatchPistonButton;
import frc.robot.commands.Climber.ClimberExtend;
import frc.robot.commands.Climber.DropdownMotor;
import frc.robot.commands.Climber.FrontRetract;
import frc.robot.commands.Climber.RearRetract;
import frc.robot.commands.Climber.StopClimber;
import frc.robot.commands.Climber.StopDropdown;
import frc.robot.commands.Drivetrain.ShiftGear;

/** 
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  
  Constants constants = Constants.getInstance();

  public Joystick driveJS = new Joystick(constants.driveJSPort);
  public Gamepad gamepad = new Gamepad(constants.gamepadPort);
  public Gamepad climberpad = new Gamepad(constants.climberpadPort);

  public OI(){
    //DRIVER JOYSTICK BUTTONS
    Button shift = new JoystickButton(driveJS, constants.shiftB);
    shift.whenPressed(new ShiftGear());

    //OPERATOR GAMEPAD BUTTONS
    Button armPiston = new JoystickButton(gamepad, constants.armPistonB);
    armPiston.whenPressed(new ArmPistonButton());

    Button hatchPiston = new JoystickButton(gamepad, constants.hatchPistonB);
    hatchPiston.whenPressed(new HatchPistonButton());

    Button armIntake = new JoystickButton(gamepad, constants.armIntakeB);
    armIntake.whenActive(new ArmIntakeButton());
    armIntake.whenInactive(new ArmStopButton());

    Button armOuttake = new JoystickButton(gamepad, constants.armOuttakeB);
    armOuttake.whenActive(new ArmOuttakeButton());
    armOuttake.whenInactive(new ArmStopButton());

    //CLIMBERPAD BUTTONS
    Button dropdownButton = new JoystickButton(climberpad, constants.dropdownB);
    dropdownButton.whenActive(new DropdownMotor());
    dropdownButton.whenInactive(new StopDropdown());

    /*Button allActuatorButton = new JoystickButton(climberpad, constants.allExtendB);
    allActuatorButton.whenActive(new ClimberExtend());
    allActuatorButton.whenInactive(new StopClimber());
    */
    Button frontRetractButton = new JoystickButton(climberpad, constants.frontRetractB);
    frontRetractButton.whenPressed(new FrontRetract());
    
    /*Button rearRetractButton = new JoystickButton(climberpad, constants.rearRetractB);
    allActuatorButton.whenActive(new RearRetract());
    allActuatorButton.whenInactive(new StopClimber());*/
  }
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
