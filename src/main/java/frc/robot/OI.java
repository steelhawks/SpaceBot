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
import frc.robot.commands.Pivot.PivotDownButton;
import frc.robot.commands.Pivot.PivotStopButton;
import frc.robot.commands.Pivot.PivotUpButton;

/** 
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  
  Constants constants = Constants.getInstance();

  public Joystick driveJS = new Joystick(constants.driveJSPort);
  public Gamepad gamepad = new Gamepad(constants.gamepadPort);

  public OI(){
    //DRIVER JOYSTICK BUTTONS
    Button shift = new JoystickButton(driveJS, constants.shiftB);
    shift.whenPressed(new ShiftGear());

    Button dropdownButton = new JoystickButton(driveJS, constants.dropdownB);
    dropdownButton.whenActive(new DropdownMotor());
    dropdownButton.whenInactive(new StopDropdown());

    //OPERATOR BUTTON BOARD BUTTONS
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

    Button pivotUp = new JoystickButton(gamepad, constants.pivotUpB);
    pivotUp.whenActive(new PivotUpButton());
    pivotUp.whenInactive(new PivotStopButton());

    Button pivotDown = new JoystickButton(gamepad, constants.pivotDownB);
    pivotDown.whenActive(new PivotDownButton());
    pivotDown.whenInactive(new PivotStopButton());

    /*Button frontActuatorRetract = new JoystickButton(gamepad, constants.frontActuatorRetractB);
    frontActuatorRetract.whenPressed(new FrontRetract());
    
    Button rearActuatorRetract = new JoystickButton(gamepad, constants.rearActuatorRetractB);
    rearRetractButton.whenPressed(new StopClimber());
    */
  }
}
