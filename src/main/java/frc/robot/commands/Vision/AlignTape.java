package frc.robot.commands.Vision;

import frc.robot.Robot;
import frc.robot.subsystems.Ultra;
import frc.robot.subsystems.Tape;

import edu.wpi.first.wpilibj.command.Command;

public class AlignTape extends Command
{    
    Tape TAPE = new Tape();

    public AlignTape() {}

    //Sets the gyro and angle at zero, both the right and left coordinates to default values, and ultrasound is enabled
    public void initialize()
    {
        Robot.drivetrain.resetGyro();
        Robot.tape.reset();
        Robot.tape.setAngle(TAPE.getNTAngle());
        Robot.tape.setXPosLeftLimit(157.5);
        Robot.tape.setXPosRightLimit(162.5);
        Robot.ultra.enable();
    }

    //Executes the vision code
    public void execute()
    {
        Robot.tape.align();
    }

    //Returns whether the vision code has finished
    public boolean isFinished()
    {
        return Robot.tape.isAligned();
    }

    //Terminates the vision code, motors are turned off, and the ultrasound is disabled
    public void end()
    {
        Robot.drivetrain.leftGroup.set(0);
        Robot.drivetrain.rightGroup.set(0);
        Robot.ultra.disable();
    }

    //Suspends robot activity, motors are turned off, and ultrasound is disabled
    public void interupted()
    {
        Robot.drivetrain.leftGroup.set(0);
        Robot.drivetrain.rightGroup.set(0);
        Robot.ultra.disable();
    }
}


