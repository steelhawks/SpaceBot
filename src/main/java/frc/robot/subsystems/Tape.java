package frc.robot.subsystems;

import edu.wpi.first.networktables.*;

import frc.robot.Robot;
import frc.robot.OI;
import frc.robot.subsystems.Ultra;

public class Tape
{
    private NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
    private NetworkTable networkTablesData = networkTables.getTable("CVResultsTable");
    private boolean initAlign;
    private boolean alignAngle;
    private double angle;
    private double xPosLeftLimit;
    private double xPosRightLimit;

    //Aligns the robot
    public void align()
    {
        if (Robot.oi.driveJS.getRawButtonPressed(2))
        {
            end();
        }
        else if (!initAlign)
        {
            System.out.println("Aligning!");
            if (Math.abs(Robot.drivetrain.gyro.getAngle()) < (getAngle() - 0.1) && getXPos() < getXPosLeftLimit())
            {
                Robot.drivetrain.rotate(0.325);
            }
            else if(Math.abs(Robot.drivetrain.gyro.getAngle()) < (getAngle() - 0.1) && getXPos() > getXPosRightLimit()) 
            {
			    Robot.drivetrain.rotate(-0.325);
            }
            else {
			    Robot.drivetrain.resetGyro();
                Robot.drivetrain.stop();
                System.out.println("Aligned!");
                initAlign = true;
            }
        }
        else
        {
            if (!Robot.ultra.isClose())
            {
                Robot.drivetrain.gyroMoveStraight(Robot.drivetrain.decimalSpeed(Robot.ultra.getRange()));
            }
            else
            {
                Robot.drivetrain.stop();
                System.out.println("Distanced!");
                alignAngle = true;
            }
        } 
    }

    //Returns the x coordinate value of the center of the hatch
    public double getXPos()
    {
        System.out.println("Center X Tape: " + networkTablesData.getEntry("CenterPoint X Tape").getDouble(0.0));
        return networkTablesData.getEntry("CenterPoint X Tape").getDouble(0.0);
    }

    //Returns the y coordinate value of the center of the hatch
    public double getYPos()
    {
        System.out.println("Center Y Tape: " + networkTablesData.getEntry("CenterPoint Y Tape").getDouble(0.0));
        return networkTablesData.getEntry("CenterPoint Y Tape").getDouble(0.0);
    }

    //Returns the apparent calculated distance between the robot and the hatch
    public double getDistance()
    {
        System.out.println("Distance Tape: " + networkTablesData.getEntry("Distance Tape").getDouble(0.0));
        return networkTablesData.getEntry("Distance Tape").getDouble(0.0);
    }

    //Returns the angle that the robot's line of vision and the center of the hatch creates 
    public double getNTAngle()
    {
        System.out.println("NT Angle Tape: " + networkTablesData.getEntry("Angle Tape").getDouble(0.0));
        return networkTablesData.getEntry("Angle Tape").getDouble(0.0);
    }

    //Sets the angle that was returned into an angle variable that can be used
    public void setAngle(double angle)
    {
       this.angle = angle;
    }

    //Returns the angle that the gyro needs in order to turn
    public double getAngle()
    {
        //System.out.println(this.angle);
        return this.angle;
    }

    //Sets the x coordinate of the left border of the hatch
    public void setXPosLeftLimit(double xPosLeftLimit)
    {
       this.xPosLeftLimit = xPosLeftLimit;
    }

    //Sets the x coordinate of the right border of the hatch
    public void setXPosRightLimit(double xPosRightLimit)
    {
       this.xPosRightLimit = xPosRightLimit;
    }

    //Returns the x coordinate of the left border of the hatch
    public double getXPosLeftLimit()
    {
        return this.xPosLeftLimit;
    }
   
    //Returns the x coordinate of the right border of the hatch
    public double getXPosRightLimit()
    {
       return this.xPosRightLimit;
    }

    //Returns the difference between the right || left border and center
    public double getXPosDiff(double xPos)
    {
        if (xPos > 160)
        {
            return xPos - 160;
        }
        return Math.abs(160 - xPos);
    }

    //Returns whether alignment and distancing to the hatch is complete
    public boolean isAligned()
    {
        return this.alignAngle;
    }

    public void end()
    {
        Robot.drivetrain.leftGroup.set(0);
        Robot.drivetrain.rightGroup.set(0);
        System.out.println("Quit...");
        this.alignAngle = true;
    }

    //Sets the gyro back to default zero values
    public void reset()
    {
        alignAngle = false;
        initAlign = false;
    }

    public void test()
    {
        getNTAngle();
        getXPos();
        getYPos();
        getDistance();
    }
}