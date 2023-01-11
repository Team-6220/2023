package frc.robot.commands;

//import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoClimbCommand extends SequentialCommandGroup {
    //private double runtime;
    private final DrivetrainSubsystem m_drivetrainSubsystem;



    /* Creates an automatic climbing command for the robot.
     * Causes the robot to drive forwards 1 meter at 0.5 meters/sec
     * Ends by engaging the robot brakes
     */
    public AutoClimbCommand(DrivetrainSubsystem drivetrainSubsystem){//drives 1m up the ramp, then locks wheels
        //super(runtime);
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        
        new DriveDistanceCommand(m_drivetrainSubsystem, 1, 0.5);

        new LockWheelsCommand(m_drivetrainSubsystem);
    }

}