package frc.robot.commands;

//import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoClimbCommand extends SequentialCommandGroup {
    //private double runtime;
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    public AutoClimbCommand(double runtime, DrivetrainSubsystem drivetrainSubsystem){
        this.m_drivetrainSubsystem = drivetrainSubsystem;

        new DriveDistanceCommand(m_drivetrainSubsystem, 1);

        new LockWheels(m_drivetrainSubsystem);
    }
}