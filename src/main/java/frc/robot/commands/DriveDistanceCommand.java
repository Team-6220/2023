package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveDistanceCommand extends CommandBase{
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final double goal;

    public DriveDistanceCommand(DrivetrainSubsystem drivetrainSubsystem, double goal){
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.goal = goal;
    }

    //TODO: IMPLEMENT DRIVING FOR GOAL METERS
}
