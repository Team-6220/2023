package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoClimbCommand extends TimedCommand {
    private double runtime;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private double speed;


    //@Param
    //runtime is time to run
    //speed is motor velocity in m/s
    //runtime * speed should equal 0.99 or 1 meter

    public AutoClimbCommand(double runtime, DrivetrainSubsystem drivetrainSubsystem, double speed){
        super(runtime);
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.speed = speed;
    }

    @Override
    protected void initialize(){
        m_drivetrainSubsystem.drive(new ChassisSpeeds(speed,0,0));
    }

    @Override
    protected void end(){
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
        m_drivetrainSubsystem.lockWheels();
    }
}