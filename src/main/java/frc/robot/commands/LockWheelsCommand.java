package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LockWheelsCommand extends CommandBase{
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private static boolean lock;

    public LockWheelsCommand(DrivetrainSubsystem drivetrainSubsystem){
        this.m_drivetrainSubsystem = drivetrainSubsystem;
    }

    @Override
    public void execute(){
        lock = !lock;
        
        if(lock){
            m_drivetrainSubsystem.getFL().set(0,Math.toRadians(45));
            m_drivetrainSubsystem.getFR().set(0,Math.toRadians(-45));
            m_drivetrainSubsystem.getBL().set(0,Math.toRadians(-45));
            m_drivetrainSubsystem.getBR().set(0,Math.toRadians(45));
        }else{//sets to 0, try and see if we can get prev and return to last state
            m_drivetrainSubsystem.getFL().set(0,Math.toRadians(0));
            m_drivetrainSubsystem.getFR().set(0,Math.toRadians(0));
            m_drivetrainSubsystem.getBL().set(0,Math.toRadians(0));
            m_drivetrainSubsystem.getBR().set(0,Math.toRadians(0));
        }
    }

    @Override
    public void end(boolean interrupted){

    }
}