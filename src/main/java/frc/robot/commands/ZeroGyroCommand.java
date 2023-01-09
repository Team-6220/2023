package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import com.kauailabs.navx.frc.*;

public class ZeroGyroCommand extends CommandBase{
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final AHRS m_navx;

    public ZeroGyroCommand(DrivetrainSubsystem drivetrainSubsystem){
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_navx = m_drivetrainSubsystem.getNavx();
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_navx.zeroYaw();
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
