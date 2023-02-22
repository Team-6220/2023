package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class CalibrateWheelsCmd extends CommandBase{
    SwerveSubsystem swerveSubsystem;
    public CalibrateWheelsCmd(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }
    
    @Override
    public void initialize(){
        double[] offsets = this.swerveSubsystem.offsets();
        Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg += offsets[0];
        Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg += offsets[1];
        Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg += offsets[2];
        Constants.DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg += offsets[3];
        swerveSubsystem.updateOffsets();
        swerveSubsystem.resetEncoders();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
