package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.*;
import java.util.*;

public class ResetWheelsCommand extends CommandBase{
    SwerveSubsystem swerveSubsystem;
    public ResetWheelsCommand(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }
    
    @Override
    public void initialize(){
        // try{
        //     Scanner sc = new Scanner(new FileReader("SavePos.txt"));
        //     String str = sc.next();
        //     String[] arr = str.split("[,]+");

        //     swerveSubsystem.frontLeft.setRelative(swerveSubsystem.frontLeft.getAbsoluteEncoderDeg() - DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg);
        //     swerveSubsystem.frontRight.setRelative(swerveSubsystem.frontRight.getAbsoluteEncoderDeg() - DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg);
        //     swerveSubsystem.backLeft.setRelative(swerveSubsystem.backLeft.getAbsoluteEncoderDeg() - DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg);
        //     swerveSubsystem.backRight.setRelative(swerveSubsystem.backRight.getAbsoluteEncoderDeg() - DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg);
        // }catch (FileNotFoundException e){
        //     e.printStackTrace();
        // }

        swerveSubsystem.frontLeft.setRelative(Math.toRadians(swerveSubsystem.frontLeft.getAbsoluteEncoderDeg() - DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg));
        swerveSubsystem.frontRight.setRelative(Math.toRadians(swerveSubsystem.frontRight.getAbsoluteEncoderDeg() - DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg));
        swerveSubsystem.backLeft.setRelative(Math.toRadians(swerveSubsystem.backLeft.getAbsoluteEncoderDeg() - DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg));
        swerveSubsystem.backRight.setRelative(Math.toRadians(swerveSubsystem.backRight.getAbsoluteEncoderDeg() - DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg));
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}