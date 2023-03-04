package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
        Scanner sc = new Scanner("SavePos.txt");
        String str = sc.next();
        String[] arr = str.split("[,]+");

        swerveSubsystem.frontLeft.setRelative(swerveSubsystem.frontLeft.getAbsoluteEncoderDeg() - Double.parseDouble(arr[0]));
        swerveSubsystem.frontRight.setRelative(swerveSubsystem.frontRight.getAbsoluteEncoderDeg() - Double.parseDouble(arr[1]));
        swerveSubsystem.backLeft.setRelative(swerveSubsystem.backLeft.getAbsoluteEncoderDeg() - Double.parseDouble(arr[2]));
        swerveSubsystem.backRight.setRelative(swerveSubsystem.backRight.getAbsoluteEncoderDeg() - Double.parseDouble(arr[3]));
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}