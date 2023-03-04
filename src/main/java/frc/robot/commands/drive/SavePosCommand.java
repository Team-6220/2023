package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.*;
import java.util.*;

public class SavePosCommand extends CommandBase{
    SwerveSubsystem swerveSubsystem;
    public SavePosCommand(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }
    
    @Override
    public void initialize(){
        FileWriter f;
        try{
            f = new FileWriter("SavePos.txt");

            f.write(swerveSubsystem.frontLeft.getAbsoluteEncoderDeg()+",");
            f.write(swerveSubsystem.frontRight.getAbsoluteEncoderDeg()+",");
            f.write(swerveSubsystem.backLeft.getAbsoluteEncoderDeg()+",");
            f.write(swerveSubsystem.backRight.getAbsoluteEncoderDeg()+"");
        }catch(IOException e){e.printStackTrace();}
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}