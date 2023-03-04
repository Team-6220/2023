package frc.robot.commands.autos;
import java.util.*;
import java.io.*;
import java.io.FileWriter;
import java.io.IOException;
import frc.robot.Constants.AutoRecPlayConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.kinematics.*;

/*
*This macro records all the movements you make in teleop and saves them to the file you specify.
*make sure you record every variable you need, if you dont record the value from a motor or a solenoid,
*you won't be able to play it back. It records it in "frames" that contain a value for each output 
* you want to use during teleop
*BE AWARE: write into the same file as you do in the Play macro
*BE AWARE: Only write/read the motors/other things that you actually have fully created in 
*your code. Otherwise you'll lose robot code randomly with no reason
*In main, the try/catch structure catches any IOExceptions or FileNotFoundExceptions. Necessary to play back
*the recorded routine during autonomous
*Dennis Melamed, Melanie Quick
*22 March, 2015
*/


public class AutoRecorderCmd extends CommandBase{
	
	//this object writes values into the file we specify
	FileWriter writer;
	
	long startTime;
	boolean isOn;
	static final String autoFile = AutoRecPlayConstants.autoFile;

	public AutoRecorderCmd(SwerveSubsystem swerveSubsystem, ATWSubsystem atwSubsystem, IntakeSubsystem intakeSubsystem) throws IOException{
			startTime = System.currentTimeMillis();
			writer = new FileWriter(autoFile);
            record(swerveSubsystem, atwSubsystem, intakeSubsystem);
	}
	

	public void record(SwerveSubsystem swerveSubsystem, ATWSubsystem atwSubsystem, IntakeSubsystem intakeSubsystem) throws IOException
	{
		if(writer != null)
		{
		//start each "frame" with the elapsed time since we started recording
		if(System.currentTimeMillis() - startTime >= (15*1000)){
			return;
		}

		writer.append("" + (System.currentTimeMillis()-startTime));
		
		
		//ChassisSpeeds stuff
        SwerveModuleState[] swerveModuleStates = swerveSubsystem.getSwerveModuleStates();
        ChassisSpeeds chassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(swerveModuleStates);
		writer.append("," + chassisSpeeds.vxMetersPerSecond);
        writer.append("," + chassisSpeeds.vyMetersPerSecond);
        writer.append("," + chassisSpeeds.omegaRadiansPerSecond);
		
		//Arm
		writer.append("," + atwSubsystem.getArmOut());
		
		//Telescope
		writer.append("," + atwSubsystem.getTeleOut());

        //Wrist
        writer.append("," + atwSubsystem.getWristOut() + "\n");

        //

		/*
		 * THE LAST ENTRY OF THINGS YOU RECORD NEEDS TO HAVE A DELIMITER CONCATENATED TO 
		 * THE STRING AT THE END. OTHERWISE GIVES NOSUCHELEMENTEXCEPTION
		 */ 
		
        //Intake
        //writer.append("," + intakeSubsystem.getIntakeVal()) //FIX INTAKE STUFF
		
		/*
		 * CAREFUL. KEEP THE LAST THING YOU RECORD BETWEEN THESE TWO COMMENTS AS A
		 * REMINDER TO APPEND THE DELIMITER (aka "\n")
		 */
		}
	}
	
	
	//this method closes the writer and makes sure that all the data you recorded makes it into the file
	public void end() throws IOException
	{
		if(writer !=null)
		{
		writer.flush();
		writer.close();
		}
	}
}