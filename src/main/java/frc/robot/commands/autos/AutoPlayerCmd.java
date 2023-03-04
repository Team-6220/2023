package frc.robot.commands.autos;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.*;
import java.io.*;
import java.io.FileWriter;
import java.io.IOException;
import frc.robot.Constants.AutoRecPlayConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;
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
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/*Code outline to implement playing back a macro recorded in BTMacroRecord
*Be sure to read out of the same file created in BTMacroRecord
*BEWARE OF: setting your motors in a different order than in BTMacroRecord and changing motor values before
*time is up. Both issues are dealt with and explained below. Also only read/write from/to the motors 
*you have fully coded for, otherwise your code will cut out for no reason. 
*In main, the try/catch structure catches any IOExceptions or FileNotFoundExceptions. Necessary to play back
*the recorded routine during autonomous
*Dennis Melamed and Melanie (sorta, she slept)
*March 22nd, 2015
*/


public class AutoPlayerCmd extends CommandBase{
	Scanner scanner;
	long startTime;

	boolean onTime = true;
	double nextDouble;
	
    static final String autoFile = AutoRecPlayConstants.autoFile;

	public AutoPlayerCmd(SwerveSubsystem swerveSubsystem, ATWSubsystem atwSubsystem, IntakeSubsystem intakeSubsystem) throws FileNotFoundException
	{
		//create a scanner to read the file created during BTMacroRecord
		//scanner is able to read out the doubles recorded into recordedAuto.csv (as of 2015)
		scanner = new Scanner(new File(autoFile));
		
		//let scanner know that the numbers are separated by a comma or a newline, as it is a .csv file
		scanner.useDelimiter(",|\\n");
		
		//lets set start time to the current time you begin autonomous
		startTime = System.currentTimeMillis();	

        while((scanner != null) && (scanner.hasNextDouble()))
		{
			double t_delta;
			
			//if we have waited the recorded amount of time assigned to each respective motor value,
			//then move on to the next double value
			//prevents the macro playback from getting ahead of itself and writing different
			//motor values too quickly
			if(onTime)
			{
				//take next value
				nextDouble = scanner.nextDouble();
			}
			
			//time recorded for values minus how far into replaying it we are--> if not zero, hold up
			t_delta = nextDouble - (System.currentTimeMillis()-startTime);
			
			//if we are on time, then set motor values
			if (t_delta <= 0)
			{
				//for 2015 robot. these are all the motors available to manipulate during autonomous.
				//it is extremely important to set the motors in the SAME ORDER as was recorded in BTMacroRecord
				//otherwise, motor values will be sent to the wrong motors and the robot will be unpredicatable
				
                //chassisSpeeds
                double x = scanner.nextDouble();
                double y = scanner.nextDouble();
                double omega = scanner.nextDouble();
                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(x,y,omega);
                SwerveModuleState[] arr = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                swerveSubsystem.setModuleStates(arr);
				
                //Arm
                atwSubsystem.setArmMotors(scanner.nextDouble());

                //Telescope
                atwSubsystem.setTeleMotors(scanner.nextDouble());

                //Wrist
                atwSubsystem.setWristMotor(scanner.nextDouble());

                //Intake
				//TODO
				
				//go to next double
				onTime = true;
			}
			//else don't change the values of the motors until we are "onTime"
			else
			{
				onTime = false;
			}
		}
		//end play, there are no more values to find
		this.end(swerveSubsystem,atwSubsystem,intakeSubsystem);
		
	}
	
	// public void play(SwerveSubsystem swerveSubsystem, ATWSubsystem atwSubsystem, IntakeSubsystem intakeSubsystem)
	// {
	// 	//if recordedAuto.csv has a double to read next, then read it
	// 	if ((scanner != null) && (scanner.hasNextDouble()))
	// 	{
	// 		double t_delta;
			
	// 		//if we have waited the recorded amount of time assigned to each respective motor value,
	// 		//then move on to the next double value
	// 		//prevents the macro playback from getting ahead of itself and writing different
	// 		//motor values too quickly
	// 		if(onTime)
	// 		{
	// 			//take next value
	// 			nextDouble = scanner.nextDouble();
	// 		}
			
	// 		//time recorded for values minus how far into replaying it we are--> if not zero, hold up
	// 		t_delta = nextDouble - (System.currentTimeMillis()-startTime);
			
	// 		//if we are on time, then set motor values
	// 		if (t_delta <= 0)
	// 		{
	// 			//for 2015 robot. these are all the motors available to manipulate during autonomous.
	// 			//it is extremely important to set the motors in the SAME ORDER as was recorded in BTMacroRecord
	// 			//otherwise, motor values will be sent to the wrong motors and the robot will be unpredicatable
				
    //             //chassisSpeeds
    //             double x = scanner.nextDouble();
    //             double y = scanner.nextDouble();
    //             double omega = scanner.nextDouble();
    //             ChassisSpeeds chassisSpeeds = new ChassisSpeeds(x,y,omega);
    //             SwerveModuleState[] arr = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    //             swerveSubsystem.setModuleStates(arr);
				
    //             //Arm
    //             atwSubsystem.setArmMotors(scanner.nextDouble());

    //             //Telescope
    //             atwSubsystem.setTeleMotors(scanner.nextDouble());

    //             //Wrist
    //             atwSubsystem.setWristMotor(scanner.nextDouble());

    //             //Intake
	// 			//TODO
				
	// 			//go to next double
	// 			onTime = true;
	// 		}
	// 		//else don't change the values of the motors until we are "onTime"
	// 		else
	// 		{
	// 			onTime = false;
	// 		}
	// 	}
	// 	//end play, there are no more values to find
	// 	else
	// 	{
	// 		this.end(swerveSubsystem,atwSubsystem,intakeSubsystem);
	// 		if (scanner != null) 
	// 		{
	// 			scanner.close();
	// 			scanner = null;
	// 		}
	// 	}
		
	// }
	
	//stop motors and end playing the recorded file
	public void end(SwerveSubsystem swerveSubsystem, ATWSubsystem atwSubsystem, IntakeSubsystem intakeSubsystem)
	{
		swerveSubsystem.stopModules();
        atwSubsystem.stopArm();
        atwSubsystem.stopTeleMotors();
        atwSubsystem.stopWristMotor();

        //TODO: stop intake
		
	}
	
}