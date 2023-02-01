package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class ArmSubsystem extends SubsystemBase{
    private final CANSparkMax armDriveLeader;
    private final CANSparkMax armDriveFollower;
    public ArmSubsystem(){
        armDriveLeader = new CANSparkMax(ArmConstants.k_ARM_DRIVE_LEADER_ID, MotorType.kBrushless);
        armDriveFollower  = new CANSparkMax(ArmConstants.k_ARM_DRIVE_FOLLOW_ID, MotorType.kBrushless);
        armDriveFollower.restoreFactoryDefaults();
        armDriveLeader.restoreFactoryDefaults();
        armDriveFollower.follow(armDriveLeader);
        armDriveFollower.setInverted(ArmConstants.k_MOTORS_REVERSED);
        armDriveLeader.setInverted(ArmConstants.k_MOTORS_REVERSED);
        armDriveFollower.burnFlash();
        armDriveLeader.burnFlash();
    }

    public void setMotors(double input){
        armDriveLeader.set(input);
    }

    public void stop(){
        armDriveLeader.set(0);
    }

    
}