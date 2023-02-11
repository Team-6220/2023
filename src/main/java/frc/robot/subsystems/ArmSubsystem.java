package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase{
    private final CANSparkMax armDriveLeader;
    private final CANSparkMax armDriveFollower;
    private final SparkMaxPIDController armPidController;
    private final RelativeEncoder armEncoder;
    private ShuffleboardTab armTab;
    private GenericEntry armAngle, pidGains, desiredArmAngle;
    public ArmSubsystem(){
        armDriveLeader = new CANSparkMax(ArmConstants.k_ARM_DRIVE_LEADER_ID, MotorType.kBrushless);
        armDriveLeader.restoreFactoryDefaults();
        armDriveLeader.setInverted(ArmConstants.k_MOTORS_REVERSED);
        armDriveLeader.setIdleMode(IdleMode.kBrake);

        armDriveFollower  = new CANSparkMax(ArmConstants.k_ARM_DRIVE_FOLLOW_ID, MotorType.kBrushless);
        armDriveFollower.restoreFactoryDefaults();
        armDriveFollower.follow(armDriveLeader);
        armDriveFollower.setInverted(ArmConstants.k_MOTORS_REVERSED);
        armDriveFollower.setIdleMode(IdleMode.kBrake);

        armPidController = armDriveLeader.getPIDController();
        armPidController.setP(.005);
        armPidController.setI(0);
        armPidController.setD(0);
        armPidController.setFF(0);
        armPidController.setIZone(0);

        armEncoder = armDriveLeader.getEncoder(Type.kQuadrature, 4096);

        armDriveFollower.burnFlash();
        armDriveLeader.burnFlash();
        
        armTab = Shuffleboard.getTab("Arm");
        armAngle = Shuffleboard.getTab("Arm").add("Arm Angle", armEncoder.getPosition()).getEntry();
    }

    public void setMotors(double input){
        armDriveLeader.set(input);
    }

    public void setMotors(double setpoint, boolean holding){

    }

    public void stop(){
        armDriveLeader.set(0);
    }

    
}