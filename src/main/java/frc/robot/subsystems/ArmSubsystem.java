package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase{
    private final CANSparkMax armDriveLeader;
    private final CANSparkMax armDriveFollower;
    private final SparkMaxPIDController armPidController;
    public ArmSubsystem(){
        armDriveLeader = new CANSparkMax(ArmConstants.k_ARM_DRIVE_LEADER_ID, MotorType.kBrushless);
        armDriveFollower  = new CANSparkMax(ArmConstants.k_ARM_DRIVE_FOLLOW_ID, MotorType.kBrushless);
        armDriveFollower.restoreFactoryDefaults();
        armDriveLeader.restoreFactoryDefaults();
        armDriveFollower.follow(armDriveLeader);
        armDriveFollower.setInverted(ArmConstants.k_MOTORS_REVERSED);
        armDriveLeader.setInverted(ArmConstants.k_MOTORS_REVERSED);
        armPidController = armDriveLeader.getPIDController();
        armPidController.setP(.005);
        armPidController.setI(0);
        armPidController.setD(0);
        armPidController.setFF(0);
        armPidController.setIZone(0);
        
        armDriveFollower.burnFlash();
        armDriveLeader.burnFlash();
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