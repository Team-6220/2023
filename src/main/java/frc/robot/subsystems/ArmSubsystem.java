package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class ArmSubsystem extends SubsystemBase{
    private final CANSparkMax armDriveLeader;
    private final CANSparkMax armDriveFollower;
    private final RelativeEncoder encoder;
    private SparkMaxPIDController pidController;

    public ArmSubsystem(){
        armDriveLeader = new CANSparkMax(ArmConstants.k_ARM_DRIVE_LEADER_ID, MotorType.kBrushless);
        armDriveFollower  = new CANSparkMax(ArmConstants.k_ARM_DRIVE_FOLLOW_ID, MotorType.kBrushless);
        armDriveFollower.restoreFactoryDefaults();
        armDriveLeader.restoreFactoryDefaults();
        armDriveFollower.follow(armDriveLeader);
        armDriveFollower.setInverted(ArmConstants.k_MOTORS_REVERSED);
        armDriveLeader.setInverted(ArmConstants.k_MOTORS_REVERSED);
        
        encoder = armDriveLeader.getEncoder();
        encoder.setPositionConversionFactor(ArmConstants.k_ARM_ENCODER_POSITION_CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(ArmConstants.k_ARM_ENCODER_VELOCITY_CONVERSION_FACTOR);

        this.pidController = armDriveLeader.getPIDController();
        pidController.setP(ArmConstants.k_P);
        pidController.setI(ArmConstants.k_I);
        pidController.setD(ArmConstants.k_D);
        pidController.setIZone(ArmConstants.k_Iz);
        pidController.setFF(ArmConstants.k_FF);
        pidController.setOutputRange(ArmConstants.k_MIN_OUT, ArmConstants.k_MAX_OUT);

        armDriveFollower.burnFlash();
        armDriveLeader.burnFlash();
    }

    public double getEncoderPosition(){
        return encoder.getPosition();
    }
    public double getEncoderVelocity(){
        return encoder.getVelocity();
    }
    public void setDesiredPosition(double target){
        
    }
}
