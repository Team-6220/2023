package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
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
    private GenericEntry armAngle, pidGains, desiredArmAngle, telescopeReading;
    public ArmSubsystem(){
        this.armDriveLeader = new CANSparkMax(ArmConstants.k_ARM_DRIVE_LEADER_ID, MotorType.kBrushless);
        this.armDriveLeader.restoreFactoryDefaults();
        this.armDriveLeader.setInverted(ArmConstants.k_MOTORS_REVERSED);
        this.armDriveLeader.setIdleMode(IdleMode.kBrake);

        this.armDriveFollower  = new CANSparkMax(ArmConstants.k_ARM_DRIVE_FOLLOW_ID, MotorType.kBrushless);
        this.armDriveFollower.restoreFactoryDefaults();
        this.armDriveFollower.follow(armDriveLeader);
        this.armDriveFollower.setInverted(ArmConstants.k_MOTORS_REVERSED);
        this.armDriveFollower.setIdleMode(IdleMode.kBrake);

        this.armPidController = armDriveLeader.getPIDController();
        this.armPidController.setP(.005);
        this.armPidController.setI(0);
        this.armPidController.setD(0);
        this.armPidController.setFF(0);
        this.armPidController.setIZone(0);

        this.armEncoder = this.armDriveLeader.getEncoder(Type.kHallSensor, 42);
        this.armEncoder.setPositionConversionFactor(1/120);

        this.armDriveFollower.burnFlash();
        this.armDriveLeader.burnFlash();
        
        this.armTab = Shuffleboard.getTab("ATW");
        this.armAngle = Shuffleboard.getTab("ATW").add("Arm Angle", 0).getEntry();
        this.telescopeReading = Shuffleboard.getTab("ATW").add("telescope reading", 0).getEntry();
    }

    public void setMotors(double input, int type){
        if(type == ArmConstants.ControlType.k_PERCENT){
            this.armDriveLeader.set(input);
            
        }else if(type == ArmConstants.ControlType.k_POSITION){
            this.armPidController.setReference(input, ControlType.kPosition);
        }
    }

    public double getArmPositionDegrees(){
        return (ArmConstants.k_ARM_ENCODER_PCF * armEncoder.getPosition())+90; 
    }

    public double getTelescopePosition(){
        return this.telescopeReading.getDouble(0);
    } 

    public void stop(){
        armDriveLeader.set(0);
    }
    @Override
    public void periodic() {
        this.armAngle.setDouble(getArmPositionDegrees());
    }

    
}