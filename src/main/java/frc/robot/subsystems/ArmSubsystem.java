package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.TelescopeConstants;

public class ArmSubsystem extends SubsystemBase{
    private final CANSparkMax armDriveLeader;
    private final CANSparkMax armDriveFollower;
    private final SparkMaxPIDController armPidController;
    private final RelativeEncoder armEncoder;
    private GenericEntry armAngle, pidGains, desiredArmAngle, armOutput;
    private final TelescopeSubsystem telescopeSubsystem;
    public ArmSubsystem(TelescopeSubsystem telescopeSubsystem){
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

        this.armDriveFollower.burnFlash();
        this.armDriveLeader.burnFlash();
        
        this.telescopeSubsystem = telescopeSubsystem;

        this.armAngle = Shuffleboard.getTab("ATW").add("Arm Angle", 0).getEntry();
        this.armOutput = Shuffleboard.getTab("ATW").add("Arm Motor Output", 0).getEntry();
    }

    public void setMotors(double input){
        input += getHoldingOutput();
        this.armDriveLeader.set(input);
        armOutput.setDouble(input);
    }

    public double getArmPositionDegrees(){
        double angle = (armEncoder.getPosition()*3)+90;
        return angle; 
    }

    public double getTelescopePosition(){
        return this.telescopeSubsystem.getTelescopePosition();
    } 

    public double getHoldingOutput(){
        double out = .023 + (getTelescopePosition()/TelescopeConstants.k_FULL_EXTENSION)*(.1-.023);
        out *= (getArmPositionDegrees() < 0)?-1:1;
        return out;
    }

    public void stop(){
        setMotors(0);
    }

    @Override
    public void periodic() {
        this.armAngle.setDouble(getArmPositionDegrees());
    }

    
}