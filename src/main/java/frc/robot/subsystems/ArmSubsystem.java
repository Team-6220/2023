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
    private GenericEntry armAngle, armOutput;
    private final TelescopeSubsystem telescopeSubsystem;
    public ArmSubsystem(TelescopeSubsystem telescopeSubsystem){
        //init arm lead
        this.armDriveLeader = new CANSparkMax(ArmConstants.k_ARM_DRIVE_LEADER_ID, MotorType.kBrushless);
        this.armDriveLeader.restoreFactoryDefaults();
        this.armDriveLeader.setInverted(ArmConstants.k_MOTORS_REVERSED);
        this.armDriveLeader.setIdleMode(IdleMode.kBrake);

        //init arm follower (will receive the same output that the arm lead has) (if motors need to spin same speed opp direction just make sure they have different inverted values)
        this.armDriveFollower  = new CANSparkMax(ArmConstants.k_ARM_DRIVE_FOLLOW_ID, MotorType.kBrushless);
        this.armDriveFollower.restoreFactoryDefaults();
        this.armDriveFollower.follow(armDriveLeader);
        this.armDriveFollower.setInverted(ArmConstants.k_MOTORS_REVERSED);
        this.armDriveFollower.setIdleMode(IdleMode.kBrake);

        //neos have built in pid controllers that work with their built in encoders
        this.armPidController = armDriveLeader.getPIDController();
        this.armPidController.setP(.005);
        this.armPidController.setI(0);
        this.armPidController.setD(0);
        this.armPidController.setFF(0);
        this.armPidController.setIZone(0);

        //built in neo encoder (42 ppr)
        this.armEncoder = this.armDriveLeader.getEncoder(Type.kHallSensor, 42);

        //ALWAYS BURN FLASH this saves changes like idle mode or set inverted to the spark max
        this.armDriveFollower.burnFlash();
        this.armDriveLeader.burnFlash();
        
        this.telescopeSubsystem = telescopeSubsystem;

        //create little output spots on shuffleboard
        this.armAngle = Shuffleboard.getTab("ATW").add("Arm Angle", 0).getEntry();
        this.armOutput = Shuffleboard.getTab("ATW").add("Arm Motor Output", 0).getEntry();
    }

    public void setMotors(double input){
        //stop it from going too far
        if(Math.abs(getArmPositionDegrees()) >= ArmConstants.k_SOFT_LIMIT && !((getArmPositionDegrees() < 0) ^ (input < 0))){
            input = 0;
        }
        //reduce input because adding the holding value could make it over 1
        input *= .9;
        //get holding output flips itself so we just add this
        input += getHoldingOutput();
        //we only set the leader cuz the follower will just do the same
        this.armDriveLeader.set(input);
        //updating the shuffle board output
        armOutput.setDouble(input);
    }

    public double getArmPositionDegrees(){
        double angle = (armEncoder.getPosition());
        return angle+90; 
    }

    public double getTelescopePosition(){
        return this.telescopeSubsystem.getTelescopePosition();
    } 

    public double getHoldingOutput(){
        //first formula is just slope-intercept as .023 is the min and .1 is the max and it should increase linearly from there
        double out = .023 + (getTelescopePosition()/TelescopeConstants.k_FULL_EXTENSION)*(.1-.023);
        //torque exerted by gravity changes with angle, this should account for that
        out *= Math.sin(Math.toRadians(getArmPositionDegrees()));
        out *= (getArmPositionDegrees() > 0)?-1:1; 
        return out;
    }

    public void stop(){
        //this will actually output the holding value not 0
        setMotors(0);
    }

    @Override
    public void periodic() {
        //update position on shuffleboard
        this.armAngle.setDouble(getArmPositionDegrees());
    }
}