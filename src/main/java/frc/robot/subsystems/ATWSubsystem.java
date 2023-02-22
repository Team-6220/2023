package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
public class ATWSubsystem extends SubsystemBase{
    private final CANSparkMax armDriveLeader;
    private final CANSparkMax armDriveFollower;
    private final SparkMaxPIDController armPidController;
    private final RelativeEncoder armEncoder;
    private final GenericEntry armAngle, armOutput, telescopeReading, telescopeOutput;
    private final TalonSRX telescopeDriveLeader;
    private final VictorSPX telescopeDriveFollower;
    private final Encoder telescopeEncoder;
    public ATWSubsystem(){
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

        //init telescope lead (has to be talon)
        this.telescopeDriveLeader = new TalonSRX(TelescopeConstants.k_TELESCOPE_DRIVE_LEADER_ID);
        this.telescopeDriveLeader.setInverted(TelescopeConstants.k_MOTORS_REVERSED);
        //init telescope follow (has to be victor)
        this.telescopeDriveFollower = new VictorSPX(TelescopeConstants.k_TELESCOPE_DRIVE_FOLLOW_ID);
        this.telescopeDriveFollower.setInverted(TelescopeConstants.k_MOTORS_REVERSED);
        this.telescopeDriveFollower.follow(telescopeDriveLeader);
        //throughbore encoder (encoding type changes the pulses per revolution (higher = more precision))
        this.telescopeEncoder = new Encoder(TelescopeConstants.k_ENC_PORT_A, TelescopeConstants.k_ENC_PORT_B, TelescopeConstants.k_ENC_REV, EncodingType.k4X);
        
        //shuffleboard
        this.telescopeReading = Shuffleboard.getTab("ATW").add("telescope reading", 0).getEntry();
        this.telescopeOutput = Shuffleboard.getTab("ATW").add("telescope output", 0).getEntry();
        this.armAngle = Shuffleboard.getTab("ATW").add("Arm Angle", 0).getEntry();
        this.armOutput = Shuffleboard.getTab("ATW").add("Arm Motor Output", 0).getEntry();
    }
    public void setArmMotors(double input){
        //stop it from going too far
        if(Math.abs(getArmPositionDegrees()) >= ArmConstants.k_SOFT_LIMIT && !((getArmPositionDegrees() < 0) ^ (input < 0))){
            input = 0;
        }
        //reduce input because adding the holding value could make it over 1
        input *= .9;
        //get holding output flips itself so we just add this
        input += getArmHoldingOutput();
        //we only set the leader cuz the follower will just do the same
        this.armDriveLeader.set(input);
        //updating the shuffle board output
        armOutput.setDouble(input);
    }

    public double getArmPositionDegrees(){
        double angle = (armEncoder.getPosition());
        return angle+90; 
    }

    public double getArmHoldingOutput(){
        //first formula is just slope-intercept as .023 is the min and .1 is the max and it should increase linearly from there
        double out = .023 + (getTelescopePosition()/TelescopeConstants.k_FULL_EXTENSION)*(.1-.023);
        //torque exerted by gravity changes with angle, this should account for that
        out *= Math.sin(Math.toRadians(getArmPositionDegrees()));
        out *= (getArmPositionDegrees() > 0)?-1:1; 
        return out;
    }

    public void stopArm(){
        //this will actually output the holding value not 0
        setArmMotors(0);
    }

    @Override
    public void periodic() {
        //update position on shuffleboard
        this.armAngle.setDouble(getArmPositionDegrees());
        this.telescopeReading.setDouble(telescopeEncoder.get());
    }
    public int getTelescopePosition(){
        //update position on shuffleboard
        return this.telescopeEncoder.get();
    }
    public void setTeleMotors(double input){
        if(getTelescopePosition() >= TelescopeConstants.k_FULL_EXTENSION && input > 0){
            //stop it from extending beyond full
            input = 0;
        }
        if(getTelescopePosition() <= TelescopeConstants.k_FULL_RETRACTION && input < 0){
            //stop it from extending beyond full
            input = 0;
        }
        telescopeDriveLeader.set(ControlMode.PercentOutput, input);
        this.telescopeOutput.setDouble(input);
    }
    public void stopTeleMotors(){
        telescopeDriveLeader.set(ControlMode.PercentOutput, 0);
        this.telescopeOutput.setDouble(0);
    }
}