package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelescopeConstants;

public class TelescopeSubsystem extends SubsystemBase{
    private TalonSRX telescopeDriveLeader;
    private VictorSPX telescopeDriveFollower;
    private Encoder telescopeEncoder;
    private GenericEntry telescopeReading, telescopeOutput;
    private ShuffleboardTab telescopeTab;
    public TelescopeSubsystem(){
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
        this.telescopeTab = Shuffleboard.getTab("ATW");
        this.telescopeReading = Shuffleboard.getTab("ATW").add("telescope reading", 0).getEntry();
        this.telescopeOutput = Shuffleboard.getTab("ATW").add("telescope output", 0).getEntry();
        
    }
    public int getTelescopePosition(){
        //update position on shuffleboard
        return this.telescopeEncoder.get();
    }
    public void setMotors(double input){
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
    public void stopMotors(){
        telescopeDriveLeader.set(ControlMode.PercentOutput, 0);
        this.telescopeOutput.setDouble(0);
    }
    @Override
    public void periodic() {
        this.telescopeReading.setDouble(telescopeEncoder.get());
    }
}
