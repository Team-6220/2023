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
        this.telescopeDriveLeader = new TalonSRX(TelescopeConstants.k_TELESCOPE_DRIVE_LEADER_ID);
        this.telescopeDriveFollower = new VictorSPX(TelescopeConstants.k_TELESCOPE_DRIVE_FOLLOW_ID);
        this.telescopeDriveLeader.setInverted(TelescopeConstants.k_MOTORS_REVERSED);
        this.telescopeDriveFollower.setInverted(TelescopeConstants.k_MOTORS_REVERSED);
        this.telescopeDriveFollower.follow(telescopeDriveLeader);
        this.telescopeEncoder = new Encoder(TelescopeConstants.k_ENC_PORT_A, TelescopeConstants.k_ENC_PORT_B, TelescopeConstants.k_ENC_REV, EncodingType.k4X);
        this.telescopeTab = Shuffleboard.getTab("ATW");
        this.telescopeReading = Shuffleboard.getTab("ATW").add("telescope reading", 0).getEntry();
        this.telescopeOutput = Shuffleboard.getTab("ATW").add("telescope output", 0).getEntry();
        
    }
    public int getTelescopePosition(){
        return this.telescopeEncoder.get();
    }
    public void setMotors(double percent){
        if(getTelescopePosition() >= TelescopeConstants.k_FULL_EXTENSION && percent > 0){
            percent = 0;
        }
        telescopeDriveLeader.set(ControlMode.PercentOutput, percent);
        this.telescopeOutput.setDouble(percent);
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
