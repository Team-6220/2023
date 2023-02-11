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
    private GenericEntry telescopeReading;
    private ShuffleboardTab telescopeTab;
    public TelescopeSubsystem(){
        this.telescopeDriveLeader = new TalonSRX(TelescopeConstants.k_TELESCOPE_DRIVE_LEADER_ID);
        this.telescopeDriveFollower = new VictorSPX(TelescopeConstants.k_TELESCOPE_DRIVE_FOLLOW_ID);
        this.telescopeDriveLeader.setInverted(TelescopeConstants.k_MOTORS_REVERSED);
        this.telescopeDriveFollower.setInverted(TelescopeConstants.k_MOTORS_REVERSED);
        this.telescopeDriveFollower.follow(telescopeDriveLeader);
        this.telescopeEncoder = new Encoder(TelescopeConstants.k_ENC_PORT_A, TelescopeConstants.k_ENC_PORT_B, TelescopeConstants.k_ENC_REV, EncodingType.k4X);
        telescopeTab = Shuffleboard.getTab("Telescope");
        this.telescopeReading = Shuffleboard.getTab("Telescope").add("raw encoder", telescopeEncoder.get()).getEntry();
    }

    public void setMotors(double percent){
        telescopeDriveLeader.set(ControlMode.PercentOutput, percent);
    }

    public void setPosition(double setpoint){

    }
}
