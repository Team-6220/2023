package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelescopeConstants;

public class TelescopeSubsystem extends SubsystemBase{
    private final TalonSRX telescopeDriveLeader;
    private final VictorSPX telescopeDriveFollower;
    public TelescopeSubsystem(){
        this.telescopeDriveLeader = new TalonSRX(TelescopeConstants.k_TELESCOPE_DRIVE_LEADER_ID);
        this.telescopeDriveFollower = new VictorSPX(TelescopeConstants.k_TELESCOPE_DRIVE_FOLLOW_ID);

        telescopeDriveFollower.follow(telescopeDriveLeader);
    }
}
