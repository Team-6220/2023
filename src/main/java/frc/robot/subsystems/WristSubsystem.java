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
import frc.robot.Constants.*;

public class WristSubsystem extends SubsystemBase{
    private TalonSRX wristDriveMotor;
    private Encoder wristEncoder;
    private GenericEntry wristReading, wristOutput;
    public WristSubsystem(){
        this.wristDriveMotor = new TalonSRX(WristConstants.k_WRIST_MOTOR_ID);
        this.wristEncoder = new Encoder(WristConstants.k_ENC_PORT_A, WristConstants.k_ENC_PORT_B, WristConstants.k_ENC_REV, EncodingType.k4X);
        this.wristReading = Shuffleboard.getTab("ATW").add("wrist reading", 0).getEntry();
        this.wristOutput = Shuffleboard.getTab("ATW").add("wrist output", 0).getEntry();
    }

    public int getTelescopePosition(){
        return this.wristEncoder.get();
    }

    public void setMotors(double percent){
        wristDriveMotor.set(ControlMode.PercentOutput, percent);
        this.wristOutput.setDouble(percent);
    }
    public void stopMotors(){
        wristDriveMotor.set(ControlMode.PercentOutput, 0);
        this.wristOutput.setDouble(0);
    }
    @Override
    public void periodic() {
        this.wristReading.setDouble(wristEncoder.get());
    }
}