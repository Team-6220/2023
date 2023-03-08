package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{

    private final VictorSPX intakeDrive;
    private final Compressor compressor;
    private final Solenoid solenoid;
    private final GenericEntry solenoidState, compressorState;
    private boolean pressureSwitch;
    public IntakeSubsystem(){
        this.intakeDrive = new VictorSPX(IntakeConstants.k_INTAKE_MOTOR_ID);
        this.intakeDrive.setInverted(false);
        this.intakeDrive.setNeutralMode(NeutralMode.Brake);
        compressor = new Compressor(15, PneumaticsModuleType.REVPH);
        this.solenoid = new Solenoid(15, PneumaticsModuleType.REVPH, IntakeConstants.SOLENOID_PORT);
        solenoidState = Shuffleboard.getTab("ATW").add("solenoid state", false).getEntry();
        compressorState = Shuffleboard.getTab("ATW").add("compressor state", compressor.isEnabled()).getEntry();
        pressureSwitch = compressor.getPressureSwitchValue();
    }

    public void setMotors(double input){
        this.intakeDrive.set(ControlMode.PercentOutput, input);
    }

    public void stopMotors(){
        this.intakeDrive.set(ControlMode.PercentOutput, 0);
    }
    public void toggleCompressor(){
        pressureSwitch = compressor.getPressureSwitchValue();
        if(!compressor.isEnabled() && pressureSwitch){
            compressor.enableDigital();
        }else{
            compressor.disable();
        }
    }
    public void toggleSolenoid(){
        solenoid.set(!solenoid.get());
    }
    public boolean getSolenoidState(){
        return this.solenoid.get();
    }
    public boolean getCompressorState(){
        return this.compressor.isEnabled();
    }
    @Override
    public void periodic() {
        solenoidState.setBoolean(solenoid.get());
        compressorState.setBoolean(compressor.isEnabled());
        pressureSwitch = compressor.getPressureSwitchValue();
        if(!pressureSwitch){
            compressor.disable();
        }
    }
    
}
