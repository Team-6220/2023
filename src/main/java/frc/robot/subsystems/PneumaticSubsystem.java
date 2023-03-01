package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class PneumaticSubsystem extends SubsystemBase{
    private final Compressor compressor;
    private final Solenoid solenoid;
    private final GenericEntry solenoidState, compressorState;
    public PneumaticSubsystem(){
        compressor = new Compressor(PneumaticsModuleType.REVPH);
        this.solenoid = new Solenoid(PneumaticsModuleType.REVPH, IntakeConstants.SOLENOID_PORT);
        solenoidState = Shuffleboard.getTab("ATW").add("solenoid state", false).getEntry();
        compressorState = Shuffleboard.getTab("ATW").add("compressor state", compressor.isEnabled()).getEntry();
    }
    public void enableCompressor(){
        if(!compressor.isEnabled()){
            compressor.enableAnalog(60, 80);
            compressor.enableDigital();
        }
    }
    public void disableCompressor(){
        if(compressor.isEnabled()){
            compressor.disable();
        }
    }
    public void openSolenoid(){
        if(!solenoid.get()){
            solenoid.toggle();
        }
    }

    public void closeSolenoid(){
        if(solenoid.get()){
            solenoid.toggle();
        }
    }
    @Override
    public void periodic() {
        solenoidState.setBoolean(solenoid.get());
        compressorState.setBoolean(compressor.isEnabled());
    }

}
