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
        compressor = new Compressor(15, PneumaticsModuleType.REVPH);
        this.solenoid = new Solenoid(15, PneumaticsModuleType.REVPH, IntakeConstants.SOLENOID_PORT);
        solenoidState = Shuffleboard.getTab("ATW").add("solenoid state", false).getEntry();
        compressorState = Shuffleboard.getTab("ATW").add("compressor state", compressor.isEnabled()).getEntry();
    }
    public void enableCompressor(){
        if(!compressor.isEnabled()){
            compressor.enableAnalog(60, 80);
            compressor.enableDigital();
        }else{
            compressor.disable();
        }
        //solenoid.set(true);
    }
    public void disableCompressor(){
        if(compressor.isEnabled()){
            compressor.disable();
        }
        //solenoid.set(false);
    }
    public void openSolenoid(){
        solenoid.set(!solenoid.get());
    }

    public void closeSolenoid(){
        solenoid.set(false);
    }
    @Override
    public void periodic() {
        solenoidState.setBoolean(solenoid.get());
        compressorState.setBoolean(compressor.isEnabled());
    }

}
