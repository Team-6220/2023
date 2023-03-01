package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class PneumaticSubsystem extends SubsystemBase{
    //private final Compressor compressor;
    private final Solenoid solenoid;
    private final PneumaticHub ph;
    private final GenericEntry solenoidState, compressorState;
    public PneumaticSubsystem(){
        //compressor = new Compressor(PneumaticsModuleType.REVPH);
        this.ph = new PneumaticHub(15);
        this.solenoid = ph.makeSolenoid(8);
        solenoidState = Shuffleboard.getTab("ATW").add("solenoid state", false).getEntry();
        compressorState = Shuffleboard.getTab("ATW").add("compressor state", false).getEntry();
    }
    public void enableCompressor(){
        ph.enableCompressorDigital();
    }
    public void disableCompressor(){
        ph.disableCompressor();
    }
    public void openSolenoid(){
        solenoid.toggle();
    }

    public void closeSolenoid(){
        solenoid.toggle();
    }
    @Override
    public void periodic() {
        solenoidState.setBoolean(solenoid.get());
        compressorState.setBoolean(ph.getCompressorCurrent()>0);
    }

}
