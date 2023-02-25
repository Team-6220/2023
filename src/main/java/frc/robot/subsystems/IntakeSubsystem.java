package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    private final Compressor compressor;
    private final Solenoid solenoid;
    private final VictorSPX intakeDrive;
    public IntakeSubsystem(){
        this.intakeDrive = new VictorSPX(IntakeConstants.k_INTAKE_MOTOR_ID);
        this.solenoid = new Solenoid(PneumaticsModuleType.REVPH, IntakeConstants.SOLENOID_PORT);
        this.compressor = new Compressor(PneumaticsModuleType.REVPH); 
    }
    
    
}
