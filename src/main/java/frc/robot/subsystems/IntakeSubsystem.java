package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{

    private final VictorSPX intakeDrive;
    public IntakeSubsystem(){
        this.intakeDrive = new VictorSPX(IntakeConstants.k_INTAKE_MOTOR_ID);
        this.intakeDrive.setInverted(false);
        this.intakeDrive.setNeutralMode(NeutralMode.Brake);
    }

    public void setMotors(double input){
        this.intakeDrive.set(ControlMode.PercentOutput, input);
    }

    public void stopMotors(){
        this.intakeDrive.set(ControlMode.PercentOutput, 0);
    }

    public void setIntakeMotors(double input){
        this.intakeDrive.set(ControlMode.PercentOutput, input);
    }   
    
    
}
