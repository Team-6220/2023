package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDefaultCommand extends CommandBase{
    private final Supplier<Boolean> LButton, LTrigger, RButton, RTrigger;
    private final IntakeSubsystem intakeSubsystem;
    public IntakeDefaultCommand(IntakeSubsystem intakeSubsystem, Supplier<Boolean> LTrigger, Supplier<Boolean> LButton, Supplier<Boolean> RTrigger, Supplier<Boolean> RButton ){
        this.LButton = LButton;
        this.RButton = RButton;
        this.LTrigger  = LTrigger;
        this.RTrigger = RTrigger;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        // if(!intakeSubsystem.getSolenoidState()){
        //     if(RButton.get() && !RTrigger.get()){
        //         intakeSubsystem.setMotors(IntakeConstants.k_CUBE_INTAKE_SPEED);
        //     }else if(RTrigger.get() && !RButton.get()){
        //         intakeSubsystem.setMotors(IntakeConstants.k_CUBE_OUTTAKE_SPEED);
        //     }else if(RTrigger.get() && RButton.get()){
        //         intakeSubsystem.setMotors(-.8);
        //     }else if(!RTrigger.get() && !RTrigger.get()){
        //         intakeSubsystem.setMotors(.1);
        //     } 
        // }else{
        //     if(RButton.get() && !RTrigger.get()){
        //         intakeSubsystem.setMotors(IntakeConstants.k_CONE_INTAKE_SPEED);
        //     }else if(RTrigger.get() && !RButton.get()){
        //         intakeSubsystem.setMotors(IntakeConstants.k_CONE_OUTTAKE_SPEED);
        //     }
        //     else{
        //         intakeSubsystem.setMotors(0);
        //     }
        // }
        if(RButton.get()){
            if(RTrigger.get()){
                intakeSubsystem.setMotors(-1);
            }
            if(!intakeSubsystem.getSolenoidState()){
                intakeSubsystem.setMotors(IntakeConstants.k_CUBE_INTAKE_SPEED);
            }else{
                intakeSubsystem.setMotors(IntakeConstants.k_CONE_INTAKE_SPEED);
            }
        }else if(RTrigger.get()){
            if(!intakeSubsystem.getSolenoidState()){
                intakeSubsystem.setMotors(IntakeConstants.k_CUBE_OUTTAKE_SPEED);
            }else{
                intakeSubsystem.setMotors(IntakeConstants.k_CONE_OUTTAKE_SPEED);
            }
        }else{
            if(!intakeSubsystem.getSolenoidState()){
                intakeSubsystem.setMotors(.2);
            }else{
                intakeSubsystem.setMotors(0);
            }
        }
        if(LButton.get()){
            intakeSubsystem.toggleSolenoid();
        }
        if(LTrigger.get()){
            intakeSubsystem.toggleCompressor();
        }
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
    }
}
