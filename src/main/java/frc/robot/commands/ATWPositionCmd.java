package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ATWSubsystem;

public class ATWPositionCmd extends CommandBase{
    private final ATWSubsystem atwSubsystem;
    private final double[] positions;
    private final PIDController armPidController, telePidController;
    public ATWPositionCmd(ATWSubsystem atwSubsystem, double[] positions){
        this.atwSubsystem = atwSubsystem;
        this.positions = positions;
        this.telePidController = new PIDController(0.0005, 0, 0);
        this.armPidController = new PIDController(0.005, 0, 0);
        addRequirements(atwSubsystem);
    }
    @Override
    public void execute() {
        double armOutput = armPidController.calculate(atwSubsystem.getArmPositionDegrees(), positions[0]);
        armOutput *= .5;
        this.atwSubsystem.setArmMotors(armOutput);
        double telescopeOutput = telePidController.calculate(atwSubsystem.getTelescopePosition(), positions[1]);
        telescopeOutput *= .5;
        this.atwSubsystem.setTeleMotors(telescopeOutput);
        
    }
}
