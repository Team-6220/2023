package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class ATWPositionCmd extends ParallelCommandGroup{
    public ATWPositionCmd(ArmSubsystem armSubsystem, TelescopeSubsystem telescopeSubsystem, double[] positions){
        addRequirements(armSubsystem, telescopeSubsystem);
        addCommands(
            new SequentialCommandGroup(new ArmPositionCmd(armSubsystem, positions[0]), new ArmHoldCmd(armSubsystem)),
            new SequentialCommandGroup(new TelescopePositionCmd(), new TelescopeHoldCmd())
        );
    }
}
