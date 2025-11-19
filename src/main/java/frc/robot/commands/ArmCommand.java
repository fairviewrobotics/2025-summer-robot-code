package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final double position;

    public ArmCommand(ArmSubsystem armSubsystem, double position) {
        this.armSubsystem = armSubsystem;
        this.position = position;
        this.armSubsystem.resetPID();
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.resetPID();
    }

    @Override
    public void execute() {
        this.armSubsystem.setArmAngle(position);
    }

    @Override
    public void end(boolean interrupted) {
        this.armSubsystem.setVoltage(0);
    }
}
