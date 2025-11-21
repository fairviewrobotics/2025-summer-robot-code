package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double intakeVoltage;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double intakeVoltage) {
        this.intakeSubsystem = intakeSubsystem;
        this.intakeVoltage = intakeVoltage;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intakeSubsystem.setVoltage(intakeVoltage);
        intakeSubsystem.setIndexerVoltage(intakeVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setVoltage(0);
        intakeSubsystem.setIndexerVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.getLinebreak(); // change to shooter linebreak
    }
}
