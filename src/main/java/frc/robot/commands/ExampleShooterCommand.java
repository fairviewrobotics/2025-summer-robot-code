package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

//Example usage of Shooter Subsystem, not in Robot Container

public class ExampleShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private double shooterRPM;

    public ExampleShooterCommand(ShooterSubsystem shooterSubsystem, double shooterRPM) {
        this.shooterSubsystem = shooterSubsystem;
        this.shooterRPM = shooterRPM;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setMotorRPM(shooterRPM);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setMotorRPM(0);
    }
}