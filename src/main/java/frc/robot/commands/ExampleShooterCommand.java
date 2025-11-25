package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

//Example usage of Shooter Subsystem, not in Robot Container

public class ExampleShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private DoubleSupplier shooterRPM;

    public ExampleShooterCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier shooterRPM) {
        this.shooterSubsystem = shooterSubsystem;
        this.shooterRPM = shooterRPM;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setMotorRPM(shooterRPM.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setMotorRPM(0);
    }

}