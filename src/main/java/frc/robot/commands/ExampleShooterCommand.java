package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

//Example usage of Shooter Subsystem, not in Robot Container

public class ExampleShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private DoubleSupplier topShooterRPM;
    private DoubleSupplier bottomShooterRPM;

    public ExampleShooterCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier topShooterRPM, DoubleSupplier bottomShooterRPM) {
        this.shooterSubsystem = shooterSubsystem;
        this.topShooterRPM = topShooterRPM;
        this.bottomShooterRPM = bottomShooterRPM;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setTopShooterMotor(topShooterRPM.getAsDouble());
        shooterSubsystem.setBottomShooterMotor(bottomShooterRPM.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setMotorRPM(0);
    }

}