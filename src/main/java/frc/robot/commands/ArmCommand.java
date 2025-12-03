package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ArmCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final double position;

    public ArmCommand(ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, double position) {
        this.armSubsystem = armSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.position = position;
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
