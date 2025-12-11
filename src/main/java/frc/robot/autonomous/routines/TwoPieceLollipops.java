package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DriveToPointCheesyPoofs;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Set;

public class TwoPieceLollipops extends SequentialCommandGroup {

    public TwoPieceLollipops(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        setName("TWO_PIECE_LOLLIPOPS");
        addRequirements(swerveSubsystem, armSubsystem, intakeSubsystem, shooterSubsystem);

        // idk if ts actually has to be defer() or not
        // Shooter can prob be spun up the entire time

        addCommands(
                new SequentialCommandGroup(

                        // 1. Intake 1
                        Commands.parallel(
                                new IntakeCommand(intakeSubsystem, shooterSubsystem, IntakeConstants.INTAKING_VOLTAGE),
                                new ArmCommand(armSubsystem, shooterSubsystem, ArmConstants.INTAKE_POSITION),
                                Commands.defer(() ->
                                    new DriveToPointCheesyPoofs(swerveSubsystem, swerveSubsystem.getPose(), FieldConstants.ALGAE_1_LOLLIPOP_POINT, 0.7), Set.of(swerveSubsystem))
                        ).until(shooterSubsystem::getLinebreak),

                        // 2. Drive to Shoot 1
                        Commands.defer(() ->
                            new DriveToPointCheesyPoofs(swerveSubsystem, swerveSubsystem.getPose(), FieldConstants.SHOOT_BARGE_POINT, 0.7), Set.of(swerveSubsystem)),

                        // 3. Shoot 1
                        Commands.sequence(
                                new ShooterCommand(shooterSubsystem, ShooterConstants.AUTO_TOP_SHOOTER_RPM, ShooterConstants.AUTO_BOTTOM_SHOOTER_RPM).withTimeout(ShooterConstants.AUTO_SHOOTER_TIMEOUT_SECONDS),
                                new IntakeCommand(intakeSubsystem, shooterSubsystem, IntakeConstants.INTAKING_VOLTAGE).withTimeout(IntakeConstants.INTAKING_TIMEOUT_SECONDS)
                        ),

                        // 4. Intake 2
                        Commands.parallel(
                                new IntakeCommand(intakeSubsystem, shooterSubsystem, IntakeConstants.INTAKING_VOLTAGE),
                                new ArmCommand(armSubsystem, shooterSubsystem, ArmConstants.INTAKE_POSITION),
                                Commands.defer(() ->
                                        new DriveToPointCheesyPoofs(swerveSubsystem, swerveSubsystem.getPose(), FieldConstants.ALGAE_2_LOLLIPOP_POINT, 0.7), Set.of(swerveSubsystem))
                        ).until(shooterSubsystem::getLinebreak),

                        // 5. Drive to Shoot 2
                        Commands.defer(() ->
                                new DriveToPointCheesyPoofs(swerveSubsystem, swerveSubsystem.getPose(), FieldConstants.SHOOT_BARGE_POINT, 0.7), Set.of(swerveSubsystem)),

                        // 6. Outtake 2
                        Commands.sequence(
                                new ShooterCommand(shooterSubsystem, ShooterConstants.AUTO_TOP_SHOOTER_RPM, ShooterConstants.AUTO_BOTTOM_SHOOTER_RPM).withTimeout(ShooterConstants.AUTO_SHOOTER_TIMEOUT_SECONDS),
                                new IntakeCommand(intakeSubsystem, shooterSubsystem, IntakeConstants.INTAKING_VOLTAGE).withTimeout(IntakeConstants.INTAKING_TIMEOUT_SECONDS)
                        )
                )
        );

    }

}
