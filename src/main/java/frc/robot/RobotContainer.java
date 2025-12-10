// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DriveToPointCheesyPoofs;
import frc.robot.commands.ExampleShooterCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.*;

import java.io.File;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.utils.NetworkTablesUtils;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandPS5Controller primary_controller = new CommandPS5Controller(0);
  final CommandXboxController secondary_controller = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
          "swerve"));
  ArmSubsystem armSubsystem = new ArmSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  NetworkTablesUtils NTAuto = NetworkTablesUtils.getTable("Autonomous");

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                  () -> primary_controller.getLeftY() * -1,
                  () -> primary_controller.getLeftX() * -1)
          .withControllerRotationAxis(() -> primary_controller.getRightX() * -1)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  SwerveInputStream driveYAxisLock = SwerveInputStream.of(drivebase.getSwerveDrive(),
                  () -> primary_controller.getLeftY() * 0,
                  () -> primary_controller.getLeftX() * -1)
          .withControllerRotationAxis(() -> primary_controller.getRightX() * 0)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(primary_controller::getRightX,
                  primary_controller::getRightY)
          .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
          .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                  () -> -primary_controller.getLeftY(),
                  () -> -primary_controller.getLeftX())
          .withControllerRotationAxis(() -> primary_controller.getRawAxis(
                  2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
          .withControllerHeadingAxis(() ->
                          Math.sin(
                                  primary_controller.getRawAxis(
                                          2) *
                                          Math.PI) *
                                  (Math.PI *
                                          2),
                  () ->
                          Math.cos(
                                  primary_controller.getRawAxis(
                                          2) *
                                          Math.PI) *
                                  (Math.PI *
                                          2))
          .headingWhile(true)
          .translationHeadingOffset(true)
          .translationHeadingOffset(Rotation2d.fromDegrees(
                  0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedYAxisLock = drivebase.driveFieldOriented(driveYAxisLock);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
            driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
            driveDirectAngleKeyboard);
    primary_controller.options().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    primary_controller.pov(0).whileTrue(drivebase.sysIdDriveMotorCommand());
    primary_controller.pov(90).whileTrue(drivebase.sysIdAngleMotorCommand());
    primary_controller.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
            () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
    Supplier<Double> armSetpointSupplier = () -> Preferences.getDouble("ARM_SETPOINT", 0.0);
    DoubleSupplier shooterSetpointSupplier = () -> Preferences.getDouble("SHOOTER_RPM", 0.0);
    DoubleSupplier bottomShooterSetpointSupplier = () -> Preferences.getDouble("SHOOTER_BOTTOM", 0.0);
    secondary_controller.leftBumper().whileTrue(new ParallelCommandGroup(new ArmCommand(armSubsystem, shooterSubsystem, 0.4), new IntakeCommand(intakeSubsystem, shooterSubsystem, 2.0)));
    secondary_controller.b().whileTrue(new ArmCommand(armSubsystem, shooterSubsystem, 0.4));
    secondary_controller.x().whileTrue(new IntakeCommand(intakeSubsystem, shooterSubsystem, -4));
    secondary_controller.rightBumper().whileTrue(new ExampleShooterCommand(shooterSubsystem, shooterSetpointSupplier, bottomShooterSetpointSupplier));
    secondary_controller.a().whileTrue(new IntakeCommand(intakeSubsystem, shooterSubsystem, 4.0));

    armSubsystem.setDefaultCommand(new ArmCommand(armSubsystem, shooterSubsystem, 1.3));
    primary_controller.R1().whileTrue(driveFieldOrientedYAxisLock);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
              Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
              new ProfiledPIDController(5,
                      0,
                      0,
                      new Constraints(5, 2)),
              new ProfiledPIDController(5,
                      0,
                      0,
                      new Constraints(Units.degreesToRadians(360),
                              Units.degreesToRadians(180))
              ));


//      driverXbox.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      primary_controller.square().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      primary_controller.triangle().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      primary_controller.options().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // primary_controller.back().whileTrue(drivebase.centerModulesCommand());
      primary_controller.L1().onTrue(Commands.none());
      primary_controller.R1().onTrue(Commands.none());
    } else
    {
      primary_controller.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      primary_controller.square().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      primary_controller.options().whileTrue(Commands.none());
      // primary_controller.back().whileTrue(Commands.none());
      primary_controller.L1().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      primary_controller.R1().onTrue(Commands.none());
    }

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  private final Supplier<Rotation2d> rotationSupplier = () -> Rotation2d.fromDegrees(Preferences.getDouble("TARGET_ROTATION", 0.0));

  private final Supplier<Pose2d> targetPoseSupplier = () -> new Pose2d(
  Preferences.getDouble("TARGET_POSE_X", 0.0),
  Preferences.getDouble("TARGET_POSE_Y", 0.0),
  rotationSupplier.get()
);
  private final double tolerance = Preferences.getDouble("TOLERANCE",0.1);
  private final double MAX_SPEED = Preferences.getDouble("MAX_SPEED", 1.0);
  // Pose2d origin = new Pose2d(Preferences.getDouble("TARGET_POSE_X", 0.0), Preferences.getDouble("TARGET_POSE_Y", 0.0), new Rotation2d(Preferences.getDouble("TARGET_ROTATION", 0.0)));
  Translation2d zero = new Translation2d(0 , 0);

  public Command getAutonomousCommand()
  {
        // return new RunCommand(() -> drivebase.drive(origin2, 0.0, true));
        //return new RunCommand(() -> drivebase.driveToPointVectorBased(origin, 1.5, 1.0, 0.1, true));
//        Command driveToTarget =
//        new RunCommand(
//            () -> drivebase.driveToPointVectorBased(targetPoseSupplier.get(), 0.05, 4.0, 180, false),
//            drivebase
//        ).until(() -> drivebase.driveToPointVectorBased(targetPoseSupplier.get(), 0.05, 4.0, 180, false));
//
//    return new SequentialCommandGroup(
//        driveToTarget,
//        new InstantCommand(() -> drivebase.drive(zero, 0.0, true)),
//        new WaitCommand(5.0)
//    );
    Pose2d algae1Point = new Pose2d(1.24, 3.64, new Rotation2d(2.74));
    Pose2d algae2Point = new Pose2d(1.33, 2.79, new Rotation2d(-2.443));
    Pose2d shootPoint = new Pose2d(3, 3, new Rotation2d(0));

    Command driveToShoot = Commands.defer(() ->
            new DriveToPointCheesyPoofs(drivebase, drivebase.getPose(), shootPoint, 1.0), Set.of(drivebase));
    Command driveToAlgae1 = Commands.defer(() ->
            new DriveToPointCheesyPoofs(drivebase, drivebase.getPose(), algae1Point, 1.0), Set.of(drivebase));

    Command driveToAlgae2 = Commands.defer(() ->
            new DriveToPointCheesyPoofs(drivebase, drivebase.getPose(), algae2Point, 1.0), Set.of(drivebase));


//    Command spinUpShooter = new ExampleShooterCommand(shooterSubsystem, () -> 3000, () -> 3000);
//    Command outtake = new SequentialCommandGroup(new ArmCommand(armSubsystem, shooterSubsystem, 1.3), new IntakeCommand(intakeSubsystem, shooterSubsystem, -4.0));

    Command intake = new ParallelCommandGroup(new IntakeCommand(intakeSubsystem, shooterSubsystem, 4.0), new ArmCommand(armSubsystem, shooterSubsystem, 0.4));
//    Command AlgaeIntake =
//            new ParallelCommandGroup(intake, driveToAlgae1).
//            until(() -> shooterSubsystem.getLinebreak()).andThen(driveToShoot).
//            andThen(outtake).
//            andThen(new ParallelCommandGroup(intake, driveToAlgae2).
//            andThen(driveToShoot).andThen(outtake));


//    Command GoodAlgaeIntake =
//            new SequentialCommandGroup(
//                new ParallelCommandGroup(new ParallelCommandGroup(new IntakeCommand(intakeSubsystem, shooterSubsystem, 4.0), new ArmCommand(armSubsystem, shooterSubsystem, 0.4)), new DriveToPointCheesyPoofs(drivebase, drivebase.getPose(), algae1Point, 0.7)).until(() -> shooterSubsystem.getLinebreak()),
//                new DriveToPointCheesyPoofs(drivebase, drivebase.getPose(), shootPoint, 0.7),
//                new SequentialCommandGroup(new ArmCommand(armSubsystem, shooterSubsystem, 1.3), new IntakeCommand(intakeSubsystem, shooterSubsystem, -4.0)),
//                new ParallelCommandGroup(intake, driveToAlgae2).until(() -> shooterSubsystem.getLinebreak()),
//                new DriveToPointCheesyPoofs(drivebase, drivebase.getPose(), shootPoint, 0.7),
//                new SequentialCommandGroup(new ArmCommand(armSubsystem, shooterSubsystem, 1.3), new IntakeCommand(intakeSubsystem, shooterSubsystem, -4.0))
//            );

    Command GooderAlgaeIntake =
            new SequentialCommandGroup(
                    // 1. Intake 1
                    Commands.parallel(
                            new IntakeCommand(intakeSubsystem, shooterSubsystem, 4.0),
                            new ArmCommand(armSubsystem, shooterSubsystem, 0.4),
                            driveToAlgae1
                    ).until(() -> shooterSubsystem.getLinebreak()),

                    // 2. Drive to Shoot 1
                    driveToShoot, // First use

                    // 3. Shoot 1
                    Commands.sequence(
                            new ExampleShooterCommand(shooterSubsystem, () -> 4000, () -> 4000).withTimeout(2.0),
                            new IntakeCommand(intakeSubsystem, shooterSubsystem, 4.0).withTimeout(2.0)
                    ),

                    // 4. Intake 2
                    Commands.parallel(
                            new IntakeCommand(intakeSubsystem, shooterSubsystem, 4.0),
                            new ArmCommand(armSubsystem, shooterSubsystem, 0.4),
                            driveToAlgae2
                    ).until(() -> shooterSubsystem.getLinebreak()),

                    // 5. Drive to Shoot 2
                    // Create a NEW instance instead of reusing driveToShoot
                    new DriveToPointCheesyPoofs(drivebase, drivebase.getPose(), shootPoint, 0.7),

                    // 6. Outtake 2
                    Commands.sequence(
                            new ExampleShooterCommand(shooterSubsystem, () -> 4000, () -> 4000).withTimeout(2.0),
                            new IntakeCommand(intakeSubsystem, shooterSubsystem, 4.0).withTimeout(2.0)
                    )
            );

    Command AlgaeIntakeSimple =
            new SequentialCommandGroup(
                    new DriveToPointCheesyPoofs(drivebase, drivebase.getPose(), algae1Point, 0.7),
                    new DriveToPointCheesyPoofs(drivebase, drivebase.getPose(), shootPoint, 0.7)
    );
    return GooderAlgaeIntake;

    // An example command will be run in autonomous
//    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }




}