package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.ConfigManager;
import frc.robot.utils.MathUtils;

public class ShooterSubsystem extends SubsystemBase {
    private final ConfigManager config = ConfigManager.getInstance();

    private final SparkFlex topShooterMotor = new SparkFlex(ShooterConstants.topShooterMotorID, SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex bottomShooterMotor = new SparkFlex(ShooterConstants.bottomShooterMotorID, SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex indexerMotor = new SparkFlex(ShooterConstants.indexerMotorID, SparkLowLevel.MotorType.kBrushless);

    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(
            config.get("ShooterFFKs", ShooterConstants.shooterFFKs),
            config.get("ShooterFFKv", ShooterConstants.shooterFFKv),
            config.get("ShooterFFKa", ShooterConstants.shooterFFKa)
    );

    private final PIDController shooterPID = new PIDController(
            ShooterConstants.shooterP, ShooterConstants.shooterI, ShooterConstants.shooterD
    );

    private final DoubleEntry shooterSetPoint = NetworkTableInstance.getDefault().getTable("Shooter").getDoubleTopic("shooter set point").getEntry(shooterPID.getSetpoint());
    private final DoubleEntry shooterError = NetworkTableInstance.getDefault().getTable("Shooter").getDoubleTopic("shooter error").getEntry(shooterPID.getError());

    public ShooterSubsystem() {

    }


    public void setMotorRPM(double rpm) {
        topShooterMotor.setVoltage(
                shooterPID.calculate(MathUtils.RPMtoRadians(topShooterMotor.getEncoder().getVelocity()), MathUtils.RPMtoRadians(rpm)) +
                        shooterPID.calculate(MathUtils.RPMtoRadians(rpm))
        );
        bottomShooterMotor.setVoltage(
                shooterPID.calculate(MathUtils.RPMtoRadians(bottomShooterMotor.getEncoder().getVelocity()), MathUtils.RPMtoRadians(rpm)) +
                        shooterPID.calculate(MathUtils.RPMtoRadians(rpm))
        );
        indexerMotor.set(rpm/2);
    }

    public void runVolts(double volts) {
        topShooterMotor.setVoltage(volts);
        bottomShooterMotor.setVoltage(volts);
    }

    public void periodic() {
        shooterPID.setP(config.get("ShooterP", ShooterConstants.shooterP));
        shooterPID.setI(config.get("ShooterI", ShooterConstants.shooterI));
        shooterPID.setD(config.get("ShooterD", ShooterConstants.shooterD));
    }

    public void resetPID() {
        shooterPID.reset();
    }
}