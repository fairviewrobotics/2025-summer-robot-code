package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.MathUtils;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex topShooterMotor = new SparkFlex(ShooterConstants.TOP_SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex bottomShooterMotor = new SparkFlex(ShooterConstants.BOTTOM_SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    public ShooterSubsystem() {
        Preferences.initDouble("SHOOTER_P", ShooterConstants.SHOOTER_P);
        Preferences.initDouble("SHOOTER_I", ShooterConstants.SHOOTER_I);
        Preferences.initDouble("SHOOTER_D", ShooterConstants.SHOOTER_D);
        Preferences.initDouble("SHOOTER_KS", ShooterConstants.SHOOTER_KS);
        Preferences.initDouble("SHOOTER_KV", ShooterConstants.SHOOTER_KV);
        Preferences.initDouble("SHOOTER_KA", ShooterConstants.SHOOTER_KA);
        Preferences.initDouble("SHOOTER_RPM", 1000.0);
    }

    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(
        Preferences.getDouble("SHOOTER_KS", ShooterConstants.SHOOTER_KS),
        Preferences.getDouble("SHOOTER_KV", ShooterConstants.SHOOTER_KV),
        Preferences.getDouble("SHOOTER_KA", ShooterConstants.SHOOTER_KA)
    );

    private final PIDController shooterPID = new PIDController(
            ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I, ShooterConstants.SHOOTER_D
    );

    private final DoubleEntry shooterSetPoint = NetworkTableInstance.getDefault().getTable("Shooter").getDoubleTopic("shooter set point").getEntry(shooterPID.getSetpoint());
    private final DoubleEntry shooterError = NetworkTableInstance.getDefault().getTable("Shooter").getDoubleTopic("shooter error").getEntry(shooterPID.getError());


    public void setMotorRPM(double rpm) {

        topShooterMotor.setVoltage(
                shooterPID.calculate(MathUtils.RPMtoRadians(topShooterMotor.getEncoder().getVelocity()), MathUtils.RPMtoRadians(rpm)) + 
                shooterFF.calculate(MathUtils.RPMtoRadians(rpm))
        );

        bottomShooterMotor.setVoltage(
                shooterPID.calculate(MathUtils.RPMtoRadians(bottomShooterMotor.getEncoder().getVelocity()), MathUtils.RPMtoRadians(rpm)) + 
                shooterFF.calculate(MathUtils.RPMtoRadians(rpm))
        );

        // indexerMotor.set(rpm/2);
    }

    public void stopMotors() {
        topShooterMotor.setVoltage(0);
        bottomShooterMotor.setVoltage(0);
    }

    public void runVolts(double volts) {
        topShooterMotor.setVoltage(volts);
        bottomShooterMotor.setVoltage(volts);
    }

    public void periodic() {

        shooterPID.setP(Preferences.getDouble("SHOOTER_P", ShooterConstants.SHOOTER_P));
        shooterPID.setI(Preferences.getDouble("SHOOTER_I", ShooterConstants.SHOOTER_I));
        shooterPID.setD(Preferences.getDouble("SHOOTER_D", ShooterConstants.SHOOTER_D));

        shooterFF.setKs(Preferences.getDouble("SHOOTER_KS", ShooterConstants.SHOOTER_KS));
        shooterFF.setKa(Preferences.getDouble("SHOOTER_KA", ShooterConstants.SHOOTER_KA));
        shooterFF.setKv(Preferences.getDouble("SHOOTER_KV", ShooterConstants.SHOOTER_KV));

    }

    public void resetPID() {
        shooterPID.reset();
    }
}