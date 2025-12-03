package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.NetworkTablesUtils;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex topShooterMotor = new SparkFlex(ShooterConstants.TOP_SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex bottomShooterMotor = new SparkFlex(ShooterConstants.BOTTOM_SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final DigitalInput shooterLinebreak = new DigitalInput(1);
    NetworkTablesUtils shooterNT = NetworkTablesUtils.getTable("Shooter");

    public ShooterSubsystem() {
        Preferences.initDouble("SHOOTER_P", ShooterConstants.SHOOTER_P);
        Preferences.initDouble("SHOOTER_I", ShooterConstants.SHOOTER_I);
        Preferences.initDouble("SHOOTER_D", ShooterConstants.SHOOTER_D);
        Preferences.initDouble("SHOOTER_KS", ShooterConstants.SHOOTER_KS);
        Preferences.initDouble("SHOOTER_KV", ShooterConstants.SHOOTER_KV);
        Preferences.initDouble("SHOOTER_KA", ShooterConstants.SHOOTER_KA);
        Preferences.initDouble("SHOOTER_RPM", 1000.0);
        Preferences.initDouble("SHOOTER_BOTTOM", 1000.0);

        SparkFlexConfig topShooterMotorConfig = new SparkFlexConfig();
        SparkFlexConfig bottomShooterMotorConfig = new SparkFlexConfig();

        topShooterMotorConfig.inverted(true);
        topShooterMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);

        bottomShooterMotorConfig.inverted(false);
        bottomShooterMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);

        topShooterMotor.configure(
                topShooterMotorConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        );
        bottomShooterMotor.configure(
                topShooterMotorConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        );
    }

    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(
        Preferences.getDouble("SHOOTER_KS", ShooterConstants.SHOOTER_KS),
        Preferences.getDouble("SHOOTER_KV", ShooterConstants.SHOOTER_KV),
        Preferences.getDouble("SHOOTER_KA", ShooterConstants.SHOOTER_KA)
    );

    private final PIDController shooterPID = new PIDController(
            ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I, ShooterConstants.SHOOTER_D
    );

    public void setTopShooterMotor(double rpm) {
        topShooterMotor.setVoltage(
                shooterPID.calculate(MathUtils.RPMtoRadians(topShooterMotor.getEncoder().getVelocity()), MathUtils.RPMtoRadians(rpm)) +
                        shooterFF.calculate(MathUtils.RPMtoRadians(rpm))
        );
    }

    public void setBottomShooterMotor(double rpm) {
        bottomShooterMotor.setVoltage(
                shooterPID.calculate(MathUtils.RPMtoRadians(bottomShooterMotor.getEncoder().getVelocity()), MathUtils.RPMtoRadians(rpm)) +
                        shooterFF.calculate(MathUtils.RPMtoRadians(rpm))
        );
    }

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

    public boolean getLinebreak() {
        return !shooterLinebreak.get();
    }


    public void periodic() {

        shooterNT.setEntry("shooter error", shooterPID.getError());
        shooterNT.setEntry("shooter setpoint", shooterPID.getSetpoint());
        shooterNT.setEntry("shooter velocity", MathUtils.RPMtoRadians(topShooterMotor.getEncoder().getVelocity()));
        shooterNT.setEntry("shooter linebreak", getLinebreak());


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