package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final SparkFlex armMotor = new SparkFlex(ArmConstants.armMotorID, SparkFlex.MotorType.kBrushless);

    private final AbsoluteEncoder armAbsEncoder = armMotor.getAbsoluteEncoder();

    private final ProfiledPIDController armPID = new ProfiledPIDController(ArmConstants.armP, ArmConstants.armI, ArmConstants.armD, ArmConstants.armConstraints);

    public ArmSubsystem() {
        armPID.setTolerance(ArmConstants.armTolerance);
        armPID.enableContinuousInput(-Math.PI, Math.PI);
        SparkFlexConfig armConfig = new SparkFlexConfig();
        armConfig.inverted(false);
        armConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        armConfig
                .absoluteEncoder
                .inverted(false)
                .positionConversionFactor(2 * Math.PI) // radians
                .velocityConversionFactor(2 * Math.PI / 60.0);
        armMotor.configure(
                armConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
        Preferences.initDouble("ARM_P", ArmConstants.armP);
        Preferences.initDouble("ARM_D", ArmConstants.armD);
        Preferences.initDouble("ARM_TOLERANCE", ArmConstants.armTolerance);
        Preferences.initDouble("ARM_MAX_VELOCITY", ArmConstants.armMaxVelocity);
        Preferences.initDouble("ARM_MAX_ACCELERATION", ArmConstants.armMaxAcceleration);
        Preferences.initDouble("ARM_SETPOINT", 0.0);
    }

    public void setVoltage(double volts) {
        armMotor.setVoltage(volts);
    }

    public boolean atTargetAngle() {
        return armPID.atSetpoint();
    }

    public void setSpeed(double speed) {
        armMotor.set(speed);
    }

    public void setArmAngle(double angle) {
        angle = MathUtil.clamp(angle, ArmConstants.armMaxAngle, ArmConstants.armMinAngle);

        double pidValue = armPID.calculate(getArmAngle(), angle);

        setVoltage(pidValue);
    }

    public double getArmAngle() {
        double x = Math.PI * 2 - armAbsEncoder.getPosition();
        if (x >= Math.PI) x -= Math.PI * 2;
        return x;
    }

    public double getArmSpeed() {
        return armAbsEncoder.getVelocity();
    }

    public void updateTuningValues() {
        armPID.setP(Preferences.getDouble("ARM_P", ArmConstants.armP));
        armPID.setD(Preferences.getDouble("ARM_D", ArmConstants.armD));
        armPID.setTolerance(
                Math.toRadians(Preferences.getDouble("ARM_TOLERANCE", ArmConstants.armTolerance)));
        armPID.setConstraints(
                new TrapezoidProfile.Constraints(
                        Preferences.getDouble("ARM_MAX_VELOCITY", ArmConstants.armMaxVelocity),
                        Preferences.getDouble("ARM_MAX_ACCELERATION", ArmConstants.armMaxAcceleration)));
    }
    public void periodic() {
        updateTuningValues();
    }

    public void resetPID() {
        armPID.reset(getArmAngle(), getArmSpeed());
    }

    public void resetEncoder() {
        armMotor.getEncoder().setPosition(0.0);
    }
}