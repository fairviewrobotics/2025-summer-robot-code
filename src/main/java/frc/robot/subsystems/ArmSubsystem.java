package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.utils.TunableNumber;
import frc.robot.utils.NetworkTablesUtils;

public class ArmSubsystem extends SubsystemBase {
    private final SparkFlex armMotor = new SparkFlex(ArmConstants.ARM_MOTOR_ID, SparkFlex.MotorType.kBrushless);

    private final AbsoluteEncoder armAbsEncoder = armMotor.getAbsoluteEncoder();

    private final ProfiledPIDController armPID = new ProfiledPIDController(ArmConstants.ARM_P, 0.0, ArmConstants.ARM_D, ArmConstants.ARM_CONSTRAINTS);

    NetworkTablesUtils NTArm = NetworkTablesUtils.getTable("Arm");

    public ArmSubsystem() {
        armPID.setTolerance(ArmConstants.ARM_TOLERANCE);
        armPID.enableContinuousInput(-Math.PI, Math.PI);
        SparkFlexConfig armConfig = new SparkFlexConfig();
        armConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .absoluteEncoder
                .inverted(true)
                .positionConversionFactor(2 * Math.PI) // radians
                .velocityConversionFactor(2 * Math.PI / 60.0);

        armMotor.configure(
                armConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        NTArm.setEntry("RAN?", "FALSE");
    }

    public void setVoltage(double volts) {
        NTArm.setEntry("RUN?", "TRUE");
        armMotor.setVoltage(volts);
    }

    public boolean atTargetAngle() {
        return armPID.atSetpoint();
    }

    public void setSpeed(double speed) {
        armMotor.set(speed);
    }

    public void setArmAngle(double angle) {
        angle = MathUtil.clamp(angle, ArmConstants.ARM_MIN_ANGLE, ArmConstants.ARM_MAX_ANGLE);
        armPID.setGoal(angle);
        double pidValue = armPID.calculate(getArmAngle());
        setSpeed(pidValue);
    }

    public double getArmAngle() {
        double x = Math.PI * 2 - armAbsEncoder.getPosition();
        if (x >= Math.PI) x -= Math.PI * 2;
        return x;
    }

    public void resetArmEncoder() {

    }

    public double getArmSpeed() {
        return armAbsEncoder.getVelocity();
    }

    public void updateTuningValues() {

    }

    public void periodic() {
        updateTuningValues();
        NTArm.setEntry("ARM_POS", getArmAngle());
        NTArm.setEntry("ARM_S", getArmSpeed());
        NTArm.setEntry("ARM_ERROR", armPID.getPositionError());
        NTArm.setEntry("ARM_SETPOINT", armPID.getGoal().position);
        NTArm.setEntry("ARM_VOLTAGE", armMotor.getOutputCurrent());
    }

    public void resetPID() {
        armPID.reset(getArmAngle(), getArmSpeed());
    }

    public void resetEncoder() {
        armMotor.getEncoder().setPosition(0.0);
    }
}