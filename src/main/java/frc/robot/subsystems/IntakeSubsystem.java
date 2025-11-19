package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkFlex intakeRollerMotor = new SparkFlex(IntakeConstants.intakeRollerMotorId, SparkFlex.MotorType.kBrushless);

    private final DigitalInput intakeLinebreak = new DigitalInput(0);

    public IntakeSubsystem() {
        intakeRollerMotor.setInverted(false);
    }

    public void setSpeed(double speed) {
        intakeRollerMotor.set(speed);
    }

    public void setVoltage(double voltage) {
        intakeRollerMotor.setVoltage(voltage);
    }

    public boolean getLinebreak() {
        return !intakeLinebreak.get();
    }

    @Override
    public void periodic() {

    }
}
