package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator.ElevatorPosition;
import frc.robot.Constants.Intake.IntakeMode;

import static frc.robot.Constants.Intake.*;

import java.util.function.Supplier;

@LoggedObject
public class Intake extends SubsystemBase implements BaseIntake {
    @Log
    private final SparkMax intakeTop_motor;
    @Log
    private final SparkMax intakeBottom_motor;

    SparkMaxConfig config = new SparkMaxConfig();

    private boolean rollersAllowed = false;

    public Intake() {
        config.closedLoop
            .p(0.05)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        intakeTop_motor = createMotor(INTAKETOP_MOTOR_INVERTED, 40, INTAKETOP_MOTOR_ID);
        intakeBottom_motor = createMotor(INTAKEBOTTOM_MOTOR_INVERTED, 40, INTAKEBOTTOM_MOTOR_ID);
    }

    private void setIntakeVoltage(double voltage) {
        intakeTop_motor.setVoltage(voltage);
        intakeBottom_motor.setVoltage(voltage);
    }

    public Command shootAlgae() {
        return Commands.startEnd(
                () -> setIntakeVoltage(IntakeMode.SHOOT.value),
                () -> setIntakeVoltage(IntakeMode.OFF.value))
                .withName("intake.shootAlgae");
    }

    public Command intakeAlgae() {
        return Commands.startEnd(
                () -> setIntakeVoltage(IntakeMode.INTAKE.value),
                () -> setIntakeVoltage(IntakeMode.HOLD.value))
                .until(() -> intakeTop_motor.getOutputCurrent() > INTAKE_CURRENT_SHUTOFF)
                .withName("intake.intakeAlgae");
    }

    public Command shootAlgaeTop() {
        return Commands.startEnd(
                () -> {intakeTop_motor.setVoltage(IntakeMode.SHOOT.value);
                        intakeBottom_motor.setVoltage(IntakeMode.OFF.value);},
                () -> setIntakeVoltage(IntakeMode.OFF.value))
                .withName("intake.shootAlgae");
    }

    public Command shootAuto(Supplier<ElevatorPosition> supp) {
        var position = supp.get();
        switch (position) {
            case ALGAE:
            case PROCESSOR:
                return shootAlgae().withTimeout(0.5);
            case L1:
            case L2:
            case L3:
            case L4:
                return shootAlgae().withTimeout(0.5);
            default:
                return Commands.none();
        }
    }

    private SparkMax createMotor(Boolean inverted, Integer currentLimit, Integer devID) {
        var motorConfig = new SparkMaxConfig();
        motorConfig
                .inverted(inverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(currentLimit);

        var motor = new SparkMax(devID, MotorType.kBrushless);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return motor;
    }

    @Override
    public Command runRollersCommand() {
        return intakeAlgae();
    }

    @Override
    public Command reverseRollersCommand() {
        return shootAlgae();
    }
    
}
