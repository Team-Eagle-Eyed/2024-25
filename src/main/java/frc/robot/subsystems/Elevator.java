package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlib.subsystems.BaseLinearMechanism;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.ElevatorPosition;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.Elevator.*;

@LoggedObject
public class Elevator extends SubsystemBase implements BaseLinearMechanism<ElevatorPosition> {
    @Log
    private final SparkMax motor;

    private SparkMaxConfig motorConfig;

    @Log(groups = "control")
    private final ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);

    @Log(groups = "control")
    private final ElevatorFeedforward feedforwardController = new ElevatorFeedforward(kS,
            kG, kV, kA);

    @Log
    private ElevatorPosition goal = frc.robot.Constants.Elevator.ElevatorPosition.BOTTOM;

    /**
     * The representation of the "elevator" for simulation. (even though this is a
     * rotational mechanism w.r.t. its setpoints, we still control it as a linear
     * mechanism since that is the cloest physical mechanism to this)
     */
    private final ElevatorSim elevatorSim = new ElevatorSim(
            MOTOR_GEARBOX_REPR,
            GEARING,
            MASS_KG,
            DRUM_RADIUS_METERS,
            MIN_HEIGHT_METERS,
            MAX_HEIGHT_METERS,
            true,
            ElevatorPosition.BOTTOM.value);

    @Log(groups = "control")
    private double feedbackVoltage = 0;
    @Log(groups = "control")
    private double feedforwardVoltage = 0;

    private double simVelocity = 0.0;

    private final MutVoltage sysidAppliedVoltageMeasure = Volts.mutable(0);
    private final MutDistance sysidPositionMeasure = Meters.mutable(0);
    private final MutLinearVelocity sysidVelocityMeasure = MetersPerSecond.mutable(0);

    private final SysIdRoutine sysIdRoutine;

    @Log
    private boolean initialized;

    public Elevator() {
        motorConfig = new SparkMaxConfig();

        motorConfig
                .inverted(MOTOR_INVERTED)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(CURRENT_LIMIT);
        motorConfig.encoder
                .positionConversionFactor(ENCODER_ROTATIONS_TO_METERS)
                .velocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0);

        motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(1).per(Seconds), Volts.of(5), null, null),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> setVoltage(volts.magnitude()),
                        log -> {
                            log.motor("primary")
                                    .voltage(sysidAppliedVoltageMeasure.mut_replace(motor.getAppliedOutput(), Volts))
                                    .linearPosition(sysidPositionMeasure.mut_replace(getPosition(), Meters))
                                    .linearVelocity(sysidVelocityMeasure.mut_replace(getVelocity(), MetersPerSecond));
                        },
                        this));

        setDefaultCommand(moveToCurrentGoalCommand());
        pidController.setTolerance(0.03
        );
    }

    @Override
    public void simulationPeriodic() {
        elevatorSim.setInput(motor.getAppliedOutput());
        elevatorSim.update(0.020);
        motor.getEncoder().setPosition(elevatorSim.getPositionMeters());
        simVelocity = elevatorSim.getVelocityMetersPerSecond();
    }


    public boolean getInitialized() {
        return initialized; //reset the encoder
    }

    @Override
    public double getPosition() {
        SmartDashboard.putNumber("ElevatorPosition", motor.getEncoder().getPosition());
        return motor.getEncoder().getPosition();
    }

    public ElevatorPosition getGoal() {
        return goal;
    }

    public double getVelocity() {
        if (RobotBase.isReal())
            return motor.getEncoder().getVelocity();
        else
            return simVelocity;
    }

    @Override
    public void resetPosition() {
        motor.getEncoder().setPosition(ElevatorPosition.BOTTOM.value);
        initialized = true;
    }

    public double retEncoder() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        voltage = Utils.applySoftStops(voltage, getPosition(), MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        if (voltage < 0 && getPosition() < Constants.Elevator.MOTION_LIMIT) {
            voltage = 0;
        }

        motor.setVoltage(voltage);

    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            feedbackVoltage = pidController.calculate(getPosition());
            feedforwardVoltage = feedforwardController.calculate(pidController.getSetpoint().velocity);
            setVoltage(feedbackVoltage + feedforwardVoltage);
        }).withName("elevator.moveToCurrentGoal");
    }

    @Override
    public Command moveToPositionCommand(Supplier<ElevatorPosition> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> goal = goalPositionSupplier.get()),
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get().value)),
                moveToCurrentGoalCommand()
                        .until(() -> pidController.atGoal()))
                .withTimeout(3)
                .withName("elevator.moveToPosition");
                

                
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
                moveToCurrentGoalCommand().until(this::atGoal)).withName("elevator.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> pidController.getGoal().position + delta.get())
                .withName("elevator.movePositionDelta");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> pidController.setGoal(getPosition())).andThen(moveToCurrentGoalCommand())
                .withName("elevator.holdCurrentPosition");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("elevator.resetPosition");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("elevator.setOverriddenSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(motor::stopMotor)
                .andThen(() -> {
                    motorConfig.idleMode(IdleMode.kCoast);
                    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                })
                .finallyDo((d) -> {
                    motorConfig.idleMode(IdleMode.kBrake);
                    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                    pidController.reset(getPosition());
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("elevator.coastMotorsCommand");
    }

    public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("elevator.sysIdQuasistatic");
    }

    public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("elevator.sysIdDynamic");
    }

    public Command resetControllersCommand() {
        return Commands.runOnce(() -> pidController.reset(getPosition()))
                .andThen(Commands.runOnce(() -> pidController.setGoal(getPosition())));
    }

    public boolean atGoal() {
        return pidController.atGoal();
    }
    public double drivetrainModifier(){
        double Elevatorlimiter = 1.0d;
        Elevatorlimiter = Math.abs(1-getPosition() *0.25);
        return Elevatorlimiter;
    }
}