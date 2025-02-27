package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class Elevator {
        public static enum ElevatorPosition {
            BOTTOM(0.0698),
            PROCESSOR(0.1234),
            L1(0.323),
            L2(0.31),
            L3(0.70),
            L4(1.27),
            ALGAE(1.57);

            public final double value;

            private ElevatorPosition(double value) {
                this.value = value;
            }
        }

        public static final double MOTION_LIMIT = 0.3;

        public static final double SCORING_MOVEMENT = -0.25;

        public static final int MOTOR_ID = 5;
        public static final boolean MOTOR_INVERTED = false;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNEO(1);
        public static final double GEARING = 5.0;
        public static final double MASS_KG = Units.lbsToKilograms(20);
        public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.32) / 2.0; // TODO
        public static final double DRUM_CIRCUMFERENCE = 2.0 * Math.PI * DRUM_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = DRUM_CIRCUMFERENCE / GEARING;

        public static final double MIN_HEIGHT_METERS = 0.005; // TODO
        public static final double MAX_HEIGHT_METERS = 1.57; // TODO

        public static final int CURRENT_LIMIT = 60;

        public static final double kP = 50; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 5; // TODO
        public static final double kS = 0.095388; // TODO
        public static final double kG = 0.54402; // TODO
        public static final double kV = 7.43; // TODO
        public static final double kA = 1.0; // TODO
        public static final double TOLERANCE = 0.02;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 1.3; // TODO
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; // TODO
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }
    

    public static final class Intake {
        public static enum IntakePosition {
            HOME(0.0698),
            OUT(0.55);

            public final double value;

            private IntakePosition(double value) {
                this.value = value;
            }
        }
        public static final int ARML_MOTOR_ID = 13;
        public static final boolean ARML_MOTOR_INVERTED = true;
        public static final int ARML_CURRENT_LIMIT = 60;

        public static final int ARMR_MOTOR_ID = 13;
        public static final boolean ARMR_MOTOR_INVERTED = true;
        public static final int ARMR_CURRENT_LIMIT = 60;

        public static final int INTAKEL_MOTOR_ID = 13;
        public static final boolean INTAKEL_MOTOR_INVERTED = true;
        public static final int INTAKEL_CURRENT_LIMIT = 60;

        public static final int INTAKEC_MOTOR_ID = 13;
        public static final boolean INTAKEC_MOTOR_INVERTED = true;
        public static final int INTAKEC_CURRENT_LIMIT = 60;

        public static final int INTAKER_MOTOR_ID = 13;
        public static final boolean INTAKER_MOTOR_INVERTED = true;
        public static final int INTAKER_CURRENT_LIMIT = 60;
    }

    public static final class Climber {
        public static final int MOTOR_ID = 6;
        public static final boolean MOTOR_INVERTED = false;
        public static final int CURRENT_LIMIT = 60;

        public static final double MIN_POSITION_METERS = 0.0;
        public static final double MAX_POSITION_METERS = 1.0; // TODO

        public static final double GEARING = 64.0;
        public static final double MASS_KG = Units.lbsToKilograms(80); // robot weight
        public static final double SPOOL_RADIUS_METERS = Units.inchesToMeters(0.5);
        public static final double SPOOL_CIRCUMFERENCE = 2.0 * Math.PI * SPOOL_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = SPOOL_CIRCUMFERENCE * GEARING;
    }
}
