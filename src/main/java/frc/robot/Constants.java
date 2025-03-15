package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Constants {
    public static final class Elevator {
        public static enum ElevatorPosition {
            BOTTOM(0),
            PROCESSOR(0.2),
            L1(0.5),
            L2(0.8),
            L3(1.2 ),
            L4(1.4),
            ALGAE(2.35);

            public final double value;

            private ElevatorPosition(double value) {
                this.value = value;
            }
        }
        
        public static final double MOTION_LIMIT = 0;

        public static final double SCORING_MOVEMENT = -0.25;

        public static final int MOTOR_ID = 5;
        public static final boolean MOTOR_INVERTED = true;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNEO(1);
        public static final double GEARING = 9;
        public static final double MASS_KG = Units.lbsToKilograms(5);
        public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.75) / 2.0; // done
        public static final double DRUM_CIRCUMFERENCE = 2.0 * Math.PI * DRUM_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = DRUM_CIRCUMFERENCE / GEARING;

        public static final double MIN_HEIGHT_METERS = 0.0; // done
        public static final double MAX_HEIGHT_METERS = 2.5; //done

        public static final int CURRENT_LIMIT = 35;

        public static final double kP = 2.5; // TODO
        public static final double kI = 0.; // TODO
        public static final double kD = 1; // TODO
        public static final double kS = 0.095388; // TODO
        //public static final double kG = 0.54402; // TODO
        public static final double kG = 0.05; // BLT todo
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
            HOME(0.5),
            OUT(15),
            GRAB(15);

            public final double value;

            private IntakePosition(double value) {
                this.value = value;
            }
        }
        public static final int ARML_MOTOR_ID = 23;
        public static final boolean ARML_MOTOR_INVERTED = true;
        public static final int ARML_CURRENT_LIMIT = 15;

        public static final int ARMR_MOTOR_ID = 21;
        public static final boolean ARMR_MOTOR_INVERTED = false;
        public static final int ARMR_CURRENT_LIMIT = 15;

        public static final int INTAKEL_MOTOR_ID = 24;
        public static final boolean INTAKEL_MOTOR_INVERTED = true;
        public static final int INTAKEL_CURRENT_LIMIT = 25;

        public static final int INTAKEC_MOTOR_ID = 30;
        public static final boolean INTAKEC_MOTOR_INVERTED = true;
        public static final int INTAKEC_CURRENT_LIMIT = 25;

        public static final int INTAKER_MOTOR_ID = 22;
        public static final boolean INTAKER_MOTOR_INVERTED = true;
        public static final int INTAKER_CURRENT_LIMIT = 25;
    }

    public static final class Climber {
        public static final int MOTOR_ID = 6;
        public static final boolean MOTOR_INVERTED = false;
        public static final int CURRENT_LIMIT = 45;

        public static final double MIN_POSITION_METERS = 0.0;
        public static final double MAX_POSITION_METERS = 1.0; // TODO

        public static final double GEARING = 64.0;
        public static final double MASS_KG = Units.lbsToKilograms(80); // robot weight
        public static final double SPOOL_RADIUS_METERS = Units.inchesToMeters(0.5);
        public static final double SPOOL_CIRCUMFERENCE = 2.0 * Math.PI * SPOOL_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = SPOOL_CIRCUMFERENCE * GEARING;
    }
}
