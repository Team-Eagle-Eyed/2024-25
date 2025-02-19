/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.*;

/**
     * Drives the swerve drivetrain in a field-centric manner, maintaining a
     * specified heading angle to ensure the robot is facing the desired direction
     * <p>
     * When users use this request, they specify the direction the robot should
     * travel oriented against the field, and the direction the robot should be facing.
     * <p>
     * An example scenario is that the robot is oriented to the east, the VelocityX
     * is +5 m/s, VelocityY is 0 m/s, and TargetDirection is 180 degrees.
     * In this scenario, the robot would drive northward at 5 m/s and turn clockwise
     * to a target of 180 degrees.
     * <p>
     * This control request is especially useful for autonomous control, where the
     * robot should be facing a changing direction throughout the motion.
     */
public class RobotCentricFacingAngle implements SwerveRequest {
    /**
     * The velocity in the X direction, in m/s.
     * X is defined as forward according to WPILib convention,
     * so this determines how fast to travel forward.
     */
    public double VelocityX = 0;
    /**
     * The velocity in the Y direction, in m/s.
     * Y is defined as to the left according to WPILib convention,
     * so this determines how fast to travel to the left.
     */
    public double VelocityY = 0;
    /**
     * The desired direction to face.
     * 0 Degrees is defined as in the direction of the X axis.
     * As a result, a TargetDirection of 90 degrees will point along
     * the Y axis, or to the left.
     */
    public Rotation2d TargetDirection = new Rotation2d();
    /**
     * The rotational rate feedforward to add to the output of the heading
     * controller, in radians per second. When using a motion profile for the
     * target direction, this can be set to the current velocity reference of
     * the profile.
     */
    public double TargetRateFeedforward = 0;

    /**
     * The allowable deadband of the request, in m/s.
     */
    public double Deadband = 0;
    /**
     * The rotational deadband of the request, in radians per second.
     */
    public double RotationalDeadband = 0;
    /**
     * The center of rotation the robot should rotate around.
     * This is (0,0) by default, which will rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /**
     * The type of control request to use for the drive motor.
     */
    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    /**
     * The type of control request to use for the steer motor.
     */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;
    /**
     * Whether to desaturate wheel speeds before applying.
     * For more information, see the documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     */
    public boolean DesaturateWheelSpeeds = true;

    /**
     * The perspective to use when determining which direction is forward.
     */
    public ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue.OperatorPerspective;

    /**
     * The PID controller used to maintain the desired heading.
     * Users can specify the PID gains to change how aggressively to maintain
     * heading.
     * <p>
     * This PID controller operates on heading radians and outputs a target
     * rotational rate in radians per second. Note that continuous input should
     * be enabled on the range [-pi, pi].
     */
    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

    private final FieldCentric m_fieldCentric = new FieldCentric();

    public FieldCentricFacingAngle() {
        HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        Rotation2d angleToFace = TargetDirection;
        if (ForwardPerspective == ForwardPerspectiveValue.OperatorPerspective) {
            /* If we're operator perspective, rotate the direction we want to face by the angle */
            angleToFace = angleToFace.rotateBy(parameters.operatorForwardDirection);
        }

        double toApplyOmega = TargetRateFeedforward +
            HeadingController.calculate(
                parameters.currentPose.getRotation().getRadians(),
                angleToFace.getRadians(),
                parameters.timestamp
            );

        return m_fieldCentric
            .withVelocityX(VelocityX)
            .withVelocityY(VelocityY)
            .withRotationalRate(toApplyOmega)
            .withDeadband(Deadband)
            .withRotationalDeadband(RotationalDeadband)
            .withCenterOfRotation(CenterOfRotation)
            .withDriveRequestType(DriveRequestType)
            .withSteerRequestType(SteerRequestType)
            .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
            .withForwardPerspective(ForwardPerspective)
            .apply(parameters, modulesToApply);
    }

    /**
     * Modifies the PID gains of the HeadingController parameter and returns itself.
     * <p>
     * Sets the proportional, integral, and differential coefficients used to maintain
     * the desired heading. Users can specify the PID gains to change how aggressively to
     * maintain heading.
     * <p>
     * This PID controller operates on heading radians and outputs a target
     * rotational rate in radians per second.
     *
     * @param kP The proportional coefficient; must be >= 0
     * @param kI The integral coefficient; must be >= 0
     * @param kD The differential coefficient; must be >= 0
     * @return this object
     */
    public FieldCentricFacingAngle withHeadingPID(double kP, double kI, double kD)
    {
        this.HeadingController.setPID(kP, kI, kD);
        return this;
    }

    /**
     * Modifies the VelocityX parameter and returns itself.
     * <p>
     * The velocity in the X direction, in m/s. X is defined as forward according to
     * WPILib convention, so this determines how fast to travel forward.
     *
     * @param newVelocityX Parameter to modify
     * @return this object
     */
    public FieldCentricFacingAngle withVelocityX(double newVelocityX) {
        this.VelocityX = newVelocityX;
        return this;
    }

    /**
     * Modifies the VelocityX parameter and returns itself.
     * <p>
     * The velocity in the X direction, in m/s. X is defined as forward according to
     * WPILib convention, so this determines how fast to travel forward.
     *
     * @param newVelocityX Parameter to modify
     * @return this object
     */
    public FieldCentricFacingAngle withVelocityX(LinearVelocity newVelocityX) {
        this.VelocityX = newVelocityX.in(MetersPerSecond);
        return this;
    }

    /**
     * Modifies the VelocityY parameter and returns itself.
     * <p>
     * The velocity in the Y direction, in m/s. Y is defined as to the left
     * according to WPILib convention, so this determines how fast to travel to the
     * left.
     *
     * @param newVelocityY Parameter to modify
     * @return this object
     */
    public FieldCentricFacingAngle withVelocityY(double newVelocityY) {
        this.VelocityY = newVelocityY;
        return this;
    }

    /**
     * Modifies the VelocityY parameter and returns itself.
     * <p>
     * The velocity in the Y direction, in m/s. Y is defined as to the left
     * according to WPILib convention, so this determines how fast to travel to the
     * left.
     *
     * @param newVelocityY Parameter to modify
     * @return this object
     */
    public FieldCentricFacingAngle withVelocityY(LinearVelocity newVelocityY) {
        this.VelocityY = newVelocityY.in(MetersPerSecond);
        return this;
    }

    /**
     * Modifies the TargetDirection parameter and returns itself.
     * <p>
     * The desired direction to face. 0 Degrees is defined as in the direction of
     * the X axis. As a result, a TargetDirection of 90 degrees will point along
     * the Y axis, or to the left.
     *
     * @param newTargetDirection Parameter to modify
     * @return this object
     */
    public FieldCentricFacingAngle withTargetDirection(Rotation2d newTargetDirection) {
        this.TargetDirection = newTargetDirection;
        return this;
    }

    /**
     * Modifies the TargetRateFeedforward parameter and returns itself.
     * <p>
     * The rotational rate feedforward to add to the output of the heading
     * controller, in radians per second. When using a motion profile for the
     * target direction, this can be set to the current velocity reference of
     * the profile.
     *
     * @param newTargetRateFeedforward Parameter to modify
     * @return this object
     */
    public FieldCentricFacingAngle withTargetRateFeedforward(double newTargetRateFeedforward) {
        this.TargetRateFeedforward = newTargetRateFeedforward;
        return this;
    }
    /**
     * Modifies the TargetRateFeedforward parameter and returns itself.
     * <p>
     * The rotational rate feedforward to add to the output of the heading
     * controller, in radians per second. When using a motion profile for the
     * target direction, this can be set to the current velocity reference of
     * the profile.
     *
     * @param newTargetRateFeedforward Parameter to modify
     * @return this object
     */
    public FieldCentricFacingAngle withTargetRateFeedforward(AngularVelocity newTargetRateFeedforward) {
        this.TargetRateFeedforward = newTargetRateFeedforward.in(RadiansPerSecond);
        return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     * <p>
     * The allowable deadband of the request, in m/s.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public FieldCentricFacingAngle withDeadband(double newDeadband) {
        this.Deadband = newDeadband;
        return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     * <p>
     * The allowable deadband of the request, in m/s.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public FieldCentricFacingAngle withDeadband(LinearVelocity newDeadband) {
        this.Deadband = newDeadband.in(MetersPerSecond);
        return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     * <p>
     * The rotational deadband of the request, in radians per second.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public FieldCentricFacingAngle withRotationalDeadband(double newRotationalDeadband) {
        this.RotationalDeadband = newRotationalDeadband;
        return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     * <p>
     * The rotational deadband of the request, in radians per second.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public FieldCentricFacingAngle withRotationalDeadband(AngularVelocity newRotationalDeadband) {
        this.RotationalDeadband = newRotationalDeadband.in(RadiansPerSecond);
        return this;
    }

    /**
     * Modifies the CenterOfRotation parameter and returns itself.
     * <p>
     * The center of rotation the robot should rotate around. This is (0,0) by
     * default, which will rotate around the center of the robot.
     *
     * @param newCenterOfRotation Parameter to modify
     * @return this object
     */
    public FieldCentricFacingAngle withCenterOfRotation(Translation2d newCenterOfRotation) {
        this.CenterOfRotation = newCenterOfRotation;
        return this;
    }

    /**
     * Modifies the DriveRequestType parameter and returns itself.
     * <p>
     * The type of control request to use for the drive motor.
     *
     * @param newDriveRequestType Parameter to modify
     * @return this object
     */
    public FieldCentricFacingAngle withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
        this.DriveRequestType = newDriveRequestType;
        return this;
    }

    /**
     * Modifies the SteerRequestType parameter and returns itself.
     * <p>
     * The type of control request to use for the drive motor.
     *
     * @param newSteerRequestType Parameter to modify
     * @return this object
     */
    public FieldCentricFacingAngle withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
        this.SteerRequestType = newSteerRequestType;
        return this;
    }

    /**
     * Modifies the DesaturateWheelSpeeds parameter and returns itself.
     * <p>
     * Whether to desaturate wheel speeds before applying. For more information, see
     * the documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     *
     * @param newDesaturateWheelSpeeds Parameter to modify
     * @return this object
     */
    public FieldCentricFacingAngle withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
        this.DesaturateWheelSpeeds = newDesaturateWheelSpeeds;
        return this;
    }

    /**
     * Modifies the ForwardPerspective parameter and returns itself.
     * <p>
     * The perspective to use when determining which direction is forward.
     *
     * @param newForwardPerspective Parameter to modify
     * @return this object
     */
    public FieldCentricFacingAngle withForwardPerspective(ForwardPerspectiveValue newForwardPerspective) {
        this.ForwardPerspective = newForwardPerspective;
        return this;
    }
}