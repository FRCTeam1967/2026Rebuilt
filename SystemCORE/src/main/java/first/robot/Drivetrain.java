// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package first.robot;

// import org.wpilib.hardware.imu.OnboardIMU;
// import org.wpilib.hardware.motor.PWMSparkMax;
// import org.wpilib.hardware.rotation.Encoder;
// import org.wpilib.math.controller.PIDController;
// import org.wpilib.math.controller.SimpleMotorFeedforward;
// import org.wpilib.math.kinematics.ChassisVelocities;
// import org.wpilib.math.kinematics.DifferentialDriveKinematics;
// import org.wpilib.math.kinematics.DifferentialDriveOdometry;
// import org.wpilib.math.kinematics.DifferentialDriveWheelVelocities;
// import first.robot.generated.TunerConstants.TunerSwerveDrivetrain;

// /** Represents a differential drive style drivetrain. */
// public class Drivetrain extends TunerSwerveDrivetrain {
//   public static final double kMaxVelocity = 3.0; // meters per second
//   public static final double kMaxAngularVelocity = 2 * Math.PI; // one rotation per second

//   private static final double kTrackwidth = 0.381 * 2; // meters
//   private static final double kWheelRadius = 0.0508; // meters
//   private static final int kEncoderResolution = 4096;

//   private final PWMSparkMax leftLeader = new PWMSparkMax(1);
//   private final PWMSparkMax leftFollower = new PWMSparkMax(2);
//   private final PWMSparkMax rightLeader = new PWMSparkMax(3);
//   private final PWMSparkMax rightFollower = new PWMSparkMax(4);

//   private final Encoder leftEncoder = new Encoder(0, 1);
//   private final Encoder rightEncoder = new Encoder(2, 3);

//   private final OnboardIMU imu = new OnboardIMU(OnboardIMU.MountOrientation.FLAT);

//   private final PIDController leftPIDController = new PIDController(1, 0, 0);
//   private final PIDController rightPIDController = new PIDController(1, 0, 0);

//   private final DifferentialDriveKinematics kinematics =
//       new DifferentialDriveKinematics(kTrackwidth);

//   private final DifferentialDriveOdometry odometry;

//   // Gains are for example purposes only - must be determined for your own robot!
//   private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);

//   /**
//    * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
//    * gyro.
//    */
//   public Drivetrain() {
//     imu.resetYaw();

//     leftLeader.addFollower(leftFollower);
//     rightLeader.addFollower(rightFollower);

//     // We need to invert one side of the drivetrain so that positive voltages
//     // result in both sides moving forward. Depending on how your robot's
//     // gearbox is constructed, you might have to invert the left side instead.
//     rightLeader.setInverted(true);

//     // Set the distance per pulse for the drive encoders. We can simply use the
//     // distance traveled for one rotation of the wheel divided by the encoder
//     // resolution.
//     leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
//     rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

//     leftEncoder.reset();
//     rightEncoder.reset();

//     odometry =
//         new DifferentialDriveOdometry(
//             imu.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
//   }

//   /**
//    * Sets the desired wheel velocities.
//    *
//    * @param velocities The desired wheel velocities.
//    */
//   public void setVelocities(DifferentialDriveWheelVelocities velocities) {
//     final double leftFeedforward = feedforward.calculate(velocities.left);
//     final double rightFeedforward = feedforward.calculate(velocities.right);

//     final double leftOutput = leftPIDController.calculate(leftEncoder.getRate(), velocities.left);
//     final double rightOutput =
//         rightPIDController.calculate(rightEncoder.getRate(), velocities.right);
//     leftLeader.setVoltage(leftOutput + leftFeedforward);
//     rightLeader.setVoltage(rightOutput + rightFeedforward);
//   }

//   /**
//    * Drives the robot with the given linear velocity and angular velocity.
//    *
//    * @param xVelocity Linear velocity in m/s.
//    * @param rot Angular velocity in rad/s.
//    */
//   public void drive(double xVelocity, double rot) {
//     var wheelVelocities = kinematics.toWheelVelocities(new ChassisVelocities(xVelocity, 0.0, rot));
//     setVelocities(wheelVelocities);
//   }

//   /** Updates the field-relative position. */
//   public void updateOdometry() {
//     odometry.update(imu.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
//   }
// }

package first.robot;

import static org.wpilib.units.Units.Second;

import java.util.Optional;
import java.util.function.Supplier;

import org.wpilib.system.RobotController;

import org.wpilib.command2.Command;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.Notifier;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

// TODO: keep checking for systemcore-compatible choreo vendordep
// import choreo.Choreo.TrajectoryLogger;
// import choreo.auto.AutoFactory;
// import choreo.trajectory.SwerveSample;

import org.wpilib.math.numbers.*;
import org.wpilib.units.*;
import org.wpilib.math.linalg.Matrix;

import dev.doglog.DogLog;
import first.robot.generated.TunerConstants.TunerSwerveDrivetrain;

import org.ejml.data.*;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 *
 * Generated by the 2026 Tuner X Swerve Project Generator
 * https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
 */
public class Drivetrain extends TunerSwerveDrivetrain {
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    
    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

    private final SwerveRequest.FieldCentric m_followRequest = new SwerveRequest.FieldCentric()
    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    // /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    // private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         null,        // Use default ramp rate (1 V/s)
    //         Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
    //         null,        // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         output -> setControl(m_translationCharacterization.withVolts(output)),
    //         null,
    //         this
    //     )
    // );

    // /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    // private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         null,        // Use default ramp rate (1 V/s)
    //         Volts.of(7), // Use dynamic voltage of 7 V
    //         null,        // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         volts -> setControl(m_steerCharacterization.withVolts(volts)),
    //         null,
    //         this
    //     )
    // );

    // /*
    //  * SysId routine for characterizing rotation.
    //  * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
    //  * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
    //  */
    // private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         /* This is in radians per second², but SysId only supports "volts per second" */
    //         Volts.of(Math.PI / 6).per(Second),
    //         /* This is in radians per second, but SysId only supports "volts" */
    //         Volts.of(Math.PI),
    //         null, // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         output -> {
    //             /* output is actually radians per second, but SysId only supports "volts" */
    //             setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
    //             /* also log the requested output for SysId */
    //             SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
    //         },
    //         null,
    //         this
    //     )
    // );

    // /* The SysId routine to test */
    // private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public Drivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
            headingController.enableContinuousInput(-Math.PI, Math.PI);
        }
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public Drivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
            headingController.enableContinuousInput(headingMin, headingMax);
        }
        headingController.enableContinuousInput(-Math.PI, Math.PI); 
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public Drivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
            headingController.enableContinuousInput(headingMin, headingMax);
        }
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    // /**
    //  * Returns a command that applies the specified control request to this swerve drivetrain.
    //  *
    //  * @param request Function returning the request to apply
    //  * @return Command to run
    //  */
    // public Command applyRequest(Supplier<SwerveRequest> request) {
    //     return run(() -> this.setControl(request.get()));
    // }

    // /**
    //  * Runs the SysId Quasistatic test in the given direction for the routine
    //  * specified by {@link #m_sysIdRoutineToApply}.
    //  *
    //  * @param direction Direction of the SysId Quasistatic test
    //  * @return Command to run
    //  */
    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutineToApply.quasistatic(direction);
    // }

    // /**
    //  * Runs the SysId Dynamic test in the given direction for the routine
    //  * specified by {@link #m_sysIdRoutineToApply}.
    //  *
    //  * @param direction Direction of the SysId Dynamic test
    //  * @return Command to run
    //  */
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutineToApply.dynamic(direction);
    // }

    /**
     * @return Pose2d from current state of the robot
     */
    public Pose2d getPose() {
        return getState().Pose;
        
    }

    // TODO: update once choreo vendordep added
    // /**
    //  * trajectory follower for choreo AutoFactory </p>
    //  * gets the current pose of the robot </p>
    //  * generates and applies the next speeds for the robot
    //  * @param sample
    //  */
    // public void followTrajectory(SwerveSample sample) {
    //     // Get the current pose of the robot
    //     Pose2d pose = getPose();
    //     double rotationalRate = sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading);

    //     if (Constants.Drivetrain.verboseLogging) {
    //         DogLog.log("Drivetrain/Trajectory/sample", new Pose2d(sample.x, sample.y, Rotation2d.fromRadians(sample.heading)));
    //         DogLog.log("Drivetrain/Trajectory/commanded rot rate", rotationalRate);
    //     }

    //     // Generate and apply the next speeds for the robot
    //     setControl(m_followRequest
    //         .withVelocityX(sample.vx + xController.calculate(pose.getX(), sample.x))
    //         .withVelocityY(sample.vy + yController.calculate(pose.getY(), sample.y))
    //         .withRotationalRate(rotationalRate)
    //         .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance));


    //    //ChassisSpeeds speeds = new ChassisSpeeds(
    //         //,sample.vx + xController.calculate(pose.getX(), sample.x)
    //         //sample.vy + yController.calculate(pose.getY(), sample.y),
    //         //sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
    //     //);

    //     // Apply the generated speeds
    //     // driveFieldRelative(speeds);
    // }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        //OLD PERIODIC CODE
        // if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
        //     DriverStation.getAlliance().ifPresent(allianceColor -> {
        //         setOperatorPerspectiveForward(
        //             allianceColor == Alliance.Red
        //                 ? kRedAlliancePerspectiveRotation
        //                 : kBlueAlliancePerspectiveRotation
        //         );
        //         m_hasAppliedOperatorPerspective = true;
        //     });
        // }

        // DogLog.log("Drivetrain/pose", getPose());

        if (!m_hasAppliedOperatorPerspective || RobotState.isDisabled()) {

            MatchState.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );

            m_hasAppliedOperatorPerspective = true;
        });
        }

        DogLog.log("Drivetrain/pose", getPose());
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
    //  * {@link #setVisionMeasurementStdDevs()} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    /**
     * Return the pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The timestamp of the pose in seconds.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
     */
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}