package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerK;
import frc.robot.Constants.SwerveK;
import frc.robot.Robot;

import frc.robot.commands.DriveWheelCharacterization;

import swervelib.SwerveDrive;
import swervelib.motors.TalonFXSwerve;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

@Logged
public class Swerve extends SubsystemBase { // physicalproperties/conversionFactors/angle/factor = 360.0 deg/4096.0 units per rotation

    private final SwerveDrive swerveDrive;
    private final TalonFX frontLeft;
    private final TalonFX frontRight;
    private final TalonFX backLeft;
    private final TalonFX backRight;
    private final SysIdRoutine sysIdRoutine; 
    private final PPHolonomicDriveController pathPlannerController = new PPHolonomicDriveController(SwerveK.ppTranslationConstants, SwerveK.ppRotationConstants);
    private boolean initializedOdometryFromVision = false;
    @SuppressWarnings("unused")
    private Pose2d pathPlannerTarget = Pose2d.kZero; // For logging
    // PID Alignment
    private final BooleanSupplier overridePathFollowing;
    private final Debouncer overrideDebouncer = new Debouncer(ControllerK.overrideTime.in(Seconds));
    private Pose2d targetPose;
    private boolean completedAlignmentBool = false;
    public final Trigger completedAlignment = new Trigger(() -> completedAlignmentBool);
    private final ProfiledPIDController xController = new ProfiledPIDController(SwerveK.translationConstants.kP, SwerveK.translationConstants.kI, SwerveK.translationConstants.kD, SwerveK.defaultTranslationConstraints);
    private final ProfiledPIDController yController = new ProfiledPIDController(SwerveK.translationConstants.kP, SwerveK.translationConstants.kI, SwerveK.translationConstants.kD, SwerveK.defaultTranslationConstraints);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(SwerveK.rotationConstants.kP, SwerveK.rotationConstants.kI, SwerveK.rotationConstants.kD, SwerveK.defaultRotationConstraints);

    public Swerve(BooleanSupplier overridePathFollowing) {
        this.overridePathFollowing = overridePathFollowing;
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        SwerveParser parser = null;
        try {
            parser = new SwerveParser(SwerveK.swerveDirectory);
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException("Swerve directory not found.");
        }
        swerveDrive = parser.createSwerveDrive(SwerveK.maxPossibleRobotSpeed.in(MetersPerSecond));
        swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(SwerveK.kS, SwerveK.kV, SwerveK.kA));
        frontLeft = (TalonFX) swerveDrive.getModules()[0].getDriveMotor().getMotor();
        frontRight = (TalonFX) swerveDrive.getModules()[1].getDriveMotor().getMotor();
        backLeft = (TalonFX) swerveDrive.getModules()[2].getDriveMotor().getMotor();
        backRight = (TalonFX) swerveDrive.getModules()[3].getDriveMotor().getMotor();
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null,        // Use default timeout (10 s)
                            // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    VoltageOut request = new VoltageOut(volts);
                    for (var module : swerveDrive.getModules()) {
                        var motor = (TalonFXSwerve) module.getDriveMotor();
                        ((TalonFX) motor.getMotor()).setControl(request);
                    }
                },
                null,
                this
            )
        );
        setupPathPlanner();
        frontLeft.getConfigurator().apply(SwerveK.currentLimitsConfig);
        frontRight.getConfigurator().apply(SwerveK.currentLimitsConfig);
        backLeft.getConfigurator().apply(SwerveK.currentLimitsConfig);
        backRight.getConfigurator().apply(SwerveK.currentLimitsConfig);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("X setpoint", xController.getSetpoint().position);
        SmartDashboard.putNumber("Y setpoint", yController.getSetpoint().position);
        // swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, result.getSecond());
    }

    private void setupPathPlanner() {
        AutoBuilder.configure(
            this::getPose, 
            this::resetOdometry, 
            this::getChassisSpeeds, 
            (speeds, feedforward) -> setChassisSpeeds(speeds), 
            pathPlannerController,
            SwerveK.robotConfig,
            Robot::onRedAlliance, 
            this);
    }

    /**
     * Commands the robot to drive according to the given velocities, this switches the direction depending on what alliance you're on
     * @param TranslationX Translation in the X direction (Forwards, Backwards) between -1 and 1
     * @param TranslationY Translation in the Y direction (Left, Right) between -1 and 1
     * @param angularVelocity Angular Velocity to set between -1 and 1
     * @param fieldRelative Whether or not swerve is controlled using field relative speeds
     * @return A command to drive the robot according to given velocities
     */
    public Command driveCommand(DoubleSupplier TranslationX, DoubleSupplier TranslationY, DoubleSupplier angularVelocity, boolean fieldRelative, boolean openLoop) {
        return runOnce(() -> {
            Translation2d translation = new Translation2d(TranslationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(), TranslationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity());
            AngularVelocity rotation = RadiansPerSecond.of(angularVelocity.getAsDouble() * (swerveDrive.getMaximumChassisVelocity() / SwerveK.driveBaseRadius.in(Meters)));
            drive(Robot.onRedAlliance() ? translation.unaryMinus() : translation, rotation, fieldRelative, openLoop);
        }).withName("Swerve Drive");
    }

    /**
     * Constructs a command to take the robot from current position to an end position. This does not flip the path depending on alliance
     * @param targetPoseSupplier Supplier of the target pose
     * @return Command to drive along the constructed path
     */
    public Command alignToPosePP(Supplier<Pose2d> targetPoseSupplier) {
        PathConstraints constraints = new PathConstraints(SwerveK.maxRobotVelocity, SwerveK.maxRobotAcceleration, SwerveK.maxRobotAngularVelocity, SwerveK.maxRobotAngularAcceleration);
        return Commands.defer(() -> {
            Pose2d targetPose = targetPoseSupplier.get();
            pathPlannerTarget = targetPose;
            PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(getPose(), targetPose), 
                constraints, 
                null, 
                new GoalEndState(MetersPerSecond.zero(), targetPose.getRotation()));
            return new FollowPathCommand(
                path, 
                this::getPose, 
                this::getChassisSpeeds, 
                (speeds, feedforward) -> setChassisSpeeds(speeds),
                pathPlannerController, 
                SwerveK.robotConfig,
                () -> false, 
                this 
            ).until(() -> overrideDebouncer.calculate(overridePathFollowing.getAsBoolean()))
            .finallyDo(this::stop);
        }, Set.of(this)).withName("PP Align");
    }

    private boolean resetpid = false;

    /**
     * Command to drive the robot to another position without creating a path
     * @param targetPoseSupplier Supplier of the target pose
     * @return The command
     */
    public Command alignToPosePID(Supplier<Pose2d> targetPoseSupplier, TrapezoidProfile.Constraints translationConstraints, TrapezoidProfile.Constraints rotationConstraints) {
        return runOnce(() -> {
            targetPose = targetPoseSupplier.get();
            overrideDebouncer.calculate(false);
            completedAlignmentBool = false;
            resetpid = true;
        }).andThen(
            run(() -> {
            var pose = getPose();
            if (resetpid) {
                var speeds = getChassisSpeeds();
                xController.reset(pose.getX() - targetPose.getX(), 0);
                yController.reset(pose.getY() - targetPose.getY(), 0);
                thetaController.reset(pose.getRotation().getRadians(), speeds.omegaRadiansPerSecond);
                xController.setConstraints(translationConstraints);
                yController.setConstraints(translationConstraints);
                thetaController.setConstraints(rotationConstraints);
                System.out.println(pose);
                System.out.println(speeds);
                resetpid = false;
            }
            double xFeedback = xController.calculate(pose.getX() - targetPose.getX(), 0);
            double yFeedback = yController.calculate(pose.getY() - targetPose.getY(), 0);
            double thetaFeedback = thetaController.calculate(pose.getRotation().getRadians(), targetPose.getRotation().getRadians());
            setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xFeedback, yFeedback, thetaFeedback, pose.getRotation()));
        })).until(() -> {
            boolean withinError = (getPose().getTranslation().getDistance(targetPose.getTranslation()) < SwerveK.maximumTranslationError.in(Meters)
            && Math.abs(getPose().getRotation().minus(targetPose.getRotation()).getDegrees()) < SwerveK.maximumRotationError.in(Degrees));
            boolean override = overrideDebouncer.calculate(overridePathFollowing.getAsBoolean());
            if (withinError) completedAlignmentBool = true;
            return withinError || override;
        }).finallyDo(this::stop).withName("PID Align");
    }

    public Command alignToPosePID(Supplier<Pose2d> targetPoseSupplier) {
        return alignToPosePID(targetPoseSupplier, SwerveK.defaultTranslationConstraints, SwerveK.defaultRotationConstraints);
    }

    public Command setDriveVoltage(Voltage volts) {
        VoltageOut request = new VoltageOut(volts);
        return run(() -> {
            for (var module : swerveDrive.getModules()) {
            var motor = (TalonFXSwerve) module.getDriveMotor();
            ((TalonFX) motor.getMotor()).setControl(request);
        }}).finallyDo(this::stop);
    }

    /**
     * Commands the drivebase to move according to the given linear and rotational velocities
     * @param translation Linear velocity of the robot in meters per second
     * @param rotation Rotation rate of the robot in Radians per second
     * @param fieldRelative Whether the robot is field relative (true) or robot relative (false)
     * @param isOpenLoop Whether it uses a closed loop velocity control or an open loop
     */
    private void drive(Translation2d translation, AngularVelocity rotation, boolean fieldRelative, boolean isOpenLoop) {
        swerveDrive.drive(translation, rotation.in(RadiansPerSecond), fieldRelative, isOpenLoop);
    }

    public void stop() {
        drive(Translation2d.kZero, RadiansPerSecond.zero(), true, false);
    }

    /**
     * Resets the odometry to the given pose
     * @param pose Pose to reset the odemetry to
     */
    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    /**
     * Returns the robot's pose
     * @return Current pose of the robot as a Pose2d
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Returns the robot's velocity (x, y, and omega)
     * @return Current velocity of the robot
     */
    public ChassisSpeeds getChassisSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    @Logged
    public double getLinearVelocity() {
        return Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond);
    }

    /**
     * Set the speed of the robot with closed loop velocity control
     * @param chassisSpeeds to set speed with (robot relative)
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Returns the robot's heading as an Angle wrapped between -180 and 180
     * @return The heading of the robot
     */
    public Angle getHeading() {
        return getPose().getRotation().getMeasure();
    }

    /**
     * Returns the gyro's yaw, not wrapped
     * @return The yaw of the gyro
     */
    public Angle getYaw() {
        return ((Pigeon2) swerveDrive.getGyro().getIMU()).getYaw().getValue();
    }

    /**
     * Returns the positions of each drive wheel in radians, front left -> front right -> back left -> back right.
     * @return
     */
    public double[] getWheelPositions() {
        var modulePositions = swerveDrive.getModulePositions();
        double[] wheelPositions = new double[modulePositions.length];
        for (int i = 0; i < modulePositions.length; i++) {
            double diameter = Units.inchesToMeters(swerveDrive.swerveDriveConfiguration.physicalCharacteristics.conversionFactor.drive.diameter);
            wheelPositions[i] = Units.rotationsToRadians(modulePositions[i].distanceMeters / (diameter * Math.PI));
        }
        return wheelPositions;
    }

    @Logged(name = "Current Command")
    public String getCurrentCommandName() {
        var cmd = getCurrentCommand();
        if (cmd == null) return "None";
        return cmd.getName();
    }

    public boolean initializedOdometryFromVision() {
        return initializedOdometryFromVision;
    }

    /**
     * Resets the gyro and odometry to the current position but the current direction is now seen as 0.
     * Useful for resetting the forward direction for field relative driving
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public void setCoastMode() {
        swerveDrive.setMotorIdleMode(false);
    }

    public void setBrakeMode() {
        swerveDrive.setMotorIdleMode(true);
    }

    public Command faceWheelsForward() {
        SwerveModuleState state = new SwerveModuleState();
        return run(() -> {
            for (var module : swerveDrive.getModules()) {
                module.setDesiredState(state, true, 0);
            }
        }).finallyDo(this::stop).withName("Face Forward");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }
     
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public Command characterizeDriveWheelDiameter() {
        return new DriveWheelCharacterization(this);
    }

}