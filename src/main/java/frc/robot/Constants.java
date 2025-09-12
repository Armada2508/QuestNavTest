package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import java.io.File;
import java.io.IOException;
import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;

public class Constants {

public static final Angle halfTurn = Degrees.of(180);
    public static final LinearAccelerationUnit InchesPerSecondPerSecond = InchesPerSecond.per(Second);

    public static class SwerveK {
        public static final Distance driveBaseRadius = Inches.of(Math.hypot(12.75, 12.75));
        public static final Distance driveBaseLength = Inches.of(35); // Base is a square so this is the same as the width
        public static final Time coastDisableTime = Seconds.of(10);

        // Currently Unused
        public static final double steerGearRatio = 41.25; 
        public static final double driveGearRatio = 4.4;

        public static final LinearVelocity maxPossibleRobotSpeed = MetersPerSecond.of(5.426);
        public static final AngularVelocity maxAngularVelocity = RadiansPerSecond.of(10.477);
        public static final CurrentLimitsConfigs currentLimitsConfig = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Amps.of(70)).withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(Amps.of(75)).withStatorCurrentLimitEnable(true);

        // Path Constraints
        public static final LinearVelocity maxRobotVelocity = FeetPerSecond.of(6); // Should be just under 3/4 of our max possible speed, arbitrary value
        public static final LinearAcceleration maxRobotAcceleration = FeetPerSecondPerSecond.of(3.5); 
        public static final AngularVelocity maxRobotAngularVelocity = DegreesPerSecond.of(180); 
        public static final AngularAcceleration maxRobotAngularAcceleration = DegreesPerSecondPerSecond.of(270); 

        // Drive Feedforward
        public static final double kS = 0.23118;
        public static final double kV = 2.1701;
        public static final double kA = 0.15136;

        // PathPlanner
        public static final PIDConstants ppTranslationConstants = new PIDConstants(5.25, 0, 0); // m/s / m
        public static final PIDConstants ppRotationConstants = new PIDConstants(5, 0, 0); // rad/s / rad
        public static RobotConfig robotConfig; static {
            try {
                robotConfig = RobotConfig.fromGUISettings();
            } catch (IOException | ParseException e) {
                e.printStackTrace();
            }
        }

        // PID Alignment
        public static final PIDConstants translationConstants = new PIDConstants(5.25, 0, 0); // m/s / m of error
        public static final PIDConstants rotationConstants = new PIDConstants(5, 0, 0); // rad/s / rad of error
        public static final TrapezoidProfile.Constraints defaultTranslationConstraints = 
            new TrapezoidProfile.Constraints(Units.feetToMeters(5), Units.feetToMeters(10)); // m/s & m/s^2
        public static final TrapezoidProfile.Constraints defaultRotationConstraints = 
            new TrapezoidProfile.Constraints(Units.degreesToRadians(360), Units.degreesToRadians(360)); // rad/s & rad/s^2
        public static final TrapezoidProfile.Constraints climbTranslationConstraints = 
            new TrapezoidProfile.Constraints(Units.feetToMeters(2), Units.feetToMeters(4)); // m/s & m/s^2
        public static final TrapezoidProfile.Constraints climbRotationConstraints = 
            new TrapezoidProfile.Constraints(Units.degreesToRadians(180), Units.degreesToRadians(180)); // rad/s & rad/s^2
        public static final Distance maximumTranslationError = Inches.of(0.25);
        public static final Angle maximumRotationError = Degrees.of(0.5);

        public static final File swerveDirectory = new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/swerve");
    }

    public static class ControllerK {
        public static final int xboxPort = 0;
        public static final double leftJoystickDeadband = 0.07;
        public static final double rightJoystickDeadband = 0.07;

        // Teleop Alignment Overriding
        public static final double overrideThreshold = 0.14;
        public static final Time overrideTime = Seconds.of(0.25);
    }

    public static class DriveK {
        // Larger number = faster rate of change, limit is in units of (units)/second. In this case the joystick [-1, 1].
        public static final Pair<Double, Double> translationAccelLimits = Pair.of(1.25, 2.0); 
        public static final Pair<Double, Double> rotationAccelLimits = Pair.of(1.0, 2.0);

        public static final double driveSpeedModifier = 1;
        public static final double rotationSpeedModifier = 1;
        public static final double exponentialControl = 1.75;
    }

    public static class VisionK {

    }

    public static class QuestK {
        public static final Transform2d questOffset = new Transform2d(0, 0, Rotation2d.k180deg);
    }
}
