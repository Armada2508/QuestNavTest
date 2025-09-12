package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.QuestK;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Quest extends SubsystemBase { {
    QuestNav questNav = new QuestNav();

    Swerve swerveDriveSubsystem = new Swerve(() -> false);
    Matrix<N3, N1> QUESTNAV_STD_DEVS =
        VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
            0.02, // Trust down to 2cm in Y direction
            0.035 // Trust down to 2 degrees rotational
    );

    if (questNav.isTracking()) {
        // Get the latest pose data frames from the Quest
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        // Loop over the pose data frames and send them to the pose estimator
        for (PoseFrame questFrame : questFrames) {
            // Get the pose of the Quest
            Pose2d questPose = questFrame.questPose();
            // Get timestamp for when the data was sent
            double timestamp = questFrame.dataTimestamp();

            // Transform by the mount pose to get your robot pose
            Pose2d robotPose = questPose.transformBy(QuestK.questOffset.inverse());

            // Convert FPGA timestamp to CTRE's time domain using Phoenix 6 utility
            // double ctreTimestamp = Utils.fpgaToCurrentTime(timestamp);

            // // You can put some sort of filtering here if you would like!

            // // Add the measurement to our estimator
            // swerveDriveSubsystem.addVisionMeasurement(robotPose, ctreTimestamp, QUESTNAV_STD_DEVS);
        };
    }
}};


