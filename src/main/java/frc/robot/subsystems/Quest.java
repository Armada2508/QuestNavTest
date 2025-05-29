package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.QuestNav;
import frc.robot.Constants.QuestK;

public class Quest extends SubsystemBase {
    // public record QuestData(
        // double batteryPercent,
        // double timestamp,
        // Pose3d pos
        //~ Maybe finish this? see how much we need it
    // ) {}

     QuestNav quest = new QuestNav();

    @Override
    public void periodic() {
       quest.cleanupResponses();
       quest.processHeartbeat();
       quest.setPose(quest.getPose().transformBy(QuestK.questOffset));
    }

    public Pose2d getRobotPose() {
        return quest.getPose().transformBy(QuestK.questOffset);
    }

    public Matrix<N3, N1> getQuestResults() {
        return null; //! Fix
    }

    public boolean isQuestHappy() {
        return quest.getConnected() && quest.getTrackingStatus();
    }

    public Rotation2d getQuestRotation() {
        return quest.getPose().getRotation();
    }
}
