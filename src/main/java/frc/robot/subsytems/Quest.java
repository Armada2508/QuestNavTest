package frc.robot.subsytems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.QuestNav;
import frc.robot.Constants.QuestK;

public class Quest extends SubsystemBase {
     QuestNav questNav = new QuestNav();

    @Override
    public void periodic() {
       questNav.cleanupResponses();
       questNav.processHeartbeat();
       questNav.setPose(questNav.getPose().transformBy(QuestK.questOffset));
    }

    public Pose2d getRobotPose() {
        return questNav.getPose().transformBy(QuestK.questOffset);
    }

    public boolean isQuestHappy() {
        return questNav.getConnected() && questNav.getTrackingStatus();
    }
}
