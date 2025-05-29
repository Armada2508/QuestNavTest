// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  public Robot() {
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
    * Returns the alliance
    * @return true if the robot is on Red, false if the robot is on Blue
    */
  public static boolean onRedAlliance() {
      return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

}
