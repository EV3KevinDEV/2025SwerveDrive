// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = false;
  public static PhotonCameraSim cameraSim;
  public static VisionSystemSim visionSim;

  public Robot() {

    Translation3d robotToCameraTrl = new Translation3d(-0.276, -0.228, 0.238);
    // and pitched 15 degrees up.
    Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-170), Math.toRadians(-35));


    Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);


    visionSim = new VisionSystemSim("main");

    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    visionSim.addAprilTags(tagLayout);

    SimCameraProperties cameraProp = new SimCameraProperties();

    // A 640 x 480 camera with a 100 degree diagonal FOV.
    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    // Approximate detection noise with average and standard deviation error in pixels.
    cameraProp.setCalibError(0.25, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    cameraProp.setFPS(20);
    // The average and standard deviation in milliseconds of image data latency.
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    PhotonCamera camera = new PhotonCamera("cameraName");

    PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);

    visionSim.addCamera(cameraSim, robotToCamera);

    m_robotContainer = new RobotContainer();


   }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SignalLogger.stop();

    if (kUseLimelight) {
    var driveState = m_robotContainer.drivetrain.getState();
    double headingDeg = driveState.Pose.getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation("limelight-happy", headingDeg, 0, 0, 0, 0, 0);

    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-happy");
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-happy");

    boolean doRejectUpdate = false;
    if(mt2.tagCount == 1 && mt2.rawFiducials.length == 1)
      {
        if(mt2.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt2.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        m_robotContainer.drivetrain.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }

      
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    // m_robotContainer.setArmCoast();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    // var driveState = m_robotContainer.drivetrain.getState();

    // visionSim.update(driveState.Pose);
    // visionSim.getDebugField();


  }
}