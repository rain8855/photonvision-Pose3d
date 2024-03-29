// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;                                                                                                                                    

import frc.robot.Constants;

import java.util.List;
import java.util.Optional;
import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;


public class photonvision extends SubsystemBase {

    private PhotonCamera camera = new PhotonCamera("limelight");
    
    private PhotonPipelineResult result = camera.getLatestResult();

    public boolean hasTargets;
    PhotonTrackedTarget target;

    Transform3d cameraToRobot = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    Transform3d robotToCam = new Transform3d(new Translation3d(0.42, 0, 0.22), new Rotation3d(0,0,0));

    public AprilTagFieldLayout aprilTagFieldLayout;
    public PhotonPoseEstimator photonPoseEstimator;
    
    private Pose2d pose2d = new Pose2d(0, 0, new Rotation2d(180));
    private Pose3d robotPose;

    private final Field2d m_field = new Field2d();

    private boolean loadSuccess;
    /**
   * Example command factory method.
   *
  //  * @return a command
   */
  public photonvision() {
    
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    
      loadSuccess = true;
    } catch (IOException e) {
      e.printStackTrace();
      loadSuccess = false;
    }
    SmartDashboard.putBoolean("load", loadSuccess);
    
    SmartDashboard.putData("Field", m_field);
    
  }

  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  // public boolean exampleCondition() {
  //   // Query some boolean state, such as a digital sensor.
  //   return false;
  // }

  public Pose2d getPose(){
    result = camera.getLatestResult();
    target = result.getBestTarget();
  
    if (target != null){
      Transform3d camerToTarget = target.getBestCameraToTarget();

      Optional<Pose3d> TagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
      
      if(TagPose.isPresent()){
        robotPose = PhotonUtils.estimateFieldToRobotAprilTag(camerToTarget, TagPose.get(), cameraToRobot);
      }
      else {
        robotPose = new Pose3d();
      }
      System.out.println("Pose get");
      return robotPose.toPose2d();
    }
    System.out.println("Pose is null");
    return new Pose2d();
  }

  // public static void subsystem(String[] args){
  //   System.out.println("fuck you");
  // }
  

  @Override
  public void periodic() {
    
    PhotonPipelineResult result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    SmartDashboard.putBoolean("hastarget", hasTargets);

    m_field.setRobotPose(getPose());
    
    SmartDashboard.putData("m_field", m_field);
    
    // if(!pose.isEmpty()) {
      // pose2d = pose.get().estimatedPose.toPose2d();
    
      // getPose2d = true;
      // System.out.println("getPose2d");
    // }
    // else{
      // System.out.println("NOgetPose2d");
    // }
    // SmartDashboard.putBoolean("getPose2d", getPose2d);


    // !!!!!! if(hasTargets = true){
    //   target = result.getBestTarget();

    // }
    
  }
  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
  }
}

