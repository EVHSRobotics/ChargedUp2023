// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilScanner extends SubsystemBase {

  private CvSink cameraFeed;
  private AprilTagDetector aprilTagDetector;
  private Mat cvMat;
  private Mat cvGrayMat;
  public Integer tag;

  /** Creates a new AprilTag. */
  public AprilScanner() {
    
    this.aprilTagDetector = new AprilTagDetector();

    aprilTagDetector.addFamily("tag16h5");
    cameraFeed = CameraServer.getVideo();
    cvMat = new Mat();
    cvGrayMat = new Mat();

  }

  public void detectAprilCode() {

    cameraFeed.grabFrame(cvMat);
    Imgproc.cvtColor(cvMat, cvGrayMat, Imgproc.COLOR_RGB2GRAY);

    AprilTagDetection[] detections = aprilTagDetector.detect(cvGrayMat);
    // if (detections.length > 0) {
    //   tags.clear();

    // }
    // SmartDashboard.putString("April", detections.toString());
    if (detections.length > 0) {
      tag = detections[0].getId();
    }


      SmartDashboard.putNumber("AprilTags", tag);
      SmartDashboard.updateValues();
    
    
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
;