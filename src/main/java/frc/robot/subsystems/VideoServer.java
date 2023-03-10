// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VideoServer extends SubsystemBase {

  // Gets usb camera and sink which contains video frames
  private UsbCamera camera;
  private CvSink cvSink;

  /** Creates a new VideoServer. */
  public VideoServer() {
    camera = CameraServer.startAutomaticCapture();
   camera.setResolution(600, 600);
    this.cvSink = CameraServer.getVideo(); 

    // Puts the video in shuffleboard
    CameraServer.putVideo("Video", 600, 600);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
