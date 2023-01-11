// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdio>
#include <thread>

#include <units/length.h>

#include <cameraserver/CameraServer.h>
#include <frc/apriltag/AprilTagDetector.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>
#include <frc/TimedRobot.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * This is a demo program showing the detection of AprilTags.
 * The image is acquired from the USB camera, then any detected AprilTags 
 * are marked up on the image and sent to the dashboard.
 */
class Robot : public frc::TimedRobot {
#if defined(__linux__) || defined(_WIN32)

 private:
  static void VisionThread() {
    frc::AprilTagDetector detector;
    // look for tag16h5, don't correct any error bits
    detector.AddFamily("tag16h5", 0);

    // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
    // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
    auto tagSize = units::length::meter_t(0.1524);
    frc::AprilTagPoseEstimator::Config postEstConfig =
      { .tagSize = tagSize, .fx = 699.3778103158814, .fy = 677.7161226393544, .cx = 345.6059345433618, .cy = 207.12741326228522 };
    //AprilTagPoseEstimator.Config poseEstConfig = new AprilTagPoseEstimator.Config(0.1524, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522);
    //AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(poseEstConfig);

    // Get the USB camera from CameraServer
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
    // Set the resolution
    camera.SetResolution(640, 480);

    // Get a CvSink. This will capture Mats from the Camera
    cs::CvSink cvSink = frc::CameraServer::GetVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    cs::CvSource outputStream =
        frc::CameraServer::PutVideo("Detected", 640, 480);

    // Mats are very memory expensive. Lets reuse this Mat.
    cv::Mat mat;
    cv::Mat grayMat;

    // Instantiate once
    // how to do ArrayList?
    cv::Scalar outlineColor = cv::Scalar(0, 255, 0);
    cv::Scalar crossColor = cv::Scalar(0, 0, 255);

    while (true) {
      // Tell the CvSink to grab a frame from the camera and
      // put it
      // in the source mat.  If there is an error notify the
      // output.
      if (cvSink.GrabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.NotifyError(cvSink.GetError());
        // skip the rest of the current iteration
        continue;
      }

      // Put a rectangle on the image
      rectangle(mat, cv::Point(100, 100), cv::Point(400, 400),
                cv::Scalar(255, 255, 255), 5);
      // Give the output stream a new image to display
      outputStream.PutFrame(mat);
    }
  }
#endif

  void RobotInit() override {
    // We need to run our vision program in a separate thread. If not, our robot
    // program will not run.
#if defined(__linux__) || defined(_WIN32)
    std::thread visionThread(VisionThread);
    visionThread.detach();
#else
    std::fputs("Vision only available on Linux or Windows.\n", stderr);
    std::fflush(stderr);
#endif
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
