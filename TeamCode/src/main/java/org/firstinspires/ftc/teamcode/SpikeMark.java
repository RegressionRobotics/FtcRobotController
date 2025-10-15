package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathbuilder.BezierLine;
import com.pedropathing.pathbuilder.PathBuilder;
import com.pedropathing.pathbuilder.PathChain;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous(name = "Spike Marks", group = "Autonomous")
public class SpikeMarks extends LinearOpMode {

    private Limelight3A limelight;
    private Follower follower;

    private static final int APRILTAG_PIPELINE = 5;

    @Override
    public void runOpMode() {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        // Initialize Pedro Pathing follower (SLOW speed)
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
        follower.setMaxPower(0.3); // SLOW SPEED - 30% power max

        telemetry.addData("Status", "Initialized - Looking for AprilTag");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Detect AprilTag
            int detectedTagId = detectAprilTag();

            telemetry.addData("Detected Tag", detectedTagId);
            telemetry.update();

            // Wait 15 seconds before moving
            telemetry.addData("Status", "Waiting 15 seconds...");
            telemetry.update();
            sleep(15000);

            // Move based on detected tag
            if (detectedTagId == 21) {
                telemetry.addData("Action", "Tag 21 - Moving 12 inches");
                telemetry.update();
                moveForward(12);

            } else if (detectedTagId == 22) {
                telemetry.addData("Action", "Tag 22 - Moving 37 inches");
                telemetry.update();
                moveForward(37);

            } else if (detectedTagId == 23) {
                telemetry.addData("Action", "Tag 23 - Moving 63 inches");
                telemetry.update();
                moveForward(63);

            } else {
                telemetry.addData("Status", "No valid tag detected (21, 22, or 23)");
                telemetry.update();
            }

            telemetry.addData("Status", "Complete!");
            telemetry.update();
        }
    }

    /**
     * Detect AprilTag and return its ID
     */
    private int detectAprilTag() {
        int timeout = 0;
        while (opModeIsActive() && timeout < 100) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                if (fiducials != null && !fiducials.isEmpty()) {
                    int tagId = (int) fiducials.get(0).getFiducialId();
                    return tagId;
                }
            }

            sleep(50);
            timeout++;
        }
        return -1; // No tag found
    }

    /**
     * Move forward a specific distance in inches (SLOW)
     */
    private void moveForward(double inches) {
        // Create path using PathBuilder
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(
                        new com.pedropathing.localization.Pose(0, 0, 0),
                        new com.pedropathing.localization.Pose(0, inches, 0)
                ))
                .build();

        // Follow the path
        follower.followPath(path);

        // Wait until path is complete
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            Pose currentPose = follower.getPose();
            telemetry.addData("Current Y", "%.2f inches", currentPose.getY());
            telemetry.addData("Target", "%.2f inches", inches);
            telemetry.update();
        }
    }
}