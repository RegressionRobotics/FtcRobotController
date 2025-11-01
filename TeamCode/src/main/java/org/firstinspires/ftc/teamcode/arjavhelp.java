package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "GoToScoringPose", group = "Opmode")
public class arjavhelp extends OpMode {

    // === Timers ===
    private final ElapsedTime opmodeTimer = new ElapsedTime();

    // === Target Pose (SAVED TO BRAIN: (90, 90) with heading 222°) ===
    private final Pose targetScoringPose = new Pose(90, 90, Math.toRadians(222));

    // === Pathing ===
    private Follower follower;
    private boolean pathStarted = false;
    private PathChain goToScoringPath;

    // === Logging ===
    private void log(String caption, Object value) {
        telemetry.addData(caption, value);
    }

    // ==============================================================
    //  OpMode Lifecycle
    // ==============================================================

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.75);  // Adjust as needed for safety/testing

        opmodeTimer.reset();

        // Build the path ONCE in init – it will use the current pose (from odometry) as start
        buildPathToScoring();

        log("Status", "INIT: Ready – Place robot anywhere, it will path to scoring pose");
        log("Target Pose", targetScoringPose);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Optional: Log current pose estimate for verification before start
        Pose currentPose = follower.getPose();
        double normH = Math.toDegrees((currentPose.getHeading() + 2 * Math.PI) % (2 * Math.PI));
        log("Current Pose (est.)", String.format("(%.2f, %.2f, %.2f°)", currentPose.getX(), currentPose.getY(), normH));
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        // Start following the path from current pose to target
        follower.followPath(goToScoringPath, true);
        pathStarted = true;
        log("Status", "START: Pathing to scoring pose");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();

        if (!pathStarted || follower.isBusy()) {
            // Still following
            log("Path Status", "Following to target");
        } else {
            // Path complete – robot is at target!
            log("Path Status", "ARRIVED AT SCORING POSE!");
        }

        Pose currentPose = follower.getPose();
        double normH = Math.toDegrees((currentPose.getHeading() + 2 * Math.PI) % (2 * Math.PI));

        log("Current X", String.format("%.2f", currentPose.getX()));
        log("Current Y", String.format("%.2f", currentPose.getY()));
        log("Current Heading", String.format("%.2f°", normH));
        log("Target X/Y/H", String.format("(%.2f, %.2f, %.2f°)", targetScoringPose.getX(), targetScoringPose.getY(), Math.toDegrees(targetScoringPose.getHeading())));
        log("Time (s)", String.format("%.2f", opmodeTimer.seconds()));

        telemetry.update();
    }

    @Override
    public void stop() {
        log("Status", "STOP: Done");
    }

    // === Path Building ===
    private void buildPathToScoring() {
        Pose currentPose = follower.getPose();  // Use current estimated pose as start (odometry must be running)

        goToScoringPath = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetScoringPose))  // Straight Bezier from current to target
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetScoringPose.getHeading())  // Smooth heading transition
                .build();

        log("Path Built", "From current to target scoring pose");
    }
}