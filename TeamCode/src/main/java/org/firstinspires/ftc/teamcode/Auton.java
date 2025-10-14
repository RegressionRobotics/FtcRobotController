package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.HashMap;
import java.util.Map;

// added imports for limelight 3a
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

// added imports for Pedro Pathing // comment: imports for path following
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
// import removed: com.pedropathing.localization.ThreeDeadWheelLocalizer
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

// To use Pedro Pathing, add to TeamCode/build.gradle in dependencies:
// implementation 'com.pedropathing:ftc:2.0.0' // replace with latest version
// implementation 'com.pedropathing:telemetry:1.0.0'
// Then sync Gradle

@Autonomous(name="DECODE Auto with Limelight and Pedro Pathing", group="Competition")
public class Auton extends LinearOpMode {

    // Drive Motors  // comment: these are the motors for driving the robot
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;

    // Game Mechanism Motors  // comment: motors for shooting and intake
    private DcMotorEx shooter;
    private DcMotorEx intake;

    // Servos  // comment: servos for transferring stuff
    private Servo leftTransfer;
    private Servo rightTransfer;

    // Limelight 3A  // comment: camera for vision
    private Limelight3AWrapper limelight;

    // Timer  // comment: timer for runtime
    private ElapsedTime runtime = new ElapsedTime();

    // Alliance Selection  // comment: blue or red team
    private Alliance alliance = Alliance.BLUE;

    // Pedro Pathing components // comment: follower for pathing
    private Follower follower;

    // Dead wheel encoders for localization // comment: encoders for tracking position
    private DcMotorEx parallelLeft;
    private DcMotorEx parallelRight;
    private DcMotorEx perpendicular;

    // Field Positions (in inches from origin)
    // Starting positions based on white triangles at field edges  // comment: starting points for blue and red
    private static final double BLUE_START_X = 12.0;  // Blue alliance starts on left side
    private static final double BLUE_START_Y = 12.0;  // Near audience wall
    private static final double RED_START_X = 132.0;  // Red alliance starts on right side
    private static final double RED_START_Y = 12.0;   // Near audience wall

    // Spike Mark positions (center of tiles, measured from field origin)  // comment: positions for spikes on field
    private static final Map<Alliance, double[][]> SPIKE_POSITIONS = new HashMap<>();
    static {
        // Blue alliance spike marks (left side of field)  // comment: blue spikes
        SPIKE_POSITIONS.put(Alliance.BLUE, new double[][] {
                {36.0, 48.0},  // Spike 1 (tiles A2/B2)
                {36.0, 72.0},  // Spike 2 (tiles A3/B3)
                {36.0, 96.0}   // Spike 3 (tiles A4/B4)
        });

        // Red alliance spike marks (right side of field)   // comment: red spikes
        SPIKE_POSITIONS.put(Alliance.RED, new double[][] {
                {108.0, 48.0}, // Spike 1 (tiles E2/F2)
                {108.0, 72.0}, // Spike 2 (tiles E3/F3)
                {108.0, 96.0}  // Spike 3 (tiles E4/F4)
        });
    }

    // Goal positions for shooting  // comment: where to shoot for blue and red
    private static final double BLUE_GOAL_X = 24.0;
    private static final double BLUE_GOAL_Y = 132.0;
    private static final double RED_GOAL_X = 120.0;
    private static final double RED_GOAL_Y = 132.0;

    // Robot Constants  // comment: robot math stuff
    private static final double COUNTS_PER_MOTOR_REV = 537.7;  // GoBILDA 312 RPM
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Dead wheel constants // comment: for localization
    private static final double TRACK_WIDTH = 14.0; // robot track width in inches
    private static final double PERP_OFFSET = 0.0; // perpendicular wheel offset

    // Shooter Constants  // comment: power for shooter
    private static final double SHOOTER_BASE_POWER = 0.7;
    private static final double SHOOTER_MAX_POWER = 1.0;

    // Assume starting heading // comment: robot starting heading in radians
    private static final double START_HEADING = 0.0;

    enum Alliance {
        BLUE, RED
    }

    @Override
    public void runOpMode() {
        // Initialize hardware  // comment: setup all hardware
        initializeHardware();

        // Initialize Limelight  // comment: setup camera
        limelight = new Limelight3AWrapper(hardwareMap, telemetry);
        limelight.setPipeline(0); // AprilTag detection pipeline

        // Initialize Pedro Pathing follower
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);

        // Alliance selection on init  // comment: choose team with gamepad
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) {
                alliance = Alliance.BLUE;
            } else if (gamepad1.b) {
                alliance = Alliance.RED;
            }
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Press X for BLUE, B for RED", "");
            telemetry.addData("Status", "Initialized");
            telemetry.update();
            idle();
        }

        waitForStart();
        runtime.reset();

        // Set starting pose for Pedro Pathing // comment: set initial position
        Pose startPose = (alliance == Alliance.BLUE) ?
                new Pose(BLUE_START_X, BLUE_START_Y, START_HEADING) :
                new Pose(RED_START_X, RED_START_Y, START_HEADING);
        follower.setStartingPose(startPose);

        if (opModeIsActive()) {
            // Execute autonomous sequence  // comment: run the auto plan
            executeAutonomous();
        }
    }

    private void initializeHardware() {
        // Initialize drive motors  // comment: get motors from hardware
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // Set motor directions (adjust based on your robot)  // comment: set how motors turn
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set motor modes  // comment: reset and use encoders
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize dead wheel encoders // comment: get encoders for localization
        parallelLeft = hardwareMap.get(DcMotorEx.class, "parallelLeft");
        parallelRight = hardwareMap.get(DcMotorEx.class, "parallelRight");
        perpendicular = hardwareMap.get(DcMotorEx.class, "perpendicular");

        // Set encoder directions // comment: adjust as needed
        parallelLeft.setDirection(DcMotor.Direction.REVERSE);
        parallelRight.setDirection(DcMotor.Direction.FORWARD);
        perpendicular.setDirection(DcMotor.Direction.FORWARD);

        // Set encoder modes // comment: reset and run without power
        parallelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicular.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        parallelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        parallelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        perpendicular.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ...existing code...

        // Initialize game mechanisms  // comment: get shooter and intake
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        leftTransfer = hardwareMap.get(Servo.class, "leftTransfer");
        rightTransfer = hardwareMap.get(Servo.class, "rightTransfer");

        // Set initial servo positions  // comment: start positions for servos
        leftTransfer.setPosition(0.0);
        rightTransfer.setPosition(1.0);
    }

    private void executeAutonomous() {
        // Step 1: Move from starting position to view obelisk  // comment: go to see the tag using Pedro path
        moveToObeliskView();

        // Step 2: Read AprilTag on obelisk to determine which spike mark to target  // comment: check the id with limelight
        int targetSpike = readObeliskAprilTag();

        // added pause after limelight id recognition before going to spike mark
        sleep(1000);  // pause for 1 second

        // Step 3: Navigate to the appropriate spike mark  // comment: go to the right spike using Pedro
        navigateToSpikeMark(targetSpike);

        // Step 4: Pick up artifacts (balls) from spike mark  // comment: collect stuff
        collectArtifacts();

        // Step 5: Navigate to shooting position  // comment: go to shoot spot using Pedro
        moveToShootingPosition();

        // Step 6: Calculate distance and shoot artifacts into goal  // comment: aim and fire
        aimAndShoot();

        // Step 7: Park in designated area (optional end game)  // comment: park the robot using Pedro
        parkRobot();
    }

    private void moveToObeliskView() {
        telemetry.addData("Status", "Moving to view obelisk");
        telemetry.update();

        // Define target pose for obelisk view // comment: calculate target based on alliance
        double targetX = (alliance == Alliance.BLUE) ? 36.0 : 108.0;
        double targetY = 48.0;
        Pose targetPose = new Pose(targetX, targetY, START_HEADING);

        // Build and follow path // comment: use Bezier line for straight path
        PathChain pathChain = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
                .build();

        follower.followPath(pathChain);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }

        sleep(500); // Allow camera to stabilize  // comment: wait for camera
    }

    private int readObeliskAprilTag() {
        telemetry.addData("Status", "Reading obelisk AprilTag");
        telemetry.update();

        // Read AprilTag ID from Limelight  // comment: get the tag id
        limelight.updateTracking();
        int aprilTagId = limelight.getTargetAprilTagId();

        // Map AprilTag ID to spike mark (1, 2, or 3)
        // IDs 21, 22, 23 correspond to different motifs  // comment: decide which spike based on id
        int spikeIndex = 0;
        switch (aprilTagId) {
            case 21:  // GPP motif
                spikeIndex = 0;
                break;
            case 22:  // PGP motif
                spikeIndex = 1;
                break;
            case 23:  // PPG motif
                spikeIndex = 2;
                break;
            default:
                spikeIndex = 1; // Default to middle spike if no tag detected
                telemetry.addData("Warning", "No valid AprilTag detected, using default");
        }

        telemetry.addData("AprilTag ID", aprilTagId);
        telemetry.addData("Target Spike", spikeIndex + 1);
        telemetry.update();

        return spikeIndex;
    }

    private void navigateToSpikeMark(int spikeIndex) {
        telemetry.addData("Status", "Navigating to spike mark " + (spikeIndex + 1));
        telemetry.update();

        double[][] spikePositions = SPIKE_POSITIONS.get(alliance);
        double targetX = spikePositions[spikeIndex][0];
        double targetY = spikePositions[spikeIndex][1];
        Pose targetPose = new Pose(targetX, targetY, START_HEADING);

        // Build and follow path to spike // comment: use Bezier line
        PathChain pathChain = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
                .build();

        follower.followPath(pathChain);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }

    private void collectArtifacts() {
        telemetry.addData("Status", "Collecting artifacts");
        telemetry.update();

        // Open transfer servos  // comment: open to collect
        leftTransfer.setPosition(0.5);
        rightTransfer.setPosition(0.5);
        sleep(300);

        // Run intake to collect artifacts  // comment: turn on intake
        intake.setPower(1.0);

        // Move slowly forward while collecting  // comment: small move, keep simple
        moveForward(6, 0.3);
        sleep(1000); // Allow time for collection

        // Close transfer servos to secure artifacts  // comment: close to hold
        leftTransfer.setPosition(0.0);
        rightTransfer.setPosition(1.0);
        sleep(300);

        // Stop intake  // comment: stop intake
        intake.setPower(0);
    }

    private void moveToShootingPosition() {
        telemetry.addData("Status", "Moving to shooting position");
        telemetry.update();

        // Define shooting pose // comment: based on alliance
        double targetX = (alliance == Alliance.BLUE) ? BLUE_GOAL_X : RED_GOAL_X;
        double targetY = BLUE_GOAL_Y; // same for both
        Pose targetPose = new Pose(targetX, targetY, START_HEADING);

        // Build and follow path // comment: to shooting position
        PathChain pathChain = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
                .build();

        follower.followPath(pathChain);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }

    private void aimAndShoot() {
        telemetry.addData("Status", "Aiming and shooting");
        telemetry.update();

        // Use Limelight to detect goal and calculate distance  // comment: switch pipeline for goal
        limelight.setPipeline(1); // Switch to goal detection pipeline
        sleep(500);

        limelight.updateTracking();
        double distance = calculateDistanceToGoal();
        double shooterPower = calculateShooterPower(distance);

        telemetry.addData("Distance to goal", distance);
        telemetry.addData("Shooter power", shooterPower);
        telemetry.update();

        // Align robot with goal using Limelight feedback  // comment: turn to target
        alignWithGoal();

        // added pause after turning to correct target before shooting
        sleep(1000);  // pause for 1 second

        // Spin up shooter  // comment: start shooter
        shooter.setPower(shooterPower);
        sleep(1500); // Allow shooter to reach speed

        // Transfer artifacts to shooter  // comment: move artifacts
        leftTransfer.setPosition(0.7);
        rightTransfer.setPosition(0.3);
        sleep(500);

        // Run intake to feed artifacts  // comment: feed and shoot
        intake.setPower(0.5);
        sleep(2000); // Time to shoot all artifacts

        // added pause after shooting
        sleep(1000);  // pause for 1 second

        // Stop all mechanisms  // comment: stop everything
        shooter.setPower(0);
        intake.setPower(0);
        leftTransfer.setPosition(0.0);
        rightTransfer.setPosition(1.0);
    }

    private double calculateDistanceToGoal() {
        // Use Limelight data to calculate distance  // comment: math for distance
        double ty = limelight.getTargetY(); // Vertical offset angle
        double mountHeight = 8.0; // Camera mount height in inches
        double goalHeight = 24.0; // Goal height in inches
        double mountAngle = 15.0; // Camera mount angle in degrees

        // Calculate distance using trigonometry
        double totalAngle = mountAngle + ty;
        double heightDiff = goalHeight - mountHeight;
        double distance = heightDiff / Math.tan(Math.toRadians(totalAngle));

        return Math.max(12.0, Math.min(distance, 72.0)); // Clamp between min/max
    }

    private double calculateShooterPower(double distance) {
        // Linear interpolation for shooter power based on distance  // comment: figure power from distance
        double minDist = 12.0;
        double maxDist = 72.0;
        double minPower = 0.5;
        double maxPower = 1.0;

        double power = minPower + (distance - minDist) * (maxPower - minPower) / (maxDist - minDist);
        return Math.max(minPower, Math.min(power, maxPower));
    }

    private void alignWithGoal() {
        double tx = limelight.getTargetX(); // Horizontal offset angle
        double tolerance = 2.0; // degrees

        while (Math.abs(tx) > tolerance && opModeIsActive()) {
            double turnPower = tx * 0.03; // P controller  // comment: simple control to turn
            turnPower = Math.max(-0.3, Math.min(turnPower, 0.3)); // Limit turn speed

            // Turn to align  // comment: set powers to turn
            frontLeft.setPower(-turnPower);
            backLeft.setPower(-turnPower);
            frontRight.setPower(turnPower);
            backRight.setPower(turnPower);

            sleep(50);
            limelight.updateTracking();
            tx = limelight.getTargetX();
        }

        // Stop turning  // comment: stop motors
        stopMotors();
    }

    private void parkRobot() {
        telemetry.addData("Status", "Parking robot");
        telemetry.update();

        // Define park pose // comment: example park position, adjust as needed
        double parkX = (alliance == Alliance.BLUE) ? 12.0 : 132.0;
        double parkY = 12.0;
        Pose parkPose = new Pose(parkX, parkY, START_HEADING);

        // Build and follow path to park // comment: use Pedro for parking
        PathChain pathChain = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), parkPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), parkPose.getHeading())
                .build();

        follower.followPath(pathChain);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }

    // Movement helper methods  // comment: kept for small moves, but main use Pedro
    private void moveForward(double inches) {
        moveForward(inches, 0.6);
    }

    private void moveForward(double inches, double power) {
        int target = (int)(inches * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + target);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + target);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + target);
        backRight.setTargetPosition(backRight.getCurrentPosition() + target);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(power);

        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Moving", "Forward %.1f inches", inches);
            telemetry.update();
        }

        stopMotors();
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void moveBackward(double inches) {
        moveForward(-inches);
    }

    private void strafeRight(double inches) {
        int target = (int)(inches * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + target);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - target);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - target);
        backRight.setTargetPosition(backRight.getCurrentPosition() + target);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(0.6);

        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Moving", "Strafe right %.1f inches", inches);
            telemetry.update();
        }

        stopMotors();
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void strafeLeft(double inches) {
        strafeRight(-inches);
    }

    private void turnRight(double degrees) {
        double inches = (degrees / 360.0) * Math.PI * 14.0; // Robot width ~14 inches
        int target = (int)(inches * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + target);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + target);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - target);
        backRight.setTargetPosition(backRight.getCurrentPosition() - target);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(0.5);

        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Turning", "Right %.1f degrees", degrees);
            telemetry.update();
        }

        stopMotors();
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void turnLeft(double degrees) {
        turnRight(-degrees);
    }

    // Utility methods  // comment: helper functions
    private void setMotorMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        backLeft.setMode(mode);
        frontRight.setMode(mode);
        backRight.setMode(mode);
    }

    private void setMotorPower(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    private void stopMotors() {
        setMotorPower(0);
    }

    private boolean motorsAreBusy() {
        return frontLeft.isBusy() || backLeft.isBusy() ||
                frontRight.isBusy() || backRight.isBusy();
    }
}

// Limelight 3A wrapper class  // comment: class for limelight control
class Limelight3AWrapper {
    private com.qualcomm.hardware.limelightvision.Limelight3A ll;
    private org.firstinspires.ftc.robotcore.external.Telemetry telemetry;
    private com.qualcomm.hardware.limelightvision.LLResult result;

    public Limelight3AWrapper(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
                              org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        this.telemetry = telemetry;
        this.ll = hardwareMap.get(com.qualcomm.hardware.limelightvision.Limelight3A.class, "limelight");
        ll.pipelineSwitch(0); // Start with AprilTag pipeline
        ll.start();
    }

    public void setPipeline(int pipeline) {
        ll.pipelineSwitch(pipeline);
    }

    public void updateTracking() {
        result = ll.getLatestResult();
    }

    public double getTargetX() {
        if (result != null && result.isValid()) {
            return result.getTx();
        }
        return 0.0;
    }

    public double getTargetY() {
        if (result != null && result.isValid()) {
            return result.getTy();
        }
        return 0.0;
    }

    public int getTargetAprilTagId() {
        if (result != null && result.isValid()) {
            java.util.List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (fiducialResults != null && !fiducialResults.isEmpty()) {
                return fiducialResults.get(0).getFiducialId();
            }
        }
        return -1; // No tag found
    }

    public boolean hasTarget() {
        return result != null && result.isValid();
    }
}