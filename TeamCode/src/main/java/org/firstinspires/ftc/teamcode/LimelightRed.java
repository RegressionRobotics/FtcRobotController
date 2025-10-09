package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.lang.reflect.Method;
import java.util.List;

@Autonomous(name = "Red Auto Limelight", group = "DECODE")
public class LimelightRed extends LinearOpMode {

    // --- Drive & mech hardware (same names as your TeleOp) ---
    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private DcMotor leftVert, rightVert;
    private DcMotor leftHorz, rightHorz;
    private CRServo intake;      // continuous rotation intake (teleop used CRServo)
    private Servo transfer;      // transfer servo (teleop name)

    // Optional shooter hardware (if you have them configured). If not present, scoring routine still runs with transfer only.
    private DcMotor shooterMotor = null;
    private Servo shooterServo = null;

    // Limelight
    private Limelight3A limelight;

    // Pinpoint (will be discovered via reflection at runtime)
    private Object pinpointDevice = null; // the Pinpoint driver instance (unknown concrete class)
    private Method pinpoint_getX = null;
    private Method pinpoint_getY = null;
    private Method pinpoint_getHeading = null;
    private Method pinpoint_isReady = null;

    // --- Field / tile math constants ---
    private static final double TILE_IN = 24.0;

    // Starting tile: B2 center -> col 2, row 2
    private static final double START_X = (2 - 0.5) * TILE_IN; // 36.0 in
    private static final double START_Y = (2 - 0.5) * TILE_IN; // 36.0 in

    // SPIKE seam V (between columns A & B) center X
    private static final double SPIKE_V_X = 24.0; // in (seam between A and B)

    // Spike rows Y coordinates (center of the tile) estimate but i looked at pdf I think these are right from depot
    private static final double SPIKE_ROW2_Y = (2 - 0.5) * TILE_IN; // 36.0 in (NEAR)
    private static final double SPIKE_ROW3_Y = (3 - 0.5) * TILE_IN; // 60.0 in (MIDDLE)
    private static final double SPIKE_ROW4_Y = (4 - 0.5) * TILE_IN; // 84.0 in (FAR)

    // Obelisk tag IDs
    private static final int OBELISK_GPP = 21; // near spike
    private static final int OBELISK_PGP = 22; // middle spike (default)
    private static final int OBELISK_PPG = 23; // far spike

    // Limelight red goal (kept for later use if desired)
    private static final int RED_GOAL_APRILTAG_ID = 24;

    // Encoder / drivetrain constants - Im guesssin ghere salil tune these

    private static final double COUNTS_PER_MOTOR_REV = 537.7; // or your motor's counts-per-rev
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double DRIVE_GEAR_RATIO = 1.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Movement tuning
    private static final double DRIVE_POWER = 0.5;
    private static final double STRAFE_POWER = 0.4;

    // Intake/collector timings
    private static final long INTAKE_RUN_MS = 2000; // run intake 2 seconds to collect artifacts

    // State (set by vision)
    private String detectedMotif = "PGP"; // default middle spike if nothing seen
    private double targetSpikeY = SPIKE_ROW3_Y;

    @Override
    public void runOpMode() {
        // --- hardware init ---
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        leftVert  = hardwareMap.get(DcMotor.class, "leftVert");
        rightVert = hardwareMap.get(DcMotor.class, "rightVert");

        leftHorz  = hardwareMap.get(DcMotor.class, "leftHorz");
        rightHorz = hardwareMap.get(DcMotor.class, "rightHorz");

        intake = hardwareMap.get(CRServo.class, "intake");
        transfer = hardwareMap.get(Servo.class, "transfer");

        // optional shooter hardware — if not in robot config, try/catch will ignore
        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
            shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        } catch (Exception e) {
            // shooter not present — scoring will fall back to transfer only
            shooterMotor = null;
            shooterServo = null;
        }

        // set left-side motors reversed (same as your teleop)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // brakes by using reverse thrust
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run using encoder mode for accurate RUN_TO_POSITION later
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize limelight (AprilTag pipeline expected as pipeline 0)
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            if (limelight != null) {
                limelight.setPollRateHz(100);
                limelight.pipelineSwitch(0);
                limelight.start();
                telemetry.addLine("Limelight initialized");
            } else {
                telemetry.addLine("No limelight found (will default)");
            }
        } catch (Exception e) {
            limelight = null;
            telemetry.addData("Limelight init err", e.getMessage());
        }

        // IDK where to find Pinpoint in hardware map via reflection (so we don't require a specific compile-time driver)
        discoverPinpointViaReflection();

        telemetry.addLine("Ready — press START");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // Step 1: scan Obelisk motif (IDs 21/22/23)
        decodeObeliskMotif();

        // Step 2: leave launch zone (drive forward ~20 in)
        driveStraightInches(20.0, DRIVE_POWER);

        // Step 3: move to spike seam and row using tile math
        goToSpikeByMotif();

        // Step 4: run intake to collect artifacts
        intake.setPower(1.0);
        sleep(INTAKE_RUN_MS);
        intake.setPower(0.0);

        // Step 5: back up and drive to general shooting area and score
        driveStraightInches(-12.0, DRIVE_POWER); // back up 12"
        driveStraightInches(18.0, DRIVE_POWER);  // drive toward goal (tweak as needed)
        scoreArtifacts();

        telemetry.addLine("AUTO COMPLETE");
        telemetry.update();
    }


    // Vision: read obelisk motif via Limelight

    private void decodeObeliskMotif() {
        telemetry.addLine("Scanning Obelisk (2.5s)...");
        telemetry.update();

        boolean found = false;
        long start = System.currentTimeMillis();
        while (opModeIsActive() && !found && (System.currentTimeMillis() - start < 2500)) {
            if (limelight == null) break;
            LLResult res = limelight.getLatestResult();
            if (res != null && res.isValid()) {
                // Many Limelight SDK versions expose getFiducialResults() so salil if this dosent work you can try this there is also a video on it so ask me ill give it to you.
                List<LLResultTypes.FiducialResult> list = null;
                try {
                    list = res.getFiducialResults();
                } catch (Exception e) {
                    // If  SDK uses a different name, this will be null and we'll fall back to default.
                    list = null;
                }
                if (list != null) {
                    for (LLResultTypes.FiducialResult fr : list) {
                        //3 spike rows we test for
                        if (fr == null) continue;
                        int id = fr.getFiducialId();
                        if (id == OBELISK_GPP) {
                            detectedMotif = "GPP";
                            targetSpikeY = SPIKE_ROW2_Y; // near
                            found = true;
                            break;
                        } else if (id == OBELISK_PGP) {
                            detectedMotif = "PGP";
                            targetSpikeY = SPIKE_ROW3_Y; // middle
                            found = true;
                            break;
                        } else if (id == OBELISK_PPG) {
                            detectedMotif = "PPG";
                            targetSpikeY = SPIKE_ROW4_Y; // far
                            found = true;
                            break;
                        }
                    }
                }
            }
            sleep(50);
        }

        if (!found) {
            // default so if the limlight doesnet work out it will default to row 3 of the thing
            detectedMotif = "PGP";
            targetSpikeY = SPIKE_ROW3_Y;
            telemetry.addLine("No motif found — defaulting to PGP (middle)");
        } else {
            telemetry.addData("Motif", detectedMotif);
        }
        telemetry.update();
    }


    // Navigation using tile math and pins/encoders.

    private void goToSpikeByMotif() {
        double targetX = SPIKE_V_X; // seam V
        double targetY = targetSpikeY;

        double dx = targetX - START_X; // usually -12 (left)
        double dy = targetY - START_Y; // forward amount to that row deriviative jsut to get poitns and make this thing move forward if all else fails

        telemetry.addData("Spike target", "%s (X=%.1f Y=%.1f)", detectedMotif, targetX, targetY);
        telemetry.addData("dx,dy", "%.1f, %.1f", dx, dy);
        // If Pinpoint is init and ready, show its live reading before moving
        if (pinpointDevice != null) {
            double[] pose = readPinpointPoseSafely();
            if (pose != null) {
                telemetry.addData("Pinpoint pose (in,deg)", "%.1f,%.1f,%.1f", pose[0] * 39.37, pose[1] * 39.37, pose[2]);
            } else {
                telemetry.addLine("Pinpoint present but pose read failed");
            }
        } else {
            telemetry.addLine("Pinpoint not available, using encoder tile math");
        }
        telemetry.update();

        // Strafe horizontally by dx (positive -> right, negative -> left)
        if (Math.abs(dx) > 0.5) {
            strafeInches(dx, STRAFE_POWER);
            sleep(120);
        }

        // Drive forward by dy/dx by moving both motors with postive ratio
        if (Math.abs(dy) > 0.5) {
            driveStraightInches(dy, DRIVE_POWER);
            sleep(120);
        }
        //Completed move towards the row and is ready to suck balls into intake
        telemetry.addLine("Arrived at spike seam/row");
        telemetry.update();
    }


    // Scoring: transfer/shooter control

    private void scoreArtifacts() {
        telemetry.addLine("Scoring (transfer + shooter)");
        telemetry.update();

        // Spin up shooter after intake collection
        if (shooterMotor != null) {
            shooterMotor.setPower(1.0);
            sleep(800);
        }

        // use transfer servo to push each artifact into the shooter motor
        final int shots = 3;
        for (int i = 0; i < shots && opModeIsActive(); i++) {
            transfer.setPosition(0.25); // push position — tune if needed
            sleep(300);
            transfer.setPosition(0.9);  // stowed position
            sleep(350);
        }

        if (shooterMotor != null) {
            shooterMotor.setPower(0.0);
        }

        telemetry.addLine("Scoring done");
        telemetry.update();
    }


    // Encoder-based helpers (RUN_TO_POSITION)

    private void driveStraightInches(double inches, double power) {
        if (!opModeIsActive()) return;
        int moveCounts = (int) Math.round(inches * COUNTS_PER_INCH);

        // compute new targets relative to current positions if bot is way off center make it calculate positon and read in the apriltags to go into the nessacary position idk if this works properly tho
        int lfTarget = frontLeft.getCurrentPosition() + moveCounts;
        int lbTarget = backLeft.getCurrentPosition() + moveCounts;
        int rfTarget = frontRight.getCurrentPosition() + moveCounts;
        int rbTarget = backRight.getCurrentPosition() + moveCounts;

        // set targets
        frontLeft.setTargetPosition(lfTarget);
        backLeft.setTargetPosition(lbTarget);
        frontRight.setTargetPosition(rfTarget);
        backRight.setTargetPosition(rbTarget);

        // run to position
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double p = Math.abs(power);
        frontLeft.setPower(p);
        backLeft.setPower(p);
        frontRight.setPower(p);
        backRight.setPower(p);

        // wait for completion (or abort on stop) had to watch video on this
        while (opModeIsActive() &&
                (frontLeft.isBusy() || backLeft.isBusy() || frontRight.isBusy() || backRight.isBusy())) {
            telemetry.addData("Remaining (ticks)", "LF %d RF %d LB %d RB %d",
                    Math.abs(frontLeft.getTargetPosition() - frontLeft.getCurrentPosition()),
                    Math.abs(frontRight.getTargetPosition() - frontRight.getCurrentPosition()),
                    Math.abs(backLeft.getTargetPosition() - backLeft.getCurrentPosition()),
                    Math.abs(backRight.getTargetPosition() - backRight.getCurrentPosition()));
            telemetry.update();
            sleep(20);
        }

        // stop motors and restore run-using mode
        stopDriveMotors();
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Strafe for mecanum: positive inches -> right, negative -> left.
     * Implementation sets opposing wheel targets so robot strafes in place.
     */
    private void strafeInches(double inches, double power) {
        if (!opModeIsActive()) return;
        int moveCounts = (int) Math.round(inches * COUNTS_PER_INCH);

        int lfTarget = frontLeft.getCurrentPosition() + moveCounts;
        int lbTarget = backLeft.getCurrentPosition() - moveCounts;
        int rfTarget = frontRight.getCurrentPosition() - moveCounts;
        int rbTarget = backRight.getCurrentPosition() + moveCounts;

        frontLeft.setTargetPosition(lfTarget);
        backLeft.setTargetPosition(lbTarget);
        frontRight.setTargetPosition(rfTarget);
        backRight.setTargetPosition(rbTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double p = Math.abs(power);
        frontLeft.setPower(p);
        backLeft.setPower(p);
        frontRight.setPower(p);
        backRight.setPower(p);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || backLeft.isBusy() || frontRight.isBusy() || backRight.isBusy())) {
            sleep(10);
        }

        stopDriveMotors();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopDriveMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }


    // Pinpoint reflective discovery & reads

    /**
     * Attempt to discover the Pinpoint device class in hardwareMap using a set of
     * driver class names. Alr salil so change this Idk if this is right
     **/
    private void discoverPinpointViaReflection() {
        String[] candidateClassNames = new String[] {
                // Common possibilities from goBILDA / community drivers (may vary by package)
                "com.gobilda.hardware.GoBildaPinpointDriver",
                "com.gobilda.PinpointDriver",
                "com.gobilda.hardware.PinpointDriver",
                "GoBildaPinpointDriver",
                "PinpointDriver",
                "com.gobilda.hardware.GoBILDApinpoint.GoBildaPinpointDriver", // other variants
        };

        for (String className : candidateClassNames) {
            try {
                Class<?> clazz = Class.forName(className);
                // hardwareMap.get requires a Class<T>, raw Class works when used as Class
                Object dev = hardwareMap.get(clazz, "Pinpoint");
                if (dev != null) {
                    pinpointDevice = dev;
                    // Try to find getX, getY, getHeading (most drivers name them something like this)
                    try { pinpoint_getX = clazz.getMethod("getX"); } catch (Exception e) { pinpoint_getX = null; }
                    try { pinpoint_getY = clazz.getMethod("getY"); } catch (Exception e) { pinpoint_getY = null; }
                    try { pinpoint_getHeading = clazz.getMethod("getHeading"); } catch (Exception e) { pinpoint_getHeading = null; }
                    try { pinpoint_isReady = clazz.getMethod("isReady"); } catch (Exception e) { pinpoint_isReady = null; }
                    telemetry.addData("Pinpoint driver loaded", className);
                    telemetry.update();
                    return;
                }
            } catch (ClassNotFoundException cnfe) {
                // try next candidate
            } catch (Exception ex) {
                // hardwareMap.get might crash for some variants — ignore and continue
            }
        }

        // Extranous
        pinpointDevice = null;
        telemetry.addLine("Pinpoint not discovered reflectively; continuing with encoders");
        telemetry.update();
    }

    /**
     * Read Pinpoint pose safely with reflection.
     * Returns an array {xMeters, yMeters, headingDegrees} or null if reading failed.
     */

    //Everything past here is idk wtf is going on idk if this works properly
    private double[] readPinpointPoseSafely() {
        if (pinpointDevice == null) return null;

        try {
            // optional isReady check
            if (pinpoint_isReady != null) {
                Object readyObj = pinpoint_isReady.invoke(pinpointDevice);
                if (readyObj instanceof Boolean) {
                    if (!((Boolean) readyObj)) {
                        return null; // not ready
                    }
                }
            }

            double x = 0.0, y = 0.0, heading = 0.0;

            if (pinpoint_getX != null) {
                Object ox = pinpoint_getX.invoke(pinpointDevice);
                if (ox instanceof Number) x = ((Number) ox).doubleValue();
            }
            if (pinpoint_getY != null) {
                Object oy = pinpoint_getY.invoke(pinpointDevice);
                if (oy instanceof Number) y = ((Number) oy).doubleValue();
            }
            if (pinpoint_getHeading != null) {
                Object oh = pinpoint_getHeading.invoke(pinpointDevice);
                if (oh instanceof Number) heading = ((Number) oh).doubleValue();
            }

            return new double[] { x, y, heading };

        } catch (Exception e) {
            // reflection failed; treat pinpoint as unavailable
            return null;
        }
    }
}
