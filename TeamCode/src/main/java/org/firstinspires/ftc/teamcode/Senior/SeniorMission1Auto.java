
package org.firstinspires.ftc.teamcode;

// Change this import if your Constants class is in another package.
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

import static com.pedropathing.api.Paths.line;

@Autonomous(
        name = "WRO Senior Mission 1",
        group = "WRO"
)
public class SeniorMission1Auto extends OpMode {

//    Pedro 3 path following.
//    Pinpoint localization through your Pedro Constants.
//    HuskyLens green-box detection.
//    Inspection of nine rack units.
//    Pickup and release arm sequence.
//    Delivery of five boxes.
//    Return to the selected Charging Station.

    // =====================================
    // PEDRO
    // =====================================

    private Follower follower;

    private final PoseFactory p = PoseFactory.degrees();

    // =====================================
    // HUSKYLENS V1
    // =====================================

    private HuskyLens huskyLens;

    private boolean cameraReady = false;

    // Green box trained as Color ID1.
    private static final int GREEN_BOX_ID = 1;

    private static final int MIN_BOX_WIDTH = 8;
    private static final int MIN_BOX_HEIGHT = 8;

    private int detectedBoxX = -1;
    private int detectedBoxY = -1;
    private int detectedBoxCount = 0;

    private int stableDetectionFrames = 0;

    // =====================================
    // ROBOT ARM
    // =====================================

    private Servo shoulder;
    private Servo elbow;
    private Servo claw;

    // Example positions only.
    // Calibrate them on your own arm.

    private static final double SHOULDER_STOW = 0.15;
    private static final double SHOULDER_PICK = 0.60;
    private static final double SHOULDER_CARRY = 0.30;
    private static final double SHOULDER_PLACE = 0.55;

    private static final double ELBOW_STOW = 0.20;
    private static final double ELBOW_PICK = 0.75;
    private static final double ELBOW_CARRY = 0.30;
    private static final double ELBOW_PLACE = 0.70;

    private static final double CLAW_OPEN = 0.25;
    private static final double CLAW_CLOSE = 0.70;

    // =====================================
    // FIELD DIMENSIONS
    // =====================================

    private static final double FIELD_WIDTH_MM = 2286.0;
    private static final double FIELD_LENGTH_MM = 2362.0;

    private static final double RACK_DEPTH_MM = 120.0;
    private static final double RACK_LENGTH_MM = 960.0;

    private static final double SIDE_CLEARANCE_MM = 563.0;
    private static final double RACK_GAP_MM = 400.0;

    private static final double RACK_START_Y_MM = 400.0;
    private static final double RACK_UNIT_LENGTH_MM = 330.0;
    private static final double SHARED_POST_MM = 15.0;

    private static final double UNIT_PITCH_MM =
            RACK_UNIT_LENGTH_MM - SHARED_POST_MM;

    // Robot approaches each rack 200mm away
    // from the rack's outer face.
    private static final double APPROACH_GAP_MM = 200.0;

    // Open travel corridor beyond rack ends.
    private static final double TRANSIT_Y_MM = 1600.0;

    // Workbench is at the far end.
    // Adjust based on the physical reach of the arm.
    private static final double BENCH_APPROACH_Y_MM = 2040.0;

    private static final double BENCH_CENTER_X_MM =
            FIELD_WIDTH_MM / 2.0;

    private static final double SLOT_SPACING_MM = 120.0;

    // =====================================
    // CHARGING STATION CONFIGURATION
    // =====================================

    // Coordinate origin: near-left corner.
    // Replace these values with measured robot-centre poses.

    private static final double START_X_MM = 0.0;
    private static final double START_Y_MM = 0.0;
    private static final double START_HEADING_DEG = 0.0;

    private static final double PARK_X_MM = 0.0;
    private static final double PARK_Y_MM = 0.0;
    private static final double PARK_HEADING_DEG = 0.0;

    // Use 200 for the left-side corridor.
    // Use FIELD_WIDTH_MM - 200 for the right-side corridor.
    private static final double START_SIDE_X_MM = 200.0;
    private static final double PARK_SIDE_X_MM = 200.0;

    // Do not enable until the field has been measured.
    private static final boolean FIELD_MAP_CALIBRATED = false;

    // Do not enable until the three arm positions
    // and gripping sequence have been tested.
    private static final boolean ARM_CALIBRATED = false;

    private final Pose START_POSE = p.of(
            mmToIn(START_X_MM),
            mmToIn(START_Y_MM),
            START_HEADING_DEG
    );

    private final Pose PARK_POSE = p.of(
            mmToIn(PARK_X_MM),
            mmToIn(PARK_Y_MM),
            PARK_HEADING_DEG
    );

    // =====================================
    // MISSION VARIABLES
    // =====================================

    private static final int TOTAL_RACKS = 9;
    private static final int TOTAL_BOXES = 5;

    private int rackIndex = 0;
    private int deliveredBoxes = 0;

    private double currentApproachXMM = 0;

    // =====================================
    // TIMERS
    // =====================================

    private final ElapsedTime timer = new ElapsedTime();

    private final ElapsedTime cameraTimer =
            new ElapsedTime();

    // =====================================
    // NAVIGATION
    // =====================================

    private final ArrayDeque<Pose> route =
            new ArrayDeque<>();

    private boolean navigationActive = false;
    private boolean routeFinished = false;

    // =====================================
    // STATES
    // =====================================

    private enum State {

        BLOCKED,
        ERROR,

        TO_RACK,
        SCAN_RACK,

        PICK_PREP,
        PICK_CLOSE,
        PICK_LIFT,

        TO_BENCH,

        PLACE_PREP,
        PLACE_RELEASE,
        STOW_ARM,

        TO_PARK,
        DONE
    }

    private State state = State.BLOCKED;

    // =====================================
    // UNIT CONVERSION
    // =====================================

    private static double mmToIn(double mm) {
        return mm / 25.4;
    }

    // =====================================
    // INITIALIZATION
    // =====================================

    @Override
    public void init() {

        // Create the Pedro follower.
        // Constants must use PinpointLocalizer
        // and your tuned Mecanum configuration.

        follower = Constants.create(hardwareMap);

        follower.setPose(START_POSE);

        // HuskyLens hardware configuration name:
        // huskylens

        huskyLens = hardwareMap.get(
                HuskyLens.class,
                "huskylens"
        );

        cameraReady = huskyLens.knock();

        if (cameraReady) {

            huskyLens.selectAlgorithm(
                    HuskyLens.Algorithm.COLOR_RECOGNITION
            );
        }

        // Arm hardware names.
        // Change these to match your configuration.

        shoulder = hardwareMap.get(
                Servo.class, "armShoulder"
        );

        elbow = hardwareMap.get(
                Servo.class, "armElbow"
        );

        claw = hardwareMap.get(
                Servo.class, "claw"
        );

        telemetry.addLine("WRO Senior Mission 1");
        telemetry.addData("Camera", cameraReady ? "Connected" : "ERROR");
        telemetry.addData("Field width", FIELD_WIDTH_MM);
        telemetry.addData("Field length", FIELD_LENGTH_MM);

        telemetry.update();
    }

    @Override
    public void init_loop() {

        telemetry.addData("HuskyLens", cameraReady);

        telemetry.addLine(
                "Train green box as Color ID1"
        );

        telemetry.addData(
                "Map calibrated",
                FIELD_MAP_CALIBRATED
        );

        telemetry.addData(
                "Arm calibrated",
                ARM_CALIBRATED
        );

        telemetry.update();
    }

    // =====================================
    // START
    // =====================================

    @Override
    public void start() {

        if (!cameraReady ||
                !FIELD_MAP_CALIBRATED ||
                !ARM_CALIBRATED) {

            state = State.BLOCKED;
            return;
        }

        follower.setPose(START_POSE);

        shoulder.setPosition(SHOULDER_STOW);
        elbow.setPosition(ELBOW_STOW);
        claw.setPosition(CLAW_OPEN);

        rackIndex = 0;
        deliveredBoxes = 0;

        stableDetectionFrames = 0;

        goToFirstRack();
    }

    // =====================================
    // MAIN LOOP
    // =====================================

    @Override
    public void loop() {

        // Pedro must update continuously.
        follower.update();

        updateNavigation();

        switch (state) {

            // --------------------------------
            // BLOCKED
            // --------------------------------

            case BLOCKED:

                telemetry.addLine(
                        "Autonomous not enabled"
                );

                telemetry.addLine(
                        "Check map, arm and HuskyLens"
                );

                break;

            case ERROR:

                follower.stop();
                break;

            // --------------------------------
            // MOVE TO RACK
            // --------------------------------

            case TO_RACK:

                if (routeFinished) {

                    routeFinished = false;

                    stableDetectionFrames = 0;

                    timer.reset();
                    cameraTimer.reset();

                    state = State.SCAN_RACK;
                }

                break;

            // --------------------------------
            // SCAN RACK
            // --------------------------------

            case SCAN_RACK:

                // Poll I2C sensor at a controlled rate.

                if (cameraTimer.seconds() >= 0.12) {

                    cameraTimer.reset();

                    boolean found = detectGreenBox();

                    if (found) {

                        stableDetectionFrames++;

                    } else {

                        stableDetectionFrames = 0;
                    }
                }

                // Require multiple stable detections.

                if (stableDetectionFrames >= 3) {

                    beginPickup();

                } else if (timer.seconds() >= 2.0) {

                    // No box at this rack.
                    advanceToNextRack(true);
                }

                break;

            // --------------------------------
            // ARM PICKUP
            // --------------------------------

            case PICK_PREP:

                if (timer.seconds() >= 0.6) {

                    claw.setPosition(CLAW_CLOSE);

                    timer.reset();

                    state = State.PICK_CLOSE;
                }

                break;

            case PICK_CLOSE:

                if (timer.seconds() >= 0.45) {

                    shoulder.setPosition(
                            SHOULDER_CARRY
                    );

                    elbow.setPosition(
                            ELBOW_CARRY
                    );

                    timer.reset();

                    state = State.PICK_LIFT;
                }

                break;

            case PICK_LIFT:

                if (timer.seconds() >= 0.6) {

                    goToWorkbench();
                }

                break;

            // --------------------------------
            // MOVE TO WORKBENCH
            // --------------------------------

            case TO_BENCH:

                if (routeFinished) {

                    routeFinished = false;

                    shoulder.setPosition(
                            SHOULDER_PLACE
                    );

                    elbow.setPosition(
                            ELBOW_PLACE
                    );

                    timer.reset();

                    state = State.PLACE_PREP;
                }

                break;

            // --------------------------------
            // PLACE BOX
            // --------------------------------

            case PLACE_PREP:

                if (timer.seconds() >= 0.6) {

                    claw.setPosition(CLAW_OPEN);

                    timer.reset();

                    state = State.PLACE_RELEASE;
                }

                break;

            case PLACE_RELEASE:

                if (timer.seconds() >= 0.5) {

                    shoulder.setPosition(
                            SHOULDER_STOW
                    );

                    elbow.setPosition(
                            ELBOW_STOW
                    );

                    timer.reset();

                    state = State.STOW_ARM;
                }

                break;

            case STOW_ARM:

                if (timer.seconds() >= 0.6) {

                    deliveredBoxes++;

                    advanceToNextRack(false);
                }

                break;

            // --------------------------------
            // RETURN TO CHARGING STATION
            // --------------------------------

            case TO_PARK:

                if (routeFinished) {

                    routeFinished = false;

                    follower.stop();

                    state = State.DONE;
                }

                break;

            // --------------------------------
            // COMPLETE
            // --------------------------------

            case DONE:

                follower.stop();
                break;
        }

        // =================================
        // TELEMETRY
        // =================================

        telemetry.addData("Mission State", state);

        telemetry.addData(
                "Rack",
                rackIndex < TOTAL_RACKS
                        ? rackIndex + 1 : "Completed"
        );

        telemetry.addData(
                "Boxes Delivered",
                deliveredBoxes
        );

        telemetry.addData(
                "Detection X",
                detectedBoxX
        );

        telemetry.addData(
                "Detection Y",
                detectedBoxY
        );

        telemetry.addData(
                "Detected blocks",
                detectedBoxCount
        );

        telemetry.addData(
                "Pedro X",
                follower.pose().x()
        );

        telemetry.addData(
                "Pedro Y",
                follower.pose().y()
        );

        telemetry.addData(
                "Pedro Heading",
                Math.toDegrees(
                        follower.pose().heading()
                )
        );

        telemetry.addData(
                "Navigation active",
                navigationActive
        );

        telemetry.update();
    }

    // =====================================
    // HUSKYLENS GREEN BOX DETECTION
    // =====================================

    private boolean detectGreenBox() {

        try {

            HuskyLens.Block[] blocks =
                    huskyLens.blocks();

            detectedBoxCount = blocks.length;

            int largestArea = 0;

            HuskyLens.Block best = null;

            for (HuskyLens.Block block : blocks) {

                if (block.id != GREEN_BOX_ID) {
                    continue;
                }

                if (block.width < MIN_BOX_WIDTH ||
                        block.height < MIN_BOX_HEIGHT) {
                    continue;
                }

                int area =
                        block.width * block.height;

                if (area > largestArea) {

                    largestArea = area;
                    best = block;
                }
            }

            if (best != null) {

                detectedBoxX = best.x;
                detectedBoxY = best.y;

                return true;
            }

            detectedBoxX = -1;
            detectedBoxY = -1;

            return false;

        } catch (RuntimeException e) {

            telemetry.addData(
                    "HuskyLens read error",
                    e.getMessage()
            );

            return false;
        }
    }

    // =====================================
    // RACK COORDINATES
    // =====================================

    // Rack numbers:
    // 1,2,3 = first row
    // 4,5,6 = second row
    // 7,8,9 = third row
    //
    // Confirm this mapping against Figure 14
    // and the orientation of your physical field.

    private double rackApproachX(int index) {

        int row = index / 3;

        double rackCenterX =
                SIDE_CLEARANCE_MM +
                        RACK_DEPTH_MM / 2.0 +
                        row * (RACK_DEPTH_MM + RACK_GAP_MM);

        return rackCenterX -
                RACK_DEPTH_MM / 2.0 -
                APPROACH_GAP_MM;
    }

    private double rackUnitY(int index) {

        int unit = index % 3;

        double firstUnitCenter =
                RACK_START_Y_MM +
                        RACK_UNIT_LENGTH_MM / 2.0;

        return firstUnitCenter +
                unit * UNIT_PITCH_MM;
    }

    private Pose rackPose(int index) {

        double x = rackApproachX(index);
        double y = rackUnitY(index);

        currentApproachXMM = x;

        return p.of(
                mmToIn(x),
                mmToIn(y),
                0
        );
    }

    // =====================================
    // CORRIDOR POSES
    // =====================================

    private Pose centerTransitPose() {

        return p.of(
                mmToIn(BENCH_CENTER_X_MM),
                mmToIn(TRANSIT_Y_MM),
                0
        );
    }

    private Pose sideTransitPose(
            double sideX
    ) {

        return p.of(
                mmToIn(sideX),
                mmToIn(TRANSIT_Y_MM),
                0
        );
    }

    // =====================================
    // FIRST RACK
    // =====================================

    private void goToFirstRack() {

        List<Pose> points = new ArrayList<>();

        // Travel along the starting side aisle.
        points.add(
                sideTransitPose(START_SIDE_X_MM)
        );

        // Enter the clear corridor.

        points.add(centerTransitPose());

        // Enter rack aisle and approach rack 1.

        points.add(
                p.of(
                        mmToIn(rackApproachX(0)),
                        mmToIn(TRANSIT_Y_MM),
                        0
                )
        );

        points.add(rackPose(0));

        beginRoute(points, State.TO_RACK);
    }

    // =====================================
    // MOVE TO NEXT RACK
    // =====================================

    private void advanceToNextRack(
            boolean currentlyAtRack
    ) {

        rackIndex++;

        if (deliveredBoxes >= TOTAL_BOXES ||
                rackIndex >= TOTAL_RACKS) {

            goToChargingStation(currentlyAtRack);
            return;
        }

        List<Pose> points = new ArrayList<>();

        if (currentlyAtRack) {

            // Leave the current rack first.
            points.add(
                    p.of(
                            mmToIn(currentApproachXMM),
                            mmToIn(TRANSIT_Y_MM),
                            0
                    )
            );
        }

        // Move along the safe corridor.

        points.add(centerTransitPose());

        // Enter next rack aisle.

        points.add(
                p.of(
                        mmToIn(rackApproachX(rackIndex)),
                        mmToIn(TRANSIT_Y_MM),
                        0
                )
        );

        points.add(rackPose(rackIndex));

        beginRoute(points, State.TO_RACK);
    }

    // =====================================
    // ARM PICKUP
    // =====================================

    private void beginPickup() {

        claw.setPosition(CLAW_OPEN);

        shoulder.setPosition(SHOULDER_PICK);

        elbow.setPosition(ELBOW_PICK);

        timer.reset();

        state = State.PICK_PREP;
    }

    // =====================================
    // WORKBENCH NAVIGATION
    // =====================================

    private void goToWorkbench() {

        List<Pose> points = new ArrayList<>();

        // Leave rack aisle.

        points.add(
                p.of(
                        mmToIn(currentApproachXMM),
                        mmToIn(TRANSIT_Y_MM),
                        0
                )
        );

        // Move through the corridor.

        points.add(centerTransitPose());

        // Calculate a separate box placement slot.

        double slotX =
                BENCH_CENTER_X_MM +
                        (deliveredBoxes - 2) * SLOT_SPACING_MM;

        // Turn towards the Workbench.

        points.add(
                p.of(
                        mmToIn(slotX),
                        mmToIn(TRANSIT_Y_MM),
                        90
                )
        );

        // Final approach.

        points.add(
                p.of(
                        mmToIn(slotX),
                        mmToIn(BENCH_APPROACH_Y_MM),
                        90
                )
        );

        beginRoute(points, State.TO_BENCH);
    }

    // =====================================
    // RETURN TO CHARGING STATION
    // =====================================

    private void goToChargingStation(
            boolean currentlyAtRack
    ) {

        List<Pose> points = new ArrayList<>();

        if (currentlyAtRack) {

            points.add(
                    p.of(
                            mmToIn(currentApproachXMM),
                            mmToIn(TRANSIT_Y_MM),
                            0
                    )
            );
        }

        points.add(centerTransitPose());

        points.add(
                sideTransitPose(PARK_SIDE_X_MM)
        );

        points.add(PARK_POSE);

        beginRoute(points, State.TO_PARK);
    }

    // =====================================
    // PEDRO ROUTE MANAGER
    // =====================================

    private void beginRoute(
            List<Pose> targets,
            State nextState
    ) {

        route.clear();

        route.addAll(targets);

        navigationActive = true;

        routeFinished = false;

        state = nextState;

        startNextPath();
    }

    private void startNextPath() {

        while (!route.isEmpty()) {

            Pose start = follower.pose();

            Pose end = route.removeFirst();

            double distance = Math.hypot(
                    end.x() - start.x(),
                    end.y() - start.y()
            );

            if (distance < 0.25) {
                continue;
            }

            // Pedro 3 line path with heading interpolation.

            follower.follow(
                    line(start, end).linear(start, end)
            );

            return;
        }

        navigationActive = false;

        routeFinished = true;
    }

    private void updateNavigation() {

        if (!navigationActive) {
            return;
        }

        if (!follower.isBusy()) {

            startNextPath();
        }
    }

    // =====================================
    // STOP
    // =====================================

    @Override
    public void stop() {

        if (follower != null) {

            follower.stop();

            follower.update();
        }
    }
}
