
package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static com.pedropathing.api.Paths.line;

@Autonomous(
        name = "WRO Senior Mission 2",
        group = "WRO"
)
public class SeniorMission2Auto extends OpMode {
//    Read five cards.
//    Validate all five rack numbers.
//    Switch HuskyLens to green-box detection.
//    Collect only the requested racks.
//    Return to the Charging Station.


    // =========================================
    // PEDRO
    // =========================================

    private Follower follower;

    private final PoseFactory p = PoseFactory.degrees();

    // =========================================
    // HUSKYLENS
    // =========================================

    private HuskyLens huskyLens;

    private boolean cameraReady = false;

    private static final int BOX_COLOR_ID = 1;

    // =========================================
    // ROBOT ARM
    // =========================================

    private Servo shoulder;
    private Servo elbow;
    private Servo claw;

    // Calibrate these values for your actual arm.

    private static final double SHOULDER_STOW = 0.15;
    private static final double SHOULDER_PICK = 0.60;
    private static final double SHOULDER_CARRY = 0.30;
    private static final double SHOULDER_PLACE = 0.55;

    private static final double ELBOW_STOW = 0.20;
    private static final double ELBOW_PICK = 0.75;
    private static final double ELBOW_CARRY = 0.30;
    private static final double ELBOW_PLACE = 0.70;

    private static final double CLAW_OPEN = 0.25;
    private static final double CLAW_CLOSED = 0.70;

    // =========================================
    // FIELD DIMENSIONS
    // =========================================

    private static final double FIELD_WIDTH_MM = 2286.0;
    private static final double FIELD_LENGTH_MM = 2362.0;

    private static final double TRANSIT_Y_MM = 1700.0;

    private static final double BENCH_CENTER_X_MM = 1143.0;

    // Tune after measuring your actual robot
    // approach to the Workbench.

    private static final double BENCH_APPROACH_Y_MM = 2100.0;

    private static final double BENCH_SLOT_SPACING_MM = 130.0;

    // =========================================
    // RACK COORDINATES
    // =========================================

    // Derived from the official field layout.
    // Assumes the field origin is near-left.

    private static final double[] RACK_ROW_CENTERS_X_MM = {
            623.0,
            1143.0,
            1663.0
    };

    private static final double[] RACK_UNIT_CENTERS_Y_MM = {
            565.0,
            880.0,
            1195.0
    };

    private static final double RACK_DEPTH_MM = 120.0;

    // Centre of robot is kept outside the rack.
    // The actual gap depends on robot dimensions
    // and the reach of your arm.

    private static final double RACK_APPROACH_GAP_MM = 200.0;

    // =========================================
    // START AND FINISH POSES
    // =========================================

    // Replace with measured robot-centre coordinates.
    // Units: millimetres and degrees.

    private static final double[] START_MM = {
            0, 0, 0
    };

    private static final double[] PARK_MM = {
            0, 0, 0
    };

    // =========================================
    // ORDER AREA POSES
    // =========================================

    // IMPORTANT:
    // Enter the measured centres of Order Areas 1-5.
    //
    // Each row = {X in mm, Y in mm, heading in degrees}
    //
    // The zero values below are placeholders.
    // Area 6 is intentionally not used.

    private static final double[][] ORDER_AREA_MM = {

            {0, 0, 0},  // Order Area 1
            {0, 0, 0},  // Order Area 2
            {0, 0, 0},  // Order Area 3
            {0, 0, 0},  // Order Area 4
            {0, 0, 0}   // Order Area 5

    };

    private Pose startPose;
    private Pose parkPose;

    private final Pose[] orderAreaPoses = new Pose[5];

    // =========================================
    // CALIBRATION SWITCHES
    // =========================================

    private static final boolean FIELD_MAP_CALIBRATED = false;

    private static final boolean ORDER_AREAS_CALIBRATED = false;

    private static final boolean ARM_CALIBRATED = false;

    private static final boolean CAMERA_CALIBRATED = false;

    // =========================================
    // MISSION VARIABLES
    // =========================================

    private static final int REQUIRED_CARDS = 5;

    private final int[] requestedRackIds =
            new int[REQUIRED_CARDS];

    private final boolean[] seenRackIds =
            new boolean[10];

    private int cardIndex = 0;

    private int targetIndex = 0;

    private int deliveredBoxes = 0;

    private int currentRackId = 0;

    // =========================================
    // CAMERA DATA
    // =========================================

    private int lastCardCandidate = 0;

    private int stableCardFrames = 0;

    private int stableBoxFrames = 0;

    private int detectedBoxX = -1;

    private int detectedBoxY = -1;

    private int detectedBoxCount = 0;

    private static final int MIN_BOX_WIDTH = 8;
    private static final int MIN_BOX_HEIGHT = 8;

    // =========================================
    // TIMERS
    // =========================================

    private final ElapsedTime timer = new ElapsedTime();

    private final ElapsedTime cameraTimer = new ElapsedTime();

    // =========================================
    // NAVIGATION QUEUE
    // =========================================

    private final ArrayDeque<Pose> route =
            new ArrayDeque<>();

    private boolean navigationActive = false;

    private boolean routeFinished = false;

    // =========================================
    // MISSION STATES
    // =========================================

    private enum State {

        BLOCKED,
        FAULT,

        MOVE_TO_CARD,
        READ_CARD,

        SWITCH_TO_COLOR,

        MOVE_TO_RACK,
        SCAN_RACK,

        ARM_DOWN,
        GRIP_BOX,
        LIFT_BOX,

        MOVE_TO_BENCH,

        LOWER_BOX,
        RELEASE_BOX,
        STOW_ARM,

        MOVE_TO_PARK,

        DONE
    }

    private State state = State.BLOCKED;

    // =========================================
    // UNIT CONVERSION
    // =========================================

    private static double mmToIn(double mm) {

        return mm / 25.4;
    }

    private Pose poseMM(
            double x,
            double y,
            double heading
    ) {

        return p.of(
                mmToIn(x),
                mmToIn(y),
                heading
        );
    }

    // =========================================
    // INITIALIZATION
    // =========================================

    @Override
    public void init() {

        // Pedro creation.
        // Constants must use PinpointLocalizer
        // and Mecanum drivetrain configuration.

        follower = Constants.create(hardwareMap);

        // Starting and parking poses.

        startPose = poseMM(
                START_MM[0],
                START_MM[1],
                START_MM[2]
        );

        parkPose = poseMM(
                PARK_MM[0],
                PARK_MM[1],
                PARK_MM[2]
        );

        // Order Area Poses.

        for (int i = 0; i < 5; i++) {

            orderAreaPoses[i] = poseMM(
                    ORDER_AREA_MM[i][0],
                    ORDER_AREA_MM[i][1],
                    ORDER_AREA_MM[i][2]
            );
        }

        follower.setPose(startPose);

        follower.update();

        // HuskyLens V1.
        // Robot Configuration name: huskylens.

        huskyLens = hardwareMap.get(
                HuskyLens.class,
                "huskylens"
        );

        cameraReady = huskyLens.knock();

        if (cameraReady) {

            huskyLens.selectAlgorithm(
                    HuskyLens.Algorithm.OBJECT_CLASSIFICATION
            );
        }

        // Robotic arm configuration.

        shoulder = hardwareMap.get(
                Servo.class,
                "armShoulder"
        );

        elbow = hardwareMap.get(
                Servo.class,
                "armElbow"
        );

        claw = hardwareMap.get(
                Servo.class,
                "claw"
        );

        telemetry.addLine("WRO Senior Mission 2");
        telemetry.addData("HuskyLens", cameraReady);
        telemetry.addData("Field width", FIELD_WIDTH_MM);
        telemetry.addData("Field length", FIELD_LENGTH_MM);

        telemetry.addLine(
                "Object Classification selected"
        );

        telemetry.update();
    }

    // =========================================
    // INIT LOOP
    // =========================================

    @Override
    public void init_loop() {

        telemetry.addData(
                "Camera",
                cameraReady ? "Connected" : "ERROR"
        );

        telemetry.addData(
                "Field calibrated",
                FIELD_MAP_CALIBRATED
        );

        telemetry.addData(
                "Order areas calibrated",
                ORDER_AREAS_CALIBRATED
        );

        telemetry.addData(
                "Arm calibrated",
                ARM_CALIBRATED
        );

        telemetry.addData(
                "Camera calibrated",
                CAMERA_CALIBRATED
        );

        telemetry.update();
    }

    // =========================================
    // START
    // =========================================

    @Override
    public void start() {

        if (!cameraReady ||
                !FIELD_MAP_CALIBRATED ||
                !ORDER_AREAS_CALIBRATED ||
                !ARM_CALIBRATED ||
                !CAMERA_CALIBRATED) {

            state = State.BLOCKED;
            return;
        }

        follower.setPose(startPose);

        shoulder.setPosition(SHOULDER_STOW);

        elbow.setPosition(ELBOW_STOW);

        claw.setPosition(CLAW_OPEN);

        cardIndex = 0;

        targetIndex = 0;

        deliveredBoxes = 0;

        Arrays.fill(requestedRackIds, 0);

        Arrays.fill(seenRackIds, false);

        // Read the first dot card.

        goToOrderArea(0);
    }

    // =========================================
    // MAIN LOOP
    // =========================================

    @Override
    public void loop() {

        // Pedro must update continuously.

        follower.update();

        updateNavigation();

        switch (state) {

            // ---------------------------------
            // WAIT FOR CALIBRATION
            // ---------------------------------

            case BLOCKED:

                telemetry.addLine(
                        "Autonomous blocked"
                );

                telemetry.addLine(
                        "Complete calibration first"
                );

                break;

            // ---------------------------------
            // FAULT
            // ---------------------------------

            case FAULT:

                telemetry.addLine(
                        "MISSION FAULT - STOPPED"
                );

                break;

            // ---------------------------------
            // MOVE TO ORDER AREA
            // ---------------------------------

            case MOVE_TO_CARD:

                if (routeFinished) {

                    routeFinished = false;

                    stableCardFrames = 0;

                    lastCardCandidate = 0;

                    timer.reset();

                    cameraTimer.reset();

                    state = State.READ_CARD;
                }

                break;

            // ---------------------------------
            // READ DOT CARD
            // ---------------------------------

            case READ_CARD:

                if (cameraTimer.seconds() >= 0.15) {

                    cameraTimer.reset();

                    int detectedId = readDotCard();

                    if (detectedId >= 1 &&
                            detectedId <= 9) {

                        if (detectedId == lastCardCandidate) {

                            stableCardFrames++;

                        } else {

                            lastCardCandidate = detectedId;

                            stableCardFrames = 1;
                        }

                    } else {

                        lastCardCandidate = 0;

                        stableCardFrames = 0;
                    }
                }

                // Three stable reads are required.

                if (stableCardFrames >= 3) {

                    acceptCard(lastCardCandidate);

                } else if (timer.seconds() >= 3.0) {

                    // Do not guess a rack number.

                    state = State.FAULT;
                }

                break;

            // ---------------------------------
            // SWITCH ALGORITHM
            // ---------------------------------

            case SWITCH_TO_COLOR:

                if (timer.seconds() >= 0.5) {

                    targetIndex = 0;

                    goToRequestedRack();
                }

                break;

            // ---------------------------------
            // MOVE TO REQUESTED RACK
            // ---------------------------------

            case MOVE_TO_RACK:

                if (routeFinished) {

                    routeFinished = false;

                    timer.reset();

                    cameraTimer.reset();

                    stableBoxFrames = 0;

                    state = State.SCAN_RACK;
                }

                break;

            // ---------------------------------
            // BOX DETECTION
            // ---------------------------------

            case SCAN_RACK:

                if (cameraTimer.seconds() >= 0.12) {

                    cameraTimer.reset();

                    if (detectGreenBox()) {

                        stableBoxFrames++;

                    } else {

                        stableBoxFrames = 0;
                    }
                }

                if (stableBoxFrames >= 3) {

                    beginPickup();

                } else if (timer.seconds() >= 2.5) {

                    // A requested box was not found.
                    // Stop rather than visit an incorrect rack.

                    state = State.FAULT;
                }

                break;

            // ---------------------------------
            // ARM LOWER
            // ---------------------------------

            case ARM_DOWN:

                if (timer.seconds() >= 0.65) {

                    claw.setPosition(CLAW_CLOSED);

                    timer.reset();

                    state = State.GRIP_BOX;
                }

                break;

            // ---------------------------------
            // GRIP BOX
            // ---------------------------------

            case GRIP_BOX:

                if (timer.seconds() >= 0.45) {

                    shoulder.setPosition(
                            SHOULDER_CARRY
                    );

                    elbow.setPosition(
                            ELBOW_CARRY
                    );

                    timer.reset();

                    state = State.LIFT_BOX;
                }

                break;

            // ---------------------------------
            // LIFT BOX
            // ---------------------------------

            case LIFT_BOX:

                if (timer.seconds() >= 0.6) {

                    goToWorkbench();
                }

                break;

            // ---------------------------------
            // MOVE TO WORKBENCH
            // ---------------------------------

            case MOVE_TO_BENCH:

                if (routeFinished) {

                    routeFinished = false;

                    shoulder.setPosition(
                            SHOULDER_PLACE
                    );

                    elbow.setPosition(
                            ELBOW_PLACE
                    );

                    timer.reset();

                    state = State.LOWER_BOX;
                }

                break;

            // ---------------------------------
            // LOWER BOX
            // ---------------------------------

            case LOWER_BOX:

                if (timer.seconds() >= 0.6) {

                    claw.setPosition(CLAW_OPEN);

                    timer.reset();

                    state = State.RELEASE_BOX;
                }

                break;

            // ---------------------------------
            // RELEASE BOX
            // ---------------------------------

            case RELEASE_BOX:

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

            // ---------------------------------
            // STOW ARM
            // ---------------------------------

            case STOW_ARM:

                if (timer.seconds() >= 0.6) {

                    deliveredBoxes++;

                    targetIndex++;

                    if (deliveredBoxes >= REQUIRED_CARDS) {

                        goToParkingStation();

                    } else {

                        goToRequestedRack();
                    }
                }

                break;

            // ---------------------------------
            // PARK
            // ---------------------------------

            case MOVE_TO_PARK:

                if (routeFinished) {

                    routeFinished = false;

                    state = State.DONE;
                }

                break;

            // ---------------------------------
            // FINISH
            // ---------------------------------

            case DONE:

                break;
        }

        // =====================================
        // TELEMETRY
        // =====================================

        telemetry.addData(
                "Mission State",
                state
        );

        telemetry.addData(
                "Card Position",
                cardIndex + " / 5"
        );

        telemetry.addData(
                "Read Rack IDs",
                Arrays.toString(requestedRackIds)
        );

        telemetry.addData(
                "Current Requested Rack",
                currentRackId
        );

        telemetry.addData(
                "Delivered Boxes",
                deliveredBoxes
        );

        telemetry.addData(
                "HuskyLens Blocks",
                detectedBoxCount
        );

        telemetry.addData(
                "Green Box X",
                detectedBoxX
        );

        telemetry.addData(
                "Green Box Y",
                detectedBoxY
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

        telemetry.update();
    }

    // =========================================
    // READ ONE DOT CARD
    // =========================================

    private int readDotCard() {

        try {

            HuskyLens.Block[] blocks =
                    huskyLens.blocks();

            // Each classification result has a learned ID.
            // The card is positioned at the image centre.

            for (HuskyLens.Block block : blocks) {

                if (block.id >= 1 &&
                        block.id <= 9) {

                    return block.id;
                }
            }

            return 0;

        } catch (RuntimeException e) {

            telemetry.addData(
                    "Card read error",
                    e.getMessage()
            );

            return 0;
        }
    }

    // =========================================
    // STORE CARD NUMBER
    // =========================================

    private void acceptCard(int rackId) {

        // The five cards must be unique.

        if (rackId < 1 || rackId > 9) {

            state = State.FAULT;
            return;
        }

        if (seenRackIds[rackId]) {

            // Duplicate card detected.
            // Do not continue with an invalid order.

            state = State.FAULT;
            return;
        }

        seenRackIds[rackId] = true;

        requestedRackIds[cardIndex] = rackId;

        cardIndex++;

        if (cardIndex >= REQUIRED_CARDS) {

            // All five cards have been read.

            huskyLens.selectAlgorithm(
                    HuskyLens.Algorithm.COLOR_RECOGNITION
            );

            timer.reset();

            state = State.SWITCH_TO_COLOR;

        } else {

            goToOrderArea(cardIndex);
        }
    }

    // =========================================
    // GREEN BOX DETECTION
    // =========================================

    private boolean detectGreenBox() {

        try {

            HuskyLens.Block[] blocks =
                    huskyLens.blocks();

            detectedBoxCount = blocks.length;

            int largestArea = 0;

            HuskyLens.Block best = null;

            for (HuskyLens.Block block : blocks) {

                if (block.id != BOX_COLOR_ID) {
                    continue;
                }

                if (block.width < MIN_BOX_WIDTH ||
                        block.height < MIN_BOX_HEIGHT) {
                    continue;
                }

                int area = block.width * block.height;

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
                    "Box detection error",
                    e.getMessage()
            );

            return false;
        }
    }

    // =========================================
    // MOVE TO ORDER AREA
    // =========================================

    private void goToOrderArea(int index) {

        Pose current = follower.pose();

        Pose currentTransit = p.of(
                current.x(),
                mmToIn(TRANSIT_Y_MM),
                0
        );

        Pose centerTransit = poseMM(
                BENCH_CENTER_X_MM,
                TRANSIT_Y_MM,
                0
        );

        Pose cardTransit = p.of(
                orderAreaPoses[index].x(),
                mmToIn(TRANSIT_Y_MM),
                orderAreaPoses[index].heading()
        );

        beginRoute(
                State.MOVE_TO_CARD,
                currentTransit,
                centerTransit,
                cardTransit,
                orderAreaPoses[index]
        );
    }

    // =========================================
    // RACK NUMBER TO RACK POSE
    // =========================================

    private double rackApproachX(int rackId) {

        int row = (rackId - 1) / 3;

        double rackCenterX =
                RACK_ROW_CENTERS_X_MM[row];

        // Approach from the negative-X side.

        return rackCenterX -
                RACK_DEPTH_MM / 2.0 -
                RACK_APPROACH_GAP_MM;
    }

    private double rackPositionY(int rackId) {

        int unit = (rackId - 1) % 3;

        return RACK_UNIT_CENTERS_Y_MM[unit];
    }

    private Pose getRackPose(int rackId) {

        return poseMM(
                rackApproachX(rackId),
                rackPositionY(rackId),
                0
        );
    }

    // =========================================
    // MOVE TO REQUESTED RACK
    // =========================================

    private void goToRequestedRack() {

        if (targetIndex >= REQUIRED_CARDS) {

            goToParkingStation();

            return;
        }

        currentRackId =
                requestedRackIds[targetIndex];

        Pose current = follower.pose();

        Pose currentTransit = p.of(
                current.x(),
                mmToIn(TRANSIT_Y_MM),
                0
        );

        Pose centerTransit = poseMM(
                BENCH_CENTER_X_MM,
                TRANSIT_Y_MM,
                0
        );

        Pose rackTransit = poseMM(
                rackApproachX(currentRackId),
                TRANSIT_Y_MM,
                0
        );

        Pose rack = getRackPose(currentRackId);

        beginRoute(
                State.MOVE_TO_RACK,
                currentTransit,
                centerTransit,
                rackTransit,
                rack
        );
    }

    // =========================================
    // PICK BOX
    // =========================================

    private void beginPickup() {

        claw.setPosition(CLAW_OPEN);

        shoulder.setPosition(SHOULDER_PICK);

        elbow.setPosition(ELBOW_PICK);

        timer.reset();

        state = State.ARM_DOWN;
    }

    // =========================================
    // MOVE TO WORKBENCH
    // =========================================

    private void goToWorkbench() {

        Pose current = follower.pose();

        double slotX =
                BENCH_CENTER_X_MM +
                        (deliveredBoxes - 2) *
                                BENCH_SLOT_SPACING_MM;

        Pose currentTransit = p.of(
                current.x(),
                mmToIn(TRANSIT_Y_MM),
                0
        );

        Pose centerTransit = poseMM(
                BENCH_CENTER_X_MM,
                TRANSIT_Y_MM,
                0
        );

        Pose slotTransit = poseMM(
                slotX,
                TRANSIT_Y_MM,
                90
        );

        Pose bench = poseMM(
                slotX,
                BENCH_APPROACH_Y_MM,
                90
        );

        beginRoute(
                State.MOVE_TO_BENCH,
                currentTransit,
                centerTransit,
                slotTransit,
                bench
        );
    }

    // =========================================
    // RETURN TO CHARGING STATION
    // =========================================

    private void goToParkingStation() {

        Pose current = follower.pose();

        Pose currentTransit = p.of(
                current.x(),
                mmToIn(TRANSIT_Y_MM),
                0
        );

        Pose centerTransit = poseMM(
                BENCH_CENTER_X_MM,
                TRANSIT_Y_MM,
                0
        );

        Pose parkTransit = p.of(
                parkPose.x(),
                mmToIn(TRANSIT_Y_MM),
                parkPose.heading()
        );

        beginRoute(
                State.MOVE_TO_PARK,
                currentTransit,
                centerTransit,
                parkTransit,
                parkPose
        );
    }

    // =========================================
    // PEDRO ROUTE MANAGER
    // =========================================

    private void beginRoute(
            State nextState,
            Pose... targets
    ) {

        route.clear();

        for (Pose target : targets) {

            route.addLast(target);
        }

        routeFinished = false;

        navigationActive = true;

        state = nextState;

        startNextLeg();
    }

    private void updateNavigation() {

        if (navigationActive &&
                !follower.isBusy()) {

            startNextLeg();
        }
    }

    private void startNextLeg() {

        while (!route.isEmpty()) {

            Pose start = follower.pose();

            Pose end = route.removeFirst();

            double distance = Math.hypot(
                    end.x() - start.x(),
                    end.y() - start.y()
            );

            // Skip duplicate waypoints.

            if (distance < 0.25) {
                continue;
            }

            Path path = line(start, end)
                    .linear(start, end);

            follower.follow(path);

            return;
        }

        navigationActive = false;

        routeFinished = true;
    }
}
