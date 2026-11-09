package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardware;

@Autonomous(name = "Auto da JSON")
public class AutoFromJson extends LinearOpMode {

    private static final String PATH_FILE = "path.json";

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------------------------------------
        // HARDWARE
        // ---------------------------------------------

        RobotHardware robot = new RobotHardware();

        robot.init(hardwareMap, telemetry);

        // Bulk caching
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.AUTO
            );
        }

        // ---------------------------------------------
        // LOAD JSON
        // ---------------------------------------------

        RobotPath path;

        try {

            path = PathLoader.load(PATH_FILE);

        } catch (Exception e) {

            telemetry.addData(
                    "Errore nel percorso",
                    e.toString()
            );

            telemetry.update();

            waitForStart();

            return;
        }

        // ---------------------------------------------
        // OBJECT INITIALIZATION
        // ---------------------------------------------

        PositionFinder finder = new PositionFinder(
                robot,
                path.startX,
                path.startY,
                path.startHeadingDeg
        );

        RobotActions actions = new RobotActions(robot);

        PathFollower follower = new PathFollower(
                robot,
                finder
        );

        AutoRunner runner = new AutoRunner(
                finder,
                follower,
                actions,
                telemetry
        );


        String missing = actions.missingConfig();

        while (!isStarted() && !isStopRequested()) {

            finder.update();

            telemetry.addData(
                    "Percorso",
                    "%d segmenti, %.1f s",
                    path.segments.size(),
                    path.matchDurationSec
            );

            telemetry.addData(
                    "Start atteso",
                    "x=%.1f y=%.1f heading=%.1f",
                    path.startX,
                    path.startY,
                    path.startHeadingDeg
            );

            telemetry.addData(
                    "Pinpoint",
                    robot.pinpoint.getDeviceStatus()
            );

            telemetry.addData(
                    "Pos letta",
                    "x=%.1f y=%.1f h=%.1f",
                    finder.getX(),
                    finder.getY(),
                    finder.getDirection()
            );

            if (!missing.isEmpty()) {
                telemetry.addData(
                        "Meccanismi NON configurati",
                        missing
                );
            }

            telemetry.update();
        }

        if (isStopRequested()) {
            return;
        }

        // ---------------------------------------------
        // AUTONOMOUS EXECUTION
        // ---------------------------------------------

        runner.start(path);

        long lastTelemetryNs = 0;

        while (opModeIsActive() && runner.update()) {

            long now = System.nanoTime();

            // Telemetry ogni 200 ms
            if (now - lastTelemetryNs > 200_000_000L) {

                lastTelemetryNs = now;

                telemetry.addData(
                        "Stato",
                        runner.getStatus()
                );

                telemetry.addData(
                        "Tempo",
                        "%.1f s",
                        runner.elapsedSec()
                );

                telemetry.addData(
                        "Posizione",
                        "x=%.1f y=%.1f heading=%.1f",
                        finder.getX(),
                        finder.getY(),
                        finder.getDirection()
                );

                telemetry.addData(
                        "Errore finale",
                        "%.1f cm, %.1f gradi",
                        follower.lastEndDist,
                        follower.lastHeadErrDeg
                );

                telemetry.update();
            }
        }

        // ---------------------------------------------
        // STOP
        // ---------------------------------------------

        runner.stop();

        telemetry.addData(
                "Fine",
                "x=%.1f y=%.1f heading=%.1f",
                finder.getX(),
                finder.getY(),
                finder.getDirection()
        );

        telemetry.update();
    }
}