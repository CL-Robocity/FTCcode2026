package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Odometry.PositionFinder;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardware;

@Autonomous(name = "Auto da JSON")
public class AutoFromJson extends LinearOpMode {

    private static final String PATH_FILE = "test.json"; // in /sdcard/FIRST/paths/

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap, telemetry);

        RobotPath path;
        try {
            path = PathLoader.load(PATH_FILE);
        } catch (Exception e) {
            telemetry.addData("Errore nel percorso", e.toString());
            telemetry.update();
            waitForStart();
            return;
        }

        // posizione e heading di partenza arrivano dal JSON
        PositionFinder finder = new PositionFinder(robot, path.startX, path.startY, path.startHeadingDeg);

        telemetry.addData("Percorso", "%d punti, %.0f cm", path.points.size(), path.length());
        telemetry.update();

        waitForStart();

        PathFollower follower = new PathFollower(robot, finder);
        follower.run(path, this::opModeIsActive);

        telemetry.addData("Fine", "x=%.1f y=%.1f dir=%.1f", finder.getX(), finder.getY(), finder.getDirection());
        telemetry.update();
    }
}
