package hell;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class FirstBot extends LinearOpMode {
    private static double SCALE = 1.0;
    private static double BALL_RADIUS = 20*SCALE;
    private static double FALLOS_WIDTH = 40*SCALE;
    private static double FALLOS_HEIGHT = 80*SCALE;

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(0, 0, Math.PI / 2);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        Trajectory fallosTrajectory = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0, BALL_RADIUS, 1.5 * Math.PI))
                .splineTo(new Vector2d(-BALL_RADIUS, 0), Math.PI)
                .splineTo(new Vector2d(-2 * BALL_RADIUS, BALL_RADIUS),Math.PI / 2)
                .splineTo(new Vector2d(-BALL_RADIUS, 2 * BALL_RADIUS), 0)
                .splineTo(new Vector2d(BALL_RADIUS, 0), 0)
                .splineTo(new Vector2d(2 * BALL_RADIUS, BALL_RADIUS), Math.PI / 2)
                .splineTo(new Vector2d(BALL_RADIUS, 2 * BALL_RADIUS), Math.PI)
                .splineTo(new Vector2d(0, BALL_RADIUS), 1.5 * Math.PI)

                .lineToLinearHeading(new Pose2d(0, BALL_RADIUS, Math.PI))
                .splineTo(new Vector2d(-FALLOS_WIDTH /2, FALLOS_HEIGHT / 2 + BALL_RADIUS), Math.PI / 2)
                .splineTo(new Vector2d(0, FALLOS_HEIGHT + BALL_RADIUS), 0)
                .splineTo(new Vector2d(FALLOS_WIDTH /2, FALLOS_HEIGHT / 2 + BALL_RADIUS), 1.5 * Math.PI)
                .splineTo(new Vector2d(0, BALL_RADIUS), Math.PI)

                .build();

        Trajectory reverseParking = drive.trajectoryBuilder(fallosTrajectory.end(), true)
                .splineTo(new Vector2d(0, 0), Math.PI / 2)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(fallosTrajectory);
        drive.followTrajectory(reverseParking);
    }
}