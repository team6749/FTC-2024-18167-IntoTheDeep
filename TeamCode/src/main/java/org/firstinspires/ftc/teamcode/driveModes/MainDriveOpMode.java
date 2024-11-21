package org.firstinspires.ftc.teamcode.driveModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RobotArm;
import org.firstinspires.ftc.teamcode.hardware.RobotClaw;
import org.firstinspires.ftc.teamcode.subsystems.CraneSystem;

@TeleOp(name="MAIN", group="Iterative OpMode")
public class MainDriveOpMode extends OpMode {
    SampleMecanumDrive drive;
    RobotArm robotArm;
    boolean fastMode = true;
    double SLOW_MODE_STICK_DIVISOR = 3;
    int loops = 0;

    @Override
    public void init() {
        robotArm = new RobotArm(hardwareMap);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap, telemetry);


    }


    @Override
    public void loop() {
loops++;
        telemetry.addData("loops", loops);

        driveCommands();
//        Pose2d poseEstimate = drive.getPoseEstimate();
//        telemetry.addData("x", poseEstimate.getX());
//        telemetry.addData("y", poseEstimate.getY());
//        telemetry.addData("heading", poseEstimate.getHeading());
//
//        if (gamepad1.a || gamepad1.x) {
//            automationCommands();
//        } else {
//            armCommands();
//            telemetry.addData("arm angle", robotArm.getCurrentRotationPosition());
//            telemetry.addData("arm extension", robotArm.getCurrentExtensionPosition());
//
//            wristCommands();
//            telemetry.addData("wrist position", robotArm.getCurrentWristPosition());
//
//            clawCommands();
//            telemetry.addData("claw position", robotArm.isClawOpen() ? "OPEN" : "CLOSED");
//        }

    }

    private void automationCommands() {
        if (gamepad1.x) {
            robotArm.driveMode();
        } else if (gamepad1.a && gamepad1.right_trigger > 0) {
            robotArm.toHighBasket();
        }
    }
    private void armCommands() {

            if (gamepad1.dpad_up) {
                robotArm.raiseArm();
            } else if (gamepad1.dpad_down) {
                robotArm.lowerArm();
            }

            if (gamepad1.right_bumper) {
                robotArm.extendArm();
            } else if (gamepad1.left_bumper) {
                robotArm.retractArm();
            }

    }

    private void wristCommands() {
        boolean operationAllowed = true;
        if (gamepad1.right_trigger > 0) {
            operationAllowed = robotArm.rotateWristRight();
        } else if (gamepad1.left_trigger > 0) {
            operationAllowed = robotArm.rotateWristLeft();
        } else {
           robotArm.rotateWristCenter();
        }
        if (!operationAllowed) {
            gamepad1.rumble(30);

            telemetry.addData("DANGER_ZONE", "WRIST MOVEMENT NOT AVAIL");
        }
    }

    private void clawCommands() {
        //Claw Movement
        if (gamepad1.y) {
            robotArm.openClaw();
        }
        if (gamepad1.b){
            robotArm.closeClaw();
        }
    }

    private void driveCommands() {
        if (gamepad1.back) {
            fastMode = !fastMode;
        }
        Pose2d weightedStickPose;
        if (fastMode) {
            weightedStickPose = new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );
        } else {
            weightedStickPose = new Pose2d(
                    -gamepad1.left_stick_y / SLOW_MODE_STICK_DIVISOR,
                    -gamepad1.left_stick_x / SLOW_MODE_STICK_DIVISOR,
                    -gamepad1.right_stick_x / SLOW_MODE_STICK_DIVISOR
            );
        }
        telemetry.addData("fastSlow", fastMode ? "fast" : "SLOW");
        telemetry.addData("stickX", gamepad1.left_stick_x);
        telemetry.addData("stickY", gamepad1.left_stick_y);
        telemetry.addData("weightedX", weightedStickPose.getX());
        telemetry.addData("weightedY", weightedStickPose.getY());

        drive.setWeightedDrivePower(weightedStickPose);
        drive.update();
    }
}