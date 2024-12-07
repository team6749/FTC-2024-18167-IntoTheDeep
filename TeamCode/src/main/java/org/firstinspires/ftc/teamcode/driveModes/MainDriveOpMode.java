package org.firstinspires.ftc.teamcode.driveModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RobotArm;
import org.firstinspires.ftc.teamcode.hardware.RobotClaw;
import org.firstinspires.ftc.teamcode.subsystems.CraneSystem;

public class MainDriveOpMode extends OpMode {
    SampleMecanumDrive drive;
    RobotArm robotArm;
    boolean fastMode = true;
    double SLOW_MODE_STICK_DIVISOR = 3;
    private boolean backButtonPressedLast = false; // Tracks the previous state of the back button

    private long lastToggleTime = 0;
    private static final long DEBOUNCE_DELAY_MS = 200;
    int loops = 0;
    boolean isBlueAlliance = true;

    public MainDriveOpMode(boolean isBlueAlliance) {
        this.isBlueAlliance = isBlueAlliance;
    }

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
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());

        if (gamepad1.a || gamepad1.x) {
            automationCommands();
        } else {
            armCommands();
            telemetry.addData("Lift Angle", robotArm.getCurrentRotationPosition());
            telemetry.addData("liftMotor desired position", robotArm.getRotationDesiredPosition());
            telemetry.addData("liftMotor power", robotArm.getLiftMotorPower());
            telemetry.addData("LiftPIDOutput",robotArm.getPIDOutput());
            telemetry.addData("LiftPowerLimit", robotArm.getLiftPowerLimit());

            telemetry.addData("Extension position", robotArm.getCurrentExtensionPosition());
            telemetry.addData("Extension desired position", robotArm.getCurrentExtensionDesiredPosition());
            telemetry.addData("Extension power", robotArm.getExtensionMotorPower());

            wristCommands();
            telemetry.addData("wrist position", robotArm.getCurrentWristPosition());

            clawCommands();
            telemetry.addData("claw position", robotArm.isClawOpen() ? "OPEN" : "CLOSED");
        }

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
            } else {
                robotArm.continueRotating();
            }

            if (gamepad1.right_bumper) {
                robotArm.extendArm();
            } else if (gamepad1.left_bumper) {
                robotArm.retractArm();
            } else {
                robotArm.continueExtension();
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
        long currentTime = System.currentTimeMillis();
        boolean backButtonPressedNow = gamepad1.back;
        if (backButtonPressedNow && !backButtonPressedLast) {
            if (currentTime - lastToggleTime > DEBOUNCE_DELAY_MS) {
                fastMode = !fastMode;
                lastToggleTime = currentTime;
            }
        }
        // Update the last state to the current state
        backButtonPressedLast = backButtonPressedNow;

        Pose2d weightedStickPose;
        int allianceFlip = isBlueAlliance?1:-1;

        //Make the small movements very slow for most of the stick position
        // Once the stick is moved far, then allow full power
        // Allow a dead zone between switching modes
        double rotationInput = 0;
        if (gamepad1.right_stick_x <= DriveConstants.ROTATION_SLOW_ZONE) {
            rotationInput = (gamepad1.right_stick_x/DriveConstants.ROTATION_SLOW_ZONE) * DriveConstants.ROTATION_SLOW_LIMITER;
        } if (gamepad1.right_stick_x > DriveConstants.ROTATION_FAST_ZONE) {
            rotationInput = (gamepad1.right_stick_x/DriveConstants.ROTATION_FAST_ZONE) * DriveConstants.ROTATION_FAST_LIMITER;
        }
        if (fastMode) {
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-drive.getPoseEstimate().getHeading());
            weightedStickPose = new Pose2d(input.getX(), input.getY(), -rotationInput);
//            weightedStickPose = new Pose2d(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x,
//                    -gamepad1.right_stick_x
//            );
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
        telemetry.addData("weightedO", rotationInput);


        drive.setWeightedDrivePower(weightedStickPose);
        drive.update();
    }
}