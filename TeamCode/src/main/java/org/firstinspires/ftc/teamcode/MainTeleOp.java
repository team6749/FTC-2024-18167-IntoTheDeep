package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Main", group="Iterative OpMode")
public class MainTeleOp extends LinearOpMode {

    private DriveTrain driveTrain;

    public static double DESIRED_DISTANCE = 6; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    public static double SPEED_GAIN = 0.023;
    public static double STRAFE_GAIN = 0.015;
    public static double TURN_GAIN = 0.014;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public static double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private WebcamName rearCamera, sideCamera;

    private final Vector2d cameraOffset = new Vector2d(0, 0);
    private static final double kCamEncoder = 360.0 / 8192 ;

    private FieldReferenceLibrary reference;

            FtcDashboard dashboard = FtcDashboard.getInstance();
    private Pose2d foundPose ;

    private Servo servoTurret;
    private FieldReferenceLibrary reflib;
    private DcMotorEx camcoder;



    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        initAprilTag();
        if (USE_WEBCAM) {
            setManualExposure(6, 250);
        }
        visionPortal.setActiveCamera(sideCamera);
        reference = getStemCenterReferences();

        servoTurret = hardwareMap.get(Servo.class, "turret");
        servoTurret.setPosition(0.5);
        camcoder = hardwareMap.get(DcMotorEx.class, "enc");
        foundPose= new Pose2d(0, 0, 0);

        waitForStart();
        camcoder.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
        camcoder.setMode( DcMotor.RunMode.RUN_USING_ENCODER) ;

        while (opModeIsActive()) {
            targetFound = false;
            desiredTag  = null;

            double viewer = kCamEncoder * camcoder.getCurrentPosition() ;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);


                Pose2d currentPose = reference.locate(desiredTag);


                telemetry.addData("x: ", currentPose.getX());
                telemetry.addData("y: ", currentPose.getY());
                telemetry.addData("rot: ", currentPose.getHeading());

                foundPose= currentPose ;
                foundPose = foundPose.plus( new Pose2d(0, 0, - viewer)) ;
                telemetry.addData("robot th:", foundPose.getHeading()) ;
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if ( targetFound && gamepad1.left_bumper) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            }

            double look = reference.cameraScan(foundPose);
            drive  = -gamepad1.left_stick_y  ;
            strafe = -gamepad1.left_stick_x  ;
            turn   = -gamepad1.right_stick_x ;

            telemetry.addData("enc ang", viewer);
            telemetry.addData("camera tag seek", look);

            double dang = viewer - look ;
            if ((dang < -5) || (dang > 5)) {
                if ((-30 < look) && (look < 70))
                {
                    double seek = - 0.3333 * ( 2 * viewer + look ) / 300 ;
                    servoTurret.setPosition(seek + 0.5);
                }
            }

            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            driveTrain.moveRobot(drive, strafe, turn);
        }

    }

    private void initAprilTag() {
        //rearCamera = hardwareMap.get(WebcamName.class, "rearcam");
        sideCamera = hardwareMap.get(WebcamName.class, "sidecam");
        CameraName switchableCam = ClassFactory
                .getInstance()
                .getCameraManager()
                .nameForSwitchableCamera(sideCamera);
        Size resolution = new Size(1280, 720);


        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(933.785, 933.785, 603.208, 387.302)
                .setTagLibrary(getStemCenterLibrary())
                .build();

        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(switchableCam)
                    .setCameraResolution(resolution)
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private AprilTagLibrary getStemCenterLibrary() {
        return new AprilTagLibrary.Builder()
                .addTag(11, "Audience Side Left",
                        5,
                        new VectorF(-70.5f, 47f, 6f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, -0.5f, 0))
                .addTag(12, "Blue alliance wall", 4,
                        new VectorF(0f,70.5f, 6f), DistanceUnit.INCH,
                        new Quaternion(0f, 0.7071f, 0f, 0.7071f, 0))
                .addTag(13, "Opp audience left", 4,
                        new VectorF(70.5f, 47f, 6f), DistanceUnit.INCH,
                        new Quaternion(0.5f, 0.5f, -0.5f, 0.5f, 0))
                .addTag(14, "Opp audience right", 4,
                        new VectorF(70.5f, -47f, 6f), DistanceUnit.INCH,
                        new Quaternion(0.5f, 0.5f, -0.5f, 0.5f, 0))
                .addTag(15, "Red alliance wall", 4,
                        new VectorF(0f, -70.5f, 6f), DistanceUnit.INCH,
                        new Quaternion(0.7071f, 0f, 0.7071f, 0f, 0))
                .addTag(16, "Audience side right", 4,
                        new VectorF(-70.5f, -47f, 6f), DistanceUnit.INCH,
                        new Quaternion(0.5f, 0.5f, -0.5f, 0.5f, 0)).build();

    }

    private FieldReferenceLibrary getStemCenterReferences() {
        return new FieldReferenceLibrary.Builder()
                .addTag(11, -70.5f, 47f, 0)
                .addTag(12, 0f, 70.5f, 90)
                .addTag(13, 70.5f, 47f, 180)
                .addTag(14, 70.5f, -47f, 180)
                .addTag(15, 0f, -70.5f, 270)
                .addTag(16, -70.5f, -47f, 0)
                .build();
    }

    private InertialFrame createInternalFrame(HardwareMap map) {
        return new InertialFrame.Builder(map, 0.001)
                .addInput("fl", -6., 0., 0., 1.)
                .build() ;
    }


}
