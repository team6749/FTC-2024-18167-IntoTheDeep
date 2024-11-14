package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotClaw {

    private Servo wristServo;
    private Servo leftClawServo;
    private Servo rightClawServo;

    public RobotClaw (HardwareMap hwMap){
        wristServo = hwMap.get(Servo.class,"wrist_servo");
        leftClawServo = hwMap.get(Servo.class,"left_claw_servo");
        rightClawServo = hwMap.get(Servo.class,"right_claw_servo");
    }
    public void openClaw() {
        leftClawServo.setPosition(0);
        rightClawServo.setPosition(1);
    }


    public void closeClaw() {
        leftClawServo.setPosition(1);
        rightClawServo.setPosition(0);
    }

    public boolean isOpen() {
        return true; //TODO actually check
    }

    public void toWristLeft() {
    wristServo.setPosition(0);
    }

    public void toWristCenter() {
    wristServo.setPosition(0.5);
    }

    public void toWristRight() {
    wristServo.setPosition(1);
    }
    private void turnWrist() {

    }
}
