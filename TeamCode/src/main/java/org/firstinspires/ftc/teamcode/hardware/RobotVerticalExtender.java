package org.firstinspires.ftc.teamcode.hardware;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotVerticalExtender {
    Motor verticalMotor;
    static final double COUNTS_PER_REVOLUTION = 537.6;// # Adjust this for the specific motor encoder counts per revolution
    public RobotVerticalExtender(HardwareMap hwMap) {
        verticalMotor = new Motor(hwMap, "vertical_motor", Motor.GoBILDA.RPM_435);
        verticalMotor.setRunMode(Motor.RunMode.PositionControl);
        verticalMotor.set(1);

        // Set the motor to run using encoders
//        verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }

    public static int MAX_HEIGHT = 50; //TODO - put in a real value
    public static int MIN_HEIGHT = 0;
    public static int BASE_HEIGHT = 5; //TODO - put in a real value

    public static int LOW_BASKET_HEIGHT = 20; //TODO - put in a real value
    public static int HIGH_BASKET_HEIGHT = (int) (7.5 * COUNTS_PER_REVOLUTION); //TODO - put in a real value
    public static int POSITION_TOLERANCE = 5; //TODO - put in a real value



    public void raise() {

    }

    public void lower() {

    }

    public void toPosition(int desiredPosition) {
//        verticalMotor.setTargetPosition(desiredPosition);
        verticalMotor.setRunMode(Motor.RunMode.RawPower);
    verticalMotor.set(0.3);
    }

    public void toHighBasket() {
        toPosition(HIGH_BASKET_HEIGHT);
    }

    public void toLowBasket() {
        toPosition(LOW_BASKET_HEIGHT);
    }

    public void toBasePosition() {
        toPosition(BASE_HEIGHT);
    }

    public boolean isAtBasePosition() {
        return isAtPosition(BASE_HEIGHT);
    }

    public boolean isAtLowBasketPosition() {
        return isAtPosition(LOW_BASKET_HEIGHT);
    }
    public boolean isAtHighBasketPosition() {
        return isAtPosition(HIGH_BASKET_HEIGHT);
    }

    private boolean isAtPosition(int positionToCheck) {
        return Math.abs(verticalMotor.getCurrentPosition() - positionToCheck) <= POSITION_TOLERANCE;
    }
}
