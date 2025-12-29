package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "StefanAtingeMinori")
public class TeleOpp extends LinearOpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor matura;
    DcMotor pivoter;
    DcMotor turatie;
    DcMotor spinner;

    DcMotor tureta;
    Servo eject;
    TouchSensor touch;

    int ticksPerRevolution = 0; // de luat de la hard
    double gearRatio = 1.0;
    double degreesToRotate = 120;
    boolean reset2 = false;

    Servo unghi;
    TouchSensor limitswitch;

    int position = 0;
    int encoderPosition = 0;
    int targetPositionSpinner = 0;
    double unghiTargetPosition = 0;
    double pivoterTargetPosition = 0;
    double error = 0;
    double lastError = 0;
    double derivative = 0;
    int Kp = 0;
    int Ki = 0;
    int Kd = 0;
    double integralSum = 0;
    double out = 0;

    int slotIndex = 0;

    private void InitWheels() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftRear");
        rightBack = hardwareMap.dcMotor.get("rightRear");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
    }

    private void InitDc() {
        matura = hardwareMap.dcMotor.get("matura");
        pivoter = hardwareMap.dcMotor.get("pivoter");
        turatie = hardwareMap.dcMotor.get("turatie");
        spinner = hardwareMap.dcMotor.get("revolver");

        tureta = hardwareMap.dcMotor.get("tureta");
        tureta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tureta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tureta.setDirection(DcMotor.Direction.FORWARD);
    }

    private void InitServo() {
        unghi = hardwareMap.servo.get("unghi tureta");
        eject = hardwareMap.servo.get("eject");
    }

    private void InitSensors() {
        limitswitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        touch = hardwareMap.get(TouchSensor.class, "limitSwitch");
    }

    @Override
    public void runOpMode() {
        InitWheels();
        InitDc();
        InitServo();
        InitSensors();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gamepadSetup();
            SetWheelsPower();

            if (gamepad1.a) { // de pus butonul
                eject.setDirection(Servo.Direction.FORWARD);
                eject.setPosition(0.55);
                eject.setPosition(0.0);

                int targetTicks = (int) ((degreesToRotate / 360.0)
                        * ticksPerRevolution * gearRatio);

                tureta.setTargetPosition(targetTicks);
                tureta.setPower(0.5);

                while (opModeIsActive() && tureta.isBusy()) {
                    telemetry.addData("Motor Ticks", tureta.getCurrentPosition());
                    telemetry.update();
                }
                tureta.setPower(0);
            }

            if (gamepad1.b) { // de pus butonul
                reset2 = true;
            }

            if (reset2) {
                tureta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                tureta.setPower(1);
                if (touch.isPressed()) {
                    tureta.setPower(0);
                    reset2 = false;
                    tureta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    tureta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }
        }
    }

    private void SetWheelsPower() {
        double left_x = gamepad1.left_stick_x;
        double left_y = -gamepad1.left_stick_y;
        double right_x = gamepad1.right_stick_x;

        double front_left_pw = left_y + left_x + right_x;
        double back_left_pw = left_y - left_x + right_x;
        double front_right_pw = left_y - left_x - right_x;
        double back_right_pw = left_y + left_x - right_x;

        double max = Math.max(Math.abs(front_left_pw),
                Math.max(Math.abs(back_left_pw),
                        Math.max(Math.abs(front_right_pw), Math.abs(back_right_pw))));

        if (max > 1.0) {
            front_left_pw /= max;
            back_left_pw /= max;
            front_right_pw /= max;
            back_right_pw /= max;
        }

        leftFront.setPower(front_left_pw);
        leftBack.setPower(back_left_pw);
        rightFront.setPower(front_right_pw);
        rightBack.setPower(back_right_pw);
    }

    private void intake() {
        matura.setPower(1.0);
    }

    private void revolver() {
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while (timer.seconds() > 3 && opModeIsActive()) {
            while (limitswitch.isPressed()) {
                encoderPosition = spinner.getCurrentPosition();
                error = targetPositionSpinner - encoderPosition;

                derivative = (error - lastError) / timer.seconds();
                integralSum = integralSum + (error * timer.seconds());

                out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
                spinner.setPower(out);

                lastError = error;
                timer.reset();
            }
        }
    }

    private void score() {
        unghi.setPosition(unghiTargetPosition / 270);

        ElapsedTime timer = new ElapsedTime();
        while (pivoterTargetPosition - pivoter.getCurrentPosition() != 0) {
            error = pivoterTargetPosition - pivoter.getCurrentPosition();

            derivative = (error - lastError) / timer.seconds();
            integralSum = integralSum + (error * timer.seconds());

            out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            pivoter.setPower(out);

            lastError = error;
            timer.reset();
        }
    }

    private void turatie() {
        turatie.setPower(0.6);
        if (slotIndex == 3) {
            turatie.setPower(1.0);
        }
    }

    private void gamepadSetup() {
        boolean X = gamepad1.x;
        boolean circle = gamepad1.circle;
        boolean square = gamepad1.square;
        boolean triangle = gamepad1.triangle;
        double rightStickX = gamepad1.right_stick_x;
        double rightStickY = gamepad1.right_stick_y;
        boolean RB = gamepad1.right_bumper;
        boolean LB = gamepad1.left_bumper;
        float LT = gamepad1.left_trigger;
        float RT = gamepad1.right_trigger;
        boolean s = gamepad1.dpad_up;
        boolean d = gamepad1.dpad_right;
        boolean st = gamepad1.dpad_left;
        boolean j = gamepad1.dpad_down;
    }
}
