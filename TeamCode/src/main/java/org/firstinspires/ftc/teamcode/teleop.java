package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "teleop")
public class teleop extends OpMode {
    public DcMotor br, bl, claw;
    public Arm arm;

    @Override
    public void init() {
        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");
        claw = hardwareMap.get(DcMotor.class, "claw");

        claw.setDirection(DcMotorSimple.Direction.REVERSE);
        arm = new Arm(hardwareMap, telemetry);
        arm.init();
    }

    @Override
    public void start() {
        arm.start();
    }

    @Override
    public void loop() {
        // Controle do chassi
        double drive = gamepad1.right_stick_x;
        double turn = -gamepad1.left_stick_y;
        double leftPower = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);

        bl.setPower(leftPower);
        br.setPower(rightPower);





        if (gamepad1.dpad_down) {
            arm.toLow();
        } else if (gamepad1.dpad_up) {
            arm.toHigh();
        } else if (gamepad1.dpad_left) {
            arm.toMid();
        } else if (gamepad1.dpad_right) {
            arm.toZero();
        }


        if (gamepad1.right_trigger > 0.1) {
            claw.setPower(1.0);
        } else if (gamepad1.left_trigger > 0.1) {
            claw.setPower(-1.0);
        } else {
            claw.setPower(0);
        }
        arm.periodic();


    }
}