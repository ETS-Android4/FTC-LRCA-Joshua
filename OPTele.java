/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="OurTeleOp", group="Linear Opmode")
//@Disabled
public class OPTele extends LinearOpMode {

    // Declare OpMode members.
    ElapsedTime runtime = new ElapsedTime();

    DcMotor leftDriveBack;
    DcMotor leftDriveFront;
    DcMotor rightDriveBack;
    DcMotor rightDriveFront;
    DcMotor collectionMotor;
    DcMotor carouselMotor;
    DcMotor viperMotor;

    double motSpeed;


    static final double INCREMENT   = 0.001;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    static final double START_POS = 0.5;

    Servo   servoBasket;
    double  position = START_POS; // Start at halfway position


    @Override
    public void runOpMode() {


        telemetry.addData("Status", "Initialized");
        //FRW (Front right wheel) port 0.
        //FLW (Front Left wheel) port 1.
        //BRW (Back right wheel) port 2.
        //BLW (Back left wheel) port 3.
        //COLLM (Collection Motor) port 0 expansion hub
        telemetry.update();


        rightDriveFront = hardwareMap.get(DcMotor.class, "FRW");
        leftDriveFront = hardwareMap.get(DcMotor.class, "FLW");
        rightDriveBack = hardwareMap.get(DcMotor.class, "BRW");
        leftDriveBack = hardwareMap.get(DcMotor.class, "BLW");
        collectionMotor = hardwareMap.get(DcMotor.class, "COLLM");
        carouselMotor = hardwareMap.get(DcMotor.class, "CARM");
        viperMotor = hardwareMap.get(DcMotor.class, "VIPM");

        servoBasket = hardwareMap.get(Servo.class, "SERVOBAS");



        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        leftDriveFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotorSimple.Direction.FORWARD);
        collectionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        carouselMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        viperMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            leftDriveFront.setPower(v1 / motSpeed);
            rightDriveFront.setPower(v2 / motSpeed);
            leftDriveBack.setPower(v3 / motSpeed);
            rightDriveBack.setPower(v4 / motSpeed);


            if (gamepad1.dpad_up) {
                viperMotor.setPower(1);
                gamepad1.dpad_down = false;
            } if (gamepad1.dpad_down) {
                viperMotor.setPower(-1);
                gamepad1.dpad_up = false;
            } else {
                viperMotor.setPower(0);
            }



            if (gamepad1.right_bumper) {
                motSpeed = 2.5;
            } else {
                motSpeed = 1;
            }


            if (gamepad1.x) {
                collectionMotor.setPower(1);
            } else {
                collectionMotor.setPower(0);
            }

            if (gamepad1.y) {
                carouselMotor.setPower(0.5);
            } else {
                carouselMotor.setPower(0);
            }

            if (gamepad1.b) {
                // Keep stepping up until we hit the max value.
                position = 0.5;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                }
            }
            if (gamepad1.a) {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                }
            }

            if (gamepad1.left_bumper) {
                carouselMotor.setPower(-0.5);
            }

            servoBasket.setPosition(position);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.update();
        }
    }
}
