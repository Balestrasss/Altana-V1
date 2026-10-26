package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// GoBildaPinpointDriver está no mesmo pacote (teamcode), então não precisa de import.
// Se o SDK for 10.2 ou mais novo, apague o arquivo do teamcode e use:
// import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@TeleOp(name = "TesteOdometryAndPods", group = "Linear")
public class TesteOdometryAndPods extends LinearOpMode {

    GoBildaPinpointDriver odo;  //Variável da própria odo

    @Override
    public void runOpMode() {
        // Inicializa UMA vez, antes do start
        initHardware();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();   // primeiro atualiza, depois lê a posição
            moveRobot();    // importante pra n ferrar tudo

            Pose2D pos = odo.getPosition();
            telemetry.addData("X (mm)", "%.1f", pos.getX(DistanceUnit.MM));   //function pra coletar dado no momento
            telemetry.addData("Y (mm)", "%.1f", pos.getY(DistanceUnit.MM));   // tmb
            telemetry.addData("Heading (graus)", "%.1f", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Status odo", odo.getDeviceStatus());
            telemetry.update();
        }
    }

    // Não pode se chamar init(): em LinearOpMode o init() é final
    private void initHardware() {
        // Pinpoint
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Offsets = posição dos pods em relação ao centro de giro do robô (mm)
        // X offset: quão pro lado está o pod que mede X (esquerda positivo)     ver no próprio computador
        // Y offset: quão pra frente está o pod que mede Y (frente positivo)
        odo.setOffsets(30, 12);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Robô PARADO aqui. O reset leva ~0.25 s, então espera antes de setar a posição
        odo.resetPosAndIMU();
        sleep(300);

        // Posição inicial do robô no campo (não confundir com os offsets)
        odo.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void moveRobot() {

        // Options zera o heading (útil se a frente do campo ficar torta)
        if (gamepad1.options) {
            Pose2D p = odo.getPosition();
            odo.setPosition(new Pose2D(DistanceUnit.MM,
                    p.getX(DistanceUnit.MM), p.getY(DistanceUnit.MM),
                    AngleUnit.RADIANS, 0));
        }

        double heading = odo.getPosition().getHeading(AngleUnit.RADIANS);

        // Field-centric: gira o vetor do joystick por -heading
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);
        double globalStrafe  = strafe * cos - forward * sin;
        double globalForward = strafe * sin + forward * cos;

        globalStrafe *= 1.1; // compensa o strafe da mecanum ser mais fraco

        // Normaliza pra nenhuma potência passar de 1
        double denom = Math.max(Math.abs(globalForward) + Math.abs(globalStrafe) + Math.abs(turn), 1);

    }
