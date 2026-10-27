{

    GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        // Inicializa UMA vez, antes do start
        initHardware();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();   // primeiro atualiza, depois lê a posição
            moveRobot();

            Pose2D pos = odo.getPosition();
            telemetry.addData("X (mm)", "%.1f", pos.getX(DistanceUnit.MM));
            telemetry.addData("Y (mm)", "%.1f", pos.getY(DistanceUnit.MM));
            telemetry.addData("Heading (graus)", "%.1f", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Status odo", odo.getDeviceStatus());
            telemetry.update();
        }
    }

    private void initHardware() {

        // Pinpoint
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Offsets = posição dos pods em relação ao centro de giro do robô (mm)
        odo.setOffsets(30, 12, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // CORREÇÃO: Substituído resetPosAndIMU() por recalibrateIMU() e setPosition()
        odo.recalibrateIMU();
        sleep(300);

        // Posição inicial do robô no campo
        odo.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void moveRobot() {
        double forward = -gamepad1.left_stick_y;  // stick Y vem invertido
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

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
        double globalStrafe = strafe * cos - forward * sin;
        double globalForward = strafe * sin + forward * cos;

        globalStrafe *= 1.1; // compensa o strafe da mecanum ser mais fraco

        // Cálculo das potências individuais para tração Mecanum (Field-Centric)
        double frontLeftPower = globalForward + globalStrafe + turn;
        double frontRightPower = globalForward - globalStrafe - turn;
        double backLeftPower = globalForward - globalStrafe + turn;
        double backRightPower = globalForward + globalStrafe - turn;

        // Normaliza pra nenhuma potência passar de 1
        double denom = Math.max(Math.abs(globalForward) + Math.abs(globalStrafe) + Math.abs(turn), 1);

    }
}
