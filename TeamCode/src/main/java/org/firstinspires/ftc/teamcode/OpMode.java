
package org.firstinspires.ftc.teamcode;
//Blbliotecas nescessárias para o TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//Motor e afins
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//Odometria
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "MainCode", group = "Linear")
public class TeleOpfinal extends LinearOpMode {

    // Locomoção
    private DcMotor Ldf, Ldt, Lef, Let;

    // Coletor
    private DcMotor Coletor;
    private DcMotor Auxilio;
    private boolean coletorLigado  = false;
    private boolean botaoAAnterior = false;

    // Lançador
    private DcMotor motorPotencia;
    private CRServo ServoC;
    private boolean servoCLigado = false;
    private boolean botaoYAnteerior = false;

    // comando da torreta (0.5 = parada)
    private ElapsedTime timer = new ElapsedTime();

    // Servos Modeladores (Se eles controlam posições, devem ser do tipo Servo)
    private Servo servoPotencia;
    private Servo servo2Potencia;
    private double PotenciaAtual = 1;
    private double PotenciaAtual2 = 1;

    // Odometria GoBilda Pinpoint
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        inicializarHardware();
        invmotores();

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odometry();   // configura a odometria UMA vez, antes do start

        telemetry.addData("Status", "Pronto");
        telemetry.addData("ServoC", ServoC == null ? "NAO ENCONTRADO na config" : "ok");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            odo.update();   // atualiza a odometria a cada ciclo

            movimentacao();
            coletor();
            motorsLancador();
            servoLancador();

            Pose2D pos = odo.getPosition();

            telemetry.addData("Lancador", "%.2f", motorPotencia.getPower());
            telemetry.addData("ServoC", ServoC == null ? "NAO ENCONTRADO" : "ok");
            telemetry.addData("Auxilio", "%.2f", Auxilio.getPower());   // MUDOU
            telemetry.addData("Coletor", "%.2f", Coletor.getPower());
            telemetry.update();
        }
    }

    public void invmotores() {
        Lef.setDirection(DcMotor.Direction.FORWARD);
        Let.setDirection(DcMotor.Direction.FORWARD);
        Ldf.setDirection(DcMotor.Direction.REVERSE);
        Ldt.setDirection(DcMotor.Direction.REVERSE);
        Coletor.setDirection(DcMotor.Direction.FORWARD);
        Auxilio.setDirection(DcMotor.Direction.REVERSE);
    }

    // Controle 1 - Locomoção
    public void movimentacao() {
        double  axial   =  this.gamepad1.left_stick_y * 0.75,
                lateral = -this.gamepad1.left_stick_x * 0.75,
                rot     = -this.gamepad1.right_stick_x * 0.75;

        double VLef = axial - lateral + rot,
                VLdf = axial + lateral - rot,
                VLet = axial + lateral + rot,
                VLdt = axial - lateral - rot;

        double max = Math.max(Math.abs(VLef), Math.abs(VLdf));
        max = Math.max(max, Math.abs(VLet));
        max = Math.max(max, Math.abs(VLdt));

        if (max > 1) {
            VLef /= max;
            VLdf /= max;
            VLet /= max;
            VLdt /= max;
        }

        Lef.setPower(VLef);
        Ldf.setPower(VLdf);
        Let.setPower(VLet);
        Ldt.setPower(VLdt);
    }

    // Controle 1 - Coletor (botão A)
    public void coletor() {             //******
        boolean botaoAAtual = this.gamepad1.a;

        if (botaoAAtual && !botaoAAnterior) {
            coletorLigado = !coletorLigado;
        }

        if (coletorLigado) {
            Coletor.setPower(1.0);
            Auxilio.setPower(1.0);
        } else {
            Coletor.setPower(0.0);
            Auxilio.setPower(0.0);
        }

        if(gamepad1.b){
            Coletor.setPower(-0.5);
            Auxilio.setPower(-0.5);
        }
        else{
            Coletor.setPower(0.0);
            Auxilio.setPower(0.0);
        }

        botaoAAnterior = botaoAAtual;
    }

    // Controle 2 - Lançador (analógico direito pra cima) e gatilho (Y)
    public void motorsLancador() {// pra cima = positivo
        boolean botaoYAtual = gamepad2.y;
        if (gamepad2.right_trigger > 0.2)
            motorPotencia.setPower(0.51);
        else {
            motorPotencia.setPower(0.0);
        }
        if(botaoYAtual & !botaoYAnteerior){
            servoCLigado = !servoCLigado;
        }
        if(servoCLigado){
            ServoC.setPower(1.0);
        }
        else{
            ServoC.setPower(0.0);
        }
        botaoYAnteerior = botaoYAtual;
    }

    //Servos Auxiliadores
    public void servoLancador() {
        if (gamepad2.a) {

            servoPotencia.setPosition(0.6);

            servo2Potencia.setPosition(0.6);
        }
        if (gamepad2.x) {
            servoPotencia.setPosition(0.0);

            servo2Potencia.setPosition(0.0);
        }

    }

    public void odometry(){

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

    public void inicializarHardware() {
        Ldf            = hardwareMap.get(DcMotor.class,   "frontRight");
        Ldt            = hardwareMap.get(DcMotor.class,   "backRight");
        Lef            = hardwareMap.get(DcMotor.class,   "frontLeft");
        Let            = hardwareMap.get(DcMotor.class,   "backLeft");
        Coletor        = hardwareMap.get(DcMotor.class,   "motorExtra");
        Auxilio        = hardwareMap.get(DcMotor.class,   "motorAuxiliar");
        motorPotencia  = hardwareMap.get(DcMotor.class,   "motorPotencia");
        servoPotencia  = hardwareMap.get(Servo.class,     "servoPotencia");
        servo2Potencia = hardwareMap.get(Servo.class,     "servo2Potencia");
        ServoC         = hardwareMap.get(CRServo.class,   "ServoColetor");
        odo            = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        motorPotencia.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        servoPotencia.setDirection(Servo.Direction.REVERSE);
        servo2Potencia.setDirection(Servo.Direction.FORWARD);

    }
}