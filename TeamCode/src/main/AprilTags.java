package testesss;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@TeleOp
public class TesteAprillTags extends LinearOpMode {


    private AprilTagLibrary myAprilTagLibrary;  // Defini a variável

    private final int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

    @Override
    public void runOpMode() {
        //Tipo 1 (sem builder)
        // Processador de AprilTag
        AprilTagProcessor myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // Vision Portal (para usar TFOD tmb basta incluir ", -nome do TFOD-" após o do AprilTag
        VisionPortal myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), myAprilTagProcessor);

        //Tipo 2 (com builder)
        // Processador de AprilTag
        AprilTagProcessor.Builder myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        AprilTagProcessor myAprilTagProcessor1;

        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);
        myAprilTagProcessorBuilder.setDrawTagID(true);       // Default: true, for all detections.
        myAprilTagProcessorBuilder.setDrawTagOutline(true);  // Default: true, when tag size was provided (thus eligible for pose estimation).

        myAprilTagProcessor1 = myAprilTagProcessorBuilder.build();

        // Vision Portal
        VisionPortal.Builder myVisionPortalBuilder = new VisionPortal.Builder();
        VisionPortal myVisionPortal1;

        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        myVisionPortalBuilder.addProcessor(myAprilTagProcessor1);

        myVisionPortalBuilder.setCameraResolution(new Size(640,480));
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        myVisionPortalBuilder.setLiveViewContainerId(cameraMonitorViewId);
        myVisionPortalBuilder.setAutoStopLiveView(true);

        myVisionPortal1 = myVisionPortalBuilder.build();

        // Tipo 3 (com builder em cadeia, para simplificar)
        //Processador de AprilTag
        AprilTagProcessor myAprilTagProcessor2 = new AprilTagProcessor.Builder()
                .setTagLibrary(myAprilTagLibrary)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .build();

        //Vision Portal
        VisionPortal myVisionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .addProcessor(myAprilTagProcessor2)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setLiveViewContainerId(cameraMonitorViewId)
                .setAutoStopLiveView(true)
                .build();

        // Para desabilitar o processador de AprilTag(liberar espaço na CPU)
        myVisionPortal2.setProcessorEnabled(myAprilTagProcessor2 , false);

        // Para detectar AprilTag (varias, para so uma é bem trivial)
        AprilTagProcessor myAprilTagProcessor3;
        List<AprilTagDetection> myAprilTagDetections = myAprilTagProcessor.getDetections();;
        int AprilTagId;

        for(AprilTagDetection deteccao : myAprilTagDetections){
            if(deteccao.metadata != null) AprilTagId = deteccao.id;
        }

        // Para detectar a posição da AprilTag relativa à camera ( eixo x pra direita, y pra frente e z pra cima
        double TagX = myAprilTagDetections.get(0).ftcPose.x;
        double TagY = myAprilTagDetections.get(0).ftcPose.y;
        double TagZ = myAprilTagDetections.get(0).ftcPose.z;
        double Pitch = myAprilTagDetections.get(0).ftcPose.pitch; // rotação em torno do eixo X
        double Roll = myAprilTagDetections.get(0).ftcPose.roll; // rotação em torno do eixo Y
        double Yaw = myAprilTagDetections.get(0).ftcPose.yaw; // rotação em torno do eixo Z

        // outras posições (como se fossem coordenadas polares)
        double Range= myAprilTagDetections.get(0).ftcPose.range; // distancia
        double Bearing = myAprilTagDetections.get(0).ftcPose.bearing; // angulo pros lados
        double Elevation = myAprilTagDetections.get(0).ftcPose.elevation; // angulo pra cima ou pra baixo


    }
}
