package first.robot; // Use your actual project package name

import first.robot.LimelightHelpers.IMUResults;
import io.avaje.jsonb.Json;

// This forces Avaje's compiler processor to auto-generate adapters for Limelight's embedded classes
@Json.Import({
    LimelightHelpers.LimelightResults.class,
    LimelightHelpers.LimelightTarget_Detector.class,
    LimelightHelpers.LimelightTarget_Fiducial.class,
    LimelightHelpers.LimelightTarget_Barcode.class,
    LimelightHelpers.LimelightTarget_Classifier.class,
    IMUResults.class
})
public interface AvajeConfig {
    // Leave this completely blank! It is just a configuration anchor for the compiler.
}
