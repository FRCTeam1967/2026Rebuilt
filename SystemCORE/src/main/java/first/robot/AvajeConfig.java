package first.robot; // Use your actual project package name
import io.avaje.jsonb.Json;

// This forces Avaje's compiler processor to auto-generate adapters for Limelight's embedded classes
@Json.Import({
})
public interface AvajeConfig {
    // Leave this completely blank! It is just a configuration anchor for the compiler.
}
