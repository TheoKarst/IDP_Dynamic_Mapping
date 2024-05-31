public interface WorldModel {

    // Return if the given observation made by the LIDAR corresponds to a dynamic or a static object:
    bool IsStatic(Observation observation);
}
