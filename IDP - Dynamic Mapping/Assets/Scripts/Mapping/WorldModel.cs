using UnityEngine;

public interface WorldModel {

    // Return if the given world position corresponds to a static object:
    bool IsStatic(Vector2 worldPosition);

    // Cleanup data that may be cached by the world model for debugging purposes:
    void Cleanup();
}
