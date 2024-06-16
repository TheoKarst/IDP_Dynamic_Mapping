using UnityEngine;

public interface WorldModel {

    // Return if the given world position corresponds to a static object:
    bool IsStatic(Vector2 worldPosition);
    }
