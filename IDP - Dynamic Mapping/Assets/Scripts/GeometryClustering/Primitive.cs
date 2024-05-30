using UnityEngine;

public interface Primitive {

    // Compute the velocity of the given point, supposing that it belongs to this primitive:
    public Vector2 VelocityOfPoint(float x, float y);
}
