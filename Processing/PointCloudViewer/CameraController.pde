private enum CameraMotion {
  IDLE,                       // The camera doesn't move
  ROTATING,                   // The camera is rotating around the target, while the mouse is dragged
  TRANSLATING                 // The camera is translating in the plane of the screen, while the mouse is dragged
}

public class CameraController {
  
  // Default parameters for the perspective camera (see https://processing.org/reference/perspective_.html):
  private final float FIELD_OF_VIEW_ANGLE = PI / 3;
  private final float ASPECT = (float) width / height;
  private final float Z_NEAR = (height/20.0) / tan(PI/6);
  private final float Z_FAR = 5 * height / tan(PI/6);
  
  private final float ZOOM_SPEED = 50;
  private final float TRANSLATE_SPEED = 1;
  private final float ROTATE_SPEED = 0.01;
  
  
  // Position and orientation of the camera (FORWARD|LEFT|DOWN|POSITION):
  private PMatrix3D transform = new PMatrix3D();
  private PMatrix3D lastTransform = new PMatrix3D();    // The transform of the camera before moving it with the mouse
  
  private CameraMotion cameraMotion = CameraMotion.IDLE;
  
  // Point in world space around which the camera is rotating:
  private PVector target = new PVector(0, 0, 0);
  private PVector verticalAxis = new PVector(0, 0, 1);
  
  private int mouseStartX, mouseStartY;
  
  public CameraController(InfiniteGrid mgrid){
    
    // Set the camera default position and orientation (look at the origin of the scene):
    transform.translate(0, 0, (height/2.0) / tan(PI*30.0/180.0));
    transform.rotateY(PI/2);
    transform.rotateX(-PI/2);
    
    EventsManager.addEventHandler(EventType.MOUSE_WHEEL, event -> {
      if(cameraMotion != CameraMotion.IDLE)
        return;
        
      transform.translate(-ZOOM_SPEED * ((MouseEvent) event).getCount(), 0, 0);
    });
    
    EventsManager.addEventHandler(EventType.MOUSE_PRESS, event -> {
      MouseEvent mouseEvent = (MouseEvent) event;
      
      if(mouseEvent.getButton() == CENTER){
        mouseStartX = mouseX;
        mouseStartY = mouseY;
        
        if(mouseEvent.isShiftDown()){
          cameraMotion = CameraMotion.TRANSLATING;
        }
        else{
          cameraMotion = CameraMotion.ROTATING;
          
          // Define the direction of rotation around the vertical axis, depending 
          // on the position of the mouse and the orientation of the camera:
          
          // Projection of the vertical axis along the forward and down vectors of the camera:
          float pX = MatrixUtils.preMultX(transform, verticalAxis);
          float pZ = MatrixUtils.preMultZ(transform, verticalAxis);
          
          // If the camera is approximately aligned with the vertical axis, then the direction of the rotation depends on the position of the click.
          // More precisely, it depends on if we are clicking on the top or the bottom of the target:
          if(abs(pX) > 0.95){
            // Compute the position and direction of the raycast produced by the mouse
            PVector rayStart = screenToWorld(mouseX, mouseY);
            PVector rayDirection = cameraRaycast(rayStart);
            
            // Compute the intersection between the raycast from the mouse and the grid:
            PVector intersection = grid.rayIntersect(rayStart, rayDirection);
            
            if(intersection != null){
              // Projection of the vector from the target to the intersection, along the down vector of the camera:
              float p = MatrixUtils.preMultZ(transform, intersection.sub(target));
              
              if(pX * p > 0)
                verticalAxis.mult(-1);
            }
            else if(pZ > 0)
              verticalAxis.mult(-1);
          }
          else if(pZ > 0)
            verticalAxis.mult(-1);
        }
        
        // Save the current transform, and apply the mouse transforms from there, until the mouse release:
        lastTransform.set(transform);
      }
      else
        cameraMotion = CameraMotion.IDLE;
    });
    
    EventsManager.addEventHandler(EventType.MOUSE_DRAG, event -> {
      if(cameraMotion == CameraMotion.IDLE)
        return;
        
      float dX = mouseX - mouseStartX;
      float dY = mouseY - mouseStartY;
      
      if(cameraMotion == CameraMotion.TRANSLATING){
        // Translate the camera:
        transform.set(lastTransform);
        transform.translate(0, dX * TRANSLATE_SPEED, -dY * TRANSLATE_SPEED);
      }
      else{
        // Rotate the camera around the target:
        transform.set(lastTransform);
        
        // Vector from the target to the camera, expressed in the referential of the camera:
        PVector translate = MatrixUtils.globalToLocal(transform, transform.m03 - target.x, 
                                                                 transform.m13 - target.y, 
                                                                 transform.m23 - target.z);
        
        // Axis around which the camera is rotating, expressed in the referential of the camera:
        PVector axis = MatrixUtils.globalToLocal(transform, verticalAxis.x, verticalAxis.y, verticalAxis.z);
        
        // To rotate around the target, around the given axis, we first translate it to the target:
        MatrixUtils.setTranslation(transform, target.x, target.y, target.z);
        
        // Then we rotate around the given axis and target: 
        transform.rotate(dX * ROTATE_SPEED, axis.x, axis.y, axis.z);
        transform.rotateY(-dY * ROTATE_SPEED);
        
        // And we finally translate back the camera:
        transform.translate(translate.x, translate.y, translate.z);
      }
    });
    
    EventsManager.addEventHandler(EventType.MOUSE_RELEASE, event -> {
      cameraMotion = CameraMotion.IDLE;
    });
  }
  
  public void updateCamera(){
    camera(transform.m03, transform.m13, transform.m23,    // eye position
          transform.m03 + transform.m00, transform.m13 + transform.m10, transform.m23 + transform.m20,     // center position = eye + forward
          transform.m02, transform.m12, transform.m22);    // down vector
  }
  
  // Convert a position from world space to screen space, assuming that the screen is at a distance of "Z_NEAR" from the camera:
  private PVector screenToWorld(float x, float y){
    
    // Size of the screen, at a distance of "Z_NEAR" from the camera eye, in world space coordinates:
    float Height = 2 * Z_NEAR * tan(FIELD_OF_VIEW_ANGLE / 2);
    float Width = Height * ASPECT;
    
    float dX = Width * (width/2 - x) / width;      // Positive at the left of the center of the screen
    float dY = Height * (y - height/2) / height;   // Positive at the bottom of the center of the screen
    
    // Center of the screen, at a distance of "Z_NEAR" from the camera eye, in world space coordinates:
    PVector center = new PVector(transform.m03 + transform.m00 * Z_NEAR,
                                transform.m13 + transform.m10 * Z_NEAR,
                                transform.m23 + transform.m20 * Z_NEAR);
    
    // Distance between the point (x, y) and the center of the screen, along the vectors (down and left) of the camera in world space:
    PVector vertical = new PVector(transform.m02 * dY, transform.m12 * dY, transform.m22 * dY);
    PVector horizontal = new PVector(transform.m01 * dX, transform.m11 * dX, transform.m21 * dX);
    
    return center.add(vertical).add(horizontal);
  }
  
  // Return a normalized vector from the camera eye to the given point in world space:
  private PVector cameraRaycast(PVector P){
    PVector ray = new PVector(
      P.x - transform.m03, 
      P.y - transform.m13, 
      P.z - transform.m23);
    
    return ray.normalize();
  }
  
  private PMatrix3D getTransform(){
    return transform;
  }
}
