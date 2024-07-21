// Implementation of methods that can be useful when dealing with PMatrix3D.
// See https://github.com/ruby-processing/processing-core/blob/master/src/main/java/processing/core/PMatrix3D.java
public static class MatrixUtils {
  public static void setTranslation(PMatrix3D matrix, PVector translation){
    matrix.m03 = translation.x;
    matrix.m13 = translation.y;
    matrix.m23 = translation.z;
  }
  
  public static void setTranslation(PMatrix3D matrix, float x, float y, float z){
    matrix.m03 = x;
    matrix.m13 = y;
    matrix.m23 = z;
  }
  
  // Returns the x-coordinate of the result of multiplying the given vector 
  // to the left of the given matrix:
  public static float preMultX(PMatrix3D matrix, PVector v){
    return v.x * matrix.m00 +  v.y * matrix.m10 + v.z * matrix.m20;
  }
  
  // Returns the y-coordinate of the result of multiplying the given vector 
  // to the left of the given matrix:
  public static float preMultY(PMatrix3D matrix, PVector v){
    return v.x * matrix.m01 +  v.y * matrix.m11 + v.z * matrix.m21;
  }
  
  // Returns the z-coordinate of the result of multiplying the given vector 
  // to the left of the given matrix:
  public static float preMultZ(PMatrix3D matrix, PVector v){
    return v.x * matrix.m02 +  v.y * matrix.m12 + v.z * matrix.m22;
  }
  
  // Returns the w-coordinate of the result of multiplying the given vector 
  // to the left of the given matrix:
  public static float preMultW(PMatrix3D matrix, PVector v){
    return v.x * matrix.m03 +  v.y * matrix.m13 + v.z * matrix.m23;
  }
  
  // TODO: here we don't take into account the translation of the matrix ! Which means that (x,y,z) is an axis and not a 3D point.
  // This is not clear !
  
  // Compute the dot product of the given point and the vectors u, v and w of the base:
  public static PVector globalToLocal(PMatrix3D matrix, float x, float y, float z){
    return new PVector(x * matrix.m00 + y * matrix.m10 + z * matrix.m20,
                       x * matrix.m01 + y * matrix.m11 + z * matrix.m21,
                       x * matrix.m02 + y * matrix.m12 + z * matrix.m22);
  }
  
  // Return u*x + v*y + w*z:
  public static PVector localToGlobal(PMatrix3D matrix, float x, float y, float z){
    return new PVector(matrix.m00 * x + matrix.m01 * y + matrix.m02 * z,
                       matrix.m10 * x + matrix.m11 * y + matrix.m12 * z,
                       matrix.m20 * x + matrix.m21 * y + matrix.m22 * z);
  }
}
