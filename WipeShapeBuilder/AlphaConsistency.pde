int[] AlphaFilter_Naive(Observation[] observations, float alpha) {
  boolean[] remove = new boolean[observations.length];
  
  for(int i = 0; i < observations.length; i++) {
    if(remove[i])
      continue;
    
    float theta = observations[i].theta;
    float x = observations[i].x, y = observations[i].y;
    
    PVector u = new PVector(cos(theta + alpha + HALF_PI), sin(theta + alpha + HALF_PI));
    PVector v = new PVector(cos(theta - alpha - HALF_PI), sin(theta - alpha - HALF_PI));
    
    for(int j = 0; j < observations.length; j++) {
      if(j == i)
        continue;
      
      float dX = observations[j].x - x;
      float dY = observations[j].y - y;
      
      float dotU = dX * u.x + dY * u.y;
      float dotV = dX * v.x + dY * v.y;
      
      if(dotU <= 0 && dotV <= 0)
        remove[j] = true;
    }
  }
  
  int count = 0;
  for(int i = 0; i < remove.length; i++)
    if(!remove[i])
      count++;
  
  int[] result = new int[count];
  
  int index = 0;
  for(int i = 0; i < remove.length; i++) {
    if(!remove[i]) {
      result[index] = i;
      index++;
    }
  }
  
  return result;
}

int[] AlphaFilter_Improved(Observation[] observations, float alpha) {
  boolean[] remove = new boolean[observations.length];
  
  for(int i = 0; i < observations.length; i++) {
    if(remove[i])
      continue;
      
    float x = observations[i].x, y = observations[i].y;
    
    // 1. Remove observations in the cone in counter-clockwise order, between theta and theta + alpha:
    int index = i;
    float theta = observations[i].theta;
    float stopAngle = theta + alpha;
    PVector u = new PVector(cos(stopAngle + HALF_PI), sin(stopAngle + HALF_PI));
    
    while(theta < stopAngle) {
      index++;
      
      // If we reached the last observation, return to the first one:
      if(index >= observations.length) {
        index = 0;
        stopAngle -= TWO_PI;
      }
      
      theta = observations[index].theta;
      float dX = observations[index].x - x, dY = observations[index].y - y;
      float dotU = dX * u.x + dY * u.y;
      
      if(dotU <= 0)
        remove[index] = true;
      else
        break;
    }
    
    // 2. Remove observations in the cone in clockwise order, between theta and theta - alpha:
    index = i;
    theta = observations[i].theta;
    stopAngle = theta - alpha;
    u = new PVector(cos(stopAngle - HALF_PI), sin(stopAngle - HALF_PI));
    
    while(theta > stopAngle) {
      index--;
      
      // If we reached the first observation, return to the last one:
      if(index <= 0) {
        index = observations.length - 1;
        stopAngle += TWO_PI;
      }
      
      theta = observations[index].theta;
      float dX = observations[index].x - x, dY = observations[index].y - y;
      float dotU = dX * u.x + dY * u.y;
      
      if(dotU <= 0)
        remove[index] = true;
      else
        break;
    }
  }
  
  int count = 0;
  for(int i = 0; i < remove.length; i++)
    if(!remove[i])
      count++;
  
  int[] result = new int[count];
  
  int index = 0;
  for(int i = 0; i < remove.length; i++) {
    if(!remove[i]) {
      result[index] = i;
      index++;
    }
  }
  
  return result;
}
