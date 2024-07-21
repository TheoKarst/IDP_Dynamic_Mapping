InfiniteGrid grid;
CameraController cameraController;
PointCloud pointCloud;

void setup(){
  size(1400, 800, P3D);
  
  grid = new InfiniteGrid();
  cameraController = new CameraController(grid);
  pointCloud = new PointCloud(1, color(0, 0, 255));
  pointCloud.loadData("GT_indoor06.csv");
}

void draw(){
  //background(53);
  background(200);
  lights();
  
  cameraController.updateCamera();  
  grid.drawGrid(cameraController.getTransform());
  
  pointCloud.Draw();
}
