#include <iostream>
#include <chrono>
#include <thread>
#include <Eigen/Eigen>

#include <pangolin/pangolin.h>
#include <pangolin/glglut.h>
#include <pangolin/gldraw.h>
#include <SceneGraph/SceneGraph.h>



using namespace std;

bool should_rotate_ = false;
bool draw_bounding_box_ = false;
bool get_image_bbox_ = true;
bool save_images_ = false;
int img_counter = 0;

ofstream sizes_file;
SceneGraph::GLMesh glMesh;
Eigen::MatrixXd bbox_corners(3, 8);
Eigen::MatrixXd image_corners(3, 4);

SceneGraph::GLText obj_w("", 0, 0, 0);
SceneGraph::GLText obj_l("", 0, 0, 0);


typedef struct {
    int width;
    int height;
    GLubyte *data;
    size_t size;
} ppm_image;


void Usage() {
    cout << "Usage: ModelViewer filename scale" << endl;
}

size_t ppm_save(ppm_image *img, FILE *outfile) {
    size_t n = 0;
    n += fprintf(outfile, "P6\n# THIS IS A COMMENT\n%d %d\n%d\n",
                 img->width, img->height, 0xFF);
    n += fwrite(img->data, 1, img->width * img->height * 3, outfile);
    return n;
}

void SaveImageAndSize(const char* filename){
  // First, finish drawing everything
  glFlush();
  glFinish();

  int width =  glutGet(GLUT_WINDOW_WIDTH);
  int height =  glutGet(GLUT_WINDOW_HEIGHT);
  GLubyte* pixels = new GLubyte[ 3 * width * height];

  // Read the pixel values from the buffer
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);

  ppm_image* img = new ppm_image();
  img->width = width;
  img->height = height;
  img->data = pixels;

  // Save to file (PPM format)
  FILE* output = fopen(filename, "w");
  ppm_save(img, output);

}

void Draw2DBoundingBox(Eigen::Vector3d c1, Eigen::Vector3d c2){

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();

  glOrtho(0.0, glutGet(GLUT_WINDOW_WIDTH), 0.0, glutGet(GLUT_WINDOW_HEIGHT), -1.0, 1.0);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glColor3f(1.0, 1.0, 0.0);

  pangolin::glDrawRectPerimeter(c1(0), c1(1), c2(0), c2(1));


  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);

}

void Draw3DBoundingBox(){
  // Draw the bouding box around the object

  glColor3f(1.0, 0.0, 0.0);

  // bottom left
  pangolin::glDrawLine3D(bbox_corners.block<3, 1>(0,0),
                       bbox_corners.block<3, 1>(0,1));

  // draw length of box side

  //bottom right
  pangolin::glDrawLine3D(bbox_corners.block<3, 1>(0,2),
                       bbox_corners.block<3, 1>(0,3));

  // top left
  pangolin::glDrawLine3D(bbox_corners.block<3, 1>(0,4),
                       bbox_corners.block<3, 1>(0,5));
  // top right
  pangolin::glDrawLine3D(bbox_corners.block<3, 1>(0,6),
                       bbox_corners.block<3, 1>(0,7));


  // front left
  pangolin::glDrawLine3D(bbox_corners.block<3, 1>(0,0),
                       bbox_corners.block<3, 1>(0,4));

  // front right
  pangolin::glDrawLine3D(bbox_corners.block<3, 1>(0,2),
                       bbox_corners.block<3, 1>(0,6));

  // rear left
  pangolin::glDrawLine3D( bbox_corners.block<3, 1>(0,1),
                       bbox_corners.block<3, 1>(0,5));

  // rear right
  pangolin::glDrawLine3D(bbox_corners.block<3, 1>(0,3),
                       bbox_corners.block<3, 1>(0,7));


  // front bottom
  pangolin::glDrawLine3D(bbox_corners.block<3, 1>(0,0),
                       bbox_corners.block<3, 1>(0,2));

  // front top
  pangolin::glDrawLine3D(bbox_corners.block<3, 1>(0,4),
                       bbox_corners.block<3, 1>(0,6));

  // rear bottom
  pangolin::glDrawLine3D(bbox_corners.block<3, 1>(0,1),
                       bbox_corners.block<3, 1>(0,3));

  // rear top
  pangolin::glDrawLine3D(bbox_corners.block<3, 1>(0,5),
                       bbox_corners.block<3, 1>(0,7));
}

int main( int argc, char* argv[] )
{
    if(argc != 3) {
        Usage();
        exit(-1);
    }

    const std::string model_filename(argv[1]);
    const int scale(atoi(argv[2]));

    // Create OpenGL window in single line thanks to GLUT
    pangolin::CreateWindowAndBind("Main",640,480);
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
    glClearColor( 0,0,0,0);
    glewInit();

    // Scenegraph to hold GLObjects and relative transformations
    SceneGraph::GLSceneGraph glGraph;

    SceneGraph::GLLight light(10,10,-100);
    glGraph.AddChild(&light);

    glGraph.AddChild(&obj_w);
    glGraph.AddChild(&obj_l);

    SceneGraph::AxisAlignedBoundingBox bbox;

    pangolin::RegisterKeyPressCallback('r', [&]() {
        should_rotate_ = !should_rotate_;
      });

    pangolin::RegisterKeyPressCallback('b', [&]() {
        draw_bounding_box_ = !draw_bounding_box_;
      });

    pangolin::RegisterKeyPressCallback('s', [&]() {
        if(!save_images_){
          std::cerr << "saving images..." << std::endl;
        }else{
          std::cerr << "stopping saving images..." << std::endl;
        }
        save_images_ = !save_images_;
      });

#ifdef HAVE_ASSIMP
    // Define a mesh object and try to load model
    try {
        glMesh.Init(model_filename);
        glGraph.AddChild(&glMesh);
        // this is model dependent...
        glMesh.SetScale(scale);
        bbox = glMesh.ObjectAndChildrenBounds();
        std::cerr << "mesh dimentions: " << glMesh.GetDimensions().transpose() <<
                     std::endl;
        std::cerr << "mesh scale: " << glMesh.GetScale().transpose() << std::endl;
    }catch(exception e) {
        cerr << "Cannot load mesh." << endl;
        cerr << e.what() << std::endl;
        exit(-1);
    }
#endif // HAVE_ASSIMP

    sizes_file.open("sizes.txt", ios_base::trunc);
    sizes_file << "image, width, height"<< std::endl;

    const Eigen::Vector3d center = bbox.Center();
    double bbbox_size = bbox.Size().norm();

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState stacks3d(
        pangolin::ProjectionMatrix(640,480,420,420,320,240, 0.01, 1000),
        pangolin::ModelViewLookAt(center(0), center(1) + bbbox_size, center(2) + bbbox_size/4, center(0), center(1), center(2), pangolin::AxisZ)
    );

    // We define a new view which will reside within the container.
    pangolin::View view3d;

    // We set the views location on screen and add a handler which will
    // let user input update the model_view matrix (stacks3d) and feed through
    // to our scenegraph
    view3d.SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
          .SetHandler(new SceneGraph::HandlerSceneGraph(glGraph,stacks3d))
          .SetDrawFunction(SceneGraph::ActivateDrawFunctor(glGraph, stacks3d));

    // Add our views as children to the base container.
    pangolin::DisplayBase().AddDisplay(view3d);

    Eigen::Vector6d pose = glMesh.GetPose();
    pose[5] += 0.52;
    glMesh.SetPose(pose);

    Eigen::Vector3d size = bbox.Size();

    // 8 corners of the bounding box
    //front bottom left
    bbox_corners.block<3, 1>(0,0) << size[0]/2, -size[1]/2, -size[2]/2;
    //rear bottom left
    bbox_corners.block<3, 1>(0,1) << size[0]/2, size[1]/2, -size[2]/2;

    //front bottom right
    bbox_corners.block<3, 1>(0,2) << -size[0]/2, -size[1]/2, -size[2]/2;
    //rear bottom right
    bbox_corners.block<3, 1>(0,3) << -size[0]/2, size[1]/2, -size[2]/2;

    //front top left
    bbox_corners.block<3, 1>(0,4) << size[0]/2, -size[1]/2, size[2]/2;
    //rear top left
    bbox_corners.block<3, 1>(0,5) << size[0]/2, size[1]/2, size[2]/2;

    //front top right
    bbox_corners.block<3, 1>(0,6) << -size[0]/2, -size[1]/2, size[2]/2;
    //rear top right
    bbox_corners.block<3, 1>(0,7) << -size[0]/2, size[1]/2, size[2]/2;

    bbox_corners = glMesh.GetPose4x4_po().block<3, 3>(0,0) * bbox_corners;




    // Default hooks for exiting (Esc) and fullscreen (tab).
    while( !pangolin::ShouldQuit() )
    {
        // Clear whole screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if(draw_bounding_box_){
          Draw3DBoundingBox();
        }

        if (should_rotate_){
          // Pause for 1/100th of a second.
          std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 100));
          Eigen::Vector6d pose = glMesh.GetPose();
          // rotate approx 1 degree (yaw)
          pose[5] += 0.017;
          Eigen::Matrix4d Ta = glMesh.GetPose4x4_po();
          glMesh.SetPose(pose);
          Eigen::Matrix4d Tab = Ta.inverse()*glMesh.GetPose4x4_po();

          // Rotate the bounding box so it is aligned with the mesh
          bbox_corners = Tab.block<3, 3>(0,0) * bbox_corners;
        }

        // get the pixel coordinates for the object bounds
        double u;
        double v;
        double zdepth;

        Eigen::Vector3d max = Eigen::Vector3d::Zero();
        Eigen::Vector3d min;
        min << 10000, 10000, 10000;

        // Project all 8 corners of the 3d box onto the image plane
        for(int i = 0; i < 8; i++){
          Eigen::Vector3d point = bbox_corners.block<3, 1>(0, i);
          view3d.GetImageCoordinates(stacks3d, u, v, zdepth, point(0), point(1), point(2));

          if(u > max(0)){
            max(0) = u;
            max(2) = zdepth;
          }
          if(u < min(0)){
            min(0) = u;
            min(2) = zdepth;
          }
          if(v > max(1)){
            max(1) = v;
            max(2) = zdepth;
          }
          if(v < min(1)){
            min(1) = v;
            min(2) = zdepth;
          }

        }

        double x1, y1, z1;
        double x2, y2, z2;
        view3d.GetObjectCoordinates(stacks3d, max(0), max(1), max(2), x1, y1, z1);
        view3d.GetObjectCoordinates(stacks3d, min(0), max(1), max(2), x2, y2, z2);

        Eigen::Vector3d dist;
        dist << x1-x2, y1-y2, z1-z2;
        double width = dist.norm();
        //draw width
        obj_w.SetPosition( (x1+x2)/2, (y1+y2)/2, (z1+z2)/2 );
        std::string w = std::to_string(width/100);
        w.append("m");

        view3d.GetObjectCoordinates(stacks3d, max(0), max(1), max(2), x1, y1, z1);
        view3d.GetObjectCoordinates(stacks3d, max(0), min(1), max(2), x2, y2, z2);
        dist << x1-x2, y1-y2, z1-z2;
        double height = dist.norm();
        //draw height
        obj_l.SetPosition( (x1+x2)/2, (y1+y2)/2, (z1+z2)/2 );
        std::string h = std::to_string(height/100);
        h.append("m");



        if(draw_bounding_box_){
          Draw2DBoundingBox(min, max);
          obj_w.SetText(w);
          obj_l.SetText(h);
        }else{
          obj_w.SetText("");
          obj_l.SetText("");
        }


        // Swap frames and Process Events
        pangolin::FinishFrame();

        if(save_images_){
          std::string filename = std::to_string(img_counter++);
          filename.append(".ppm");
          SaveImageAndSize(filename.c_str());
          sizes_file << filename << ", " << width << ", " << height <<
                        std::endl;
        }

        // Pause for 1/60th of a second.
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 60));
    }

    return 0;
}
