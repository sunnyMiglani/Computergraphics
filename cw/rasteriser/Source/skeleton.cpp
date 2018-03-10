#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;
using glm::ivec2;
using glm::vec2;

namespace std
{
  std::ostream& operator<<(std::ostream& os, glm::vec4& p)
  {
    return os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
  }

  std::ostream& operator<<(std::ostream& os, glm::ivec2& p)
  {
    return os << "(" << p.x << ", " << p.y << ")";
  }
};

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 320
#define FULLSCREEN_MODE false
#define CHECKING_KEY_STATE true
#define FOCAL_LENGTH (SCREEN_HEIGHT*0.98)

struct Intersection
{
  vec4 position;
  float distance;
  int triangleIndex;
};


/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update(vec4& cameraPos, mat4& cameraDirection);
void Draw(screen* screen);
void VertexShader( vec4 vertices, ivec2& projPos );
void DrawLineSDL( SDL_Surface* surface, ivec2 a, ivec2 b, vec3 color );
void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result );
void DrawPolygonEdges( const vector<vec4>& vertices , screen* screen);
mat4 rotation(float yaw);
void BarycentricCoordinates(vector<ivec2>& projectedVertices, int y, int x, bool& pointInTriangle);
float edgeFunction(ivec2& a, ivec2& b, ivec2& c);
void Barycentric(ivec2 a, ivec2 b, ivec2 c, ivec2 p, float &u, float &v, float &w);

/*
  ---------------------------------------------------
  VARIABLES
*/

vector<Triangle> triangles;
vec4 cameraPos( 0, 0, -3.001,1);
glm::mat4 R;
float yaw = 0; // Yaw angle controlling camera rotation around y-axis
mat4 cameraDirection =  rotation(0);

int main(int argc, char* argv[])
{

  screen *screen = InitializeSDL(SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE);
  LoadTestModel(triangles);

  while(NoQuitMessageSDL())
    {
      Update(cameraPos, cameraDirection);
      Draw(screen);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage(screen, "screenshot.bmp");

  KillSDL(screen);
  return 0;
}

void Draw(screen *screen)
{
  vector<ivec2> leftPixels(SCREEN_HEIGHT);
  vector<ivec2> rightPixels(SCREEN_HEIGHT);

  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  bool pointInTriangle;

  for(uint32_t i = 0; i < triangles.size(); i++){
    Triangle triangle = triangles[i];
    // Transform each vertex from 3D world position to 2D image position:
    vector<ivec2> projectedVertices(3);
    VertexShader(triangle.v0, projectedVertices[0]);
    VertexShader(triangle.v1, projectedVertices[1]);
    VertexShader(triangle.v2, projectedVertices[2]);

    int maxX = -numeric_limits<int>::max();
    int minX = +numeric_limits<int>::max();
    int maxY = -numeric_limits<int>::max();
    int minY = +numeric_limits<int>::max();
    for(int i = 0; i < projectedVertices.size(); i++){
      maxX = max(maxX,projectedVertices[i].y);
      minX = min(minX,projectedVertices[i].y); // WTF WHY???
      maxY = max(maxY,projectedVertices[i].x);
      minY = min(minY,projectedVertices[i].x);
    }
    for(int row = minY; row < maxY; row++){
      for(int col = minX; col < maxX; col++){
        BarycentricCoordinates(projectedVertices, row, col, pointInTriangle);
        if(pointInTriangle){
          PutPixelSDL(screen, row, col, triangle.color);
          //break;
        }
      }
    }
  }

  // for(int row = 0; row < screen->height; row++){
  //   for(int col = 0; col < screen->width; col++){
  //     for(uint32_t i=0; i<triangles.size(); ++i)
  //     {
  //       Triangle triangle = triangles[i];
  //       bool pointInTriangle;
  //       vector<vec4> vertices(3);
  //       vector<ivec2> vecProjPos(3);
  //       vertices[0] = triangle.v0;
  //       vertices[1] = triangle.v1;
  //       vertices[2] = triangle.v2;
  //
  //       // DrawPolygonEdges(vertices, screen);
  //       // ComputePolygonRows(vertices, leftPixels, rightPixels);
  //       BarycentricCoordinates(vertices, row, col, pointInTriangle);
  //       if(pointInTriangle){
  //         PutPixelSDL(screen, row, col, triangle.color);
  //         //break;
  //       }
  //
  //     }
  //   }
  // }

}
/*Place updates of parameters here*/
void Update(vec4& cameraPos, mat4& cameraDirection)
{
  static int t = SDL_GetTicks();

  vec4 right(cameraDirection[0][0], cameraDirection[0][1], cameraDirection[0][2], 1);
  vec4 down(cameraDirection[1][0], cameraDirection[1][1], cameraDirection[1][2], 1);
  vec4 forward( cameraDirection[2][0], cameraDirection[2][1], cameraDirection[2][2], 1);

  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
  std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/

  if(CHECKING_KEY_STATE){
     const uint8_t* keystate = SDL_GetKeyboardState(0);

     if(keystate == NULL){
       printf("Keys are NULL \n");
     }
     else {
 //Move Camera
       if(keystate[SDL_SCANCODE_UP]){
         cameraPos += (forward * 0.05f);
       }
       if(keystate[SDL_SCANCODE_DOWN]){
         cameraPos -= (forward * 0.05f);
       }
       if(keystate[SDL_SCANCODE_LEFT]){
         yaw -= 0.04;
         cameraDirection = rotation(yaw);
       }
       if(keystate[SDL_SCANCODE_RIGHT]){
         yaw += 0.04;
         cameraDirection = rotation(yaw);
       }
 // //Move Light Source
 //       if(keystate[SDL_SCANCODE_W]){
 //         lightPositon += (forward*0.05f);
 //       }
 //       if(keystate[SDL_SCANCODE_S]){
 //         lightPositon -= (forward*0.05f);
 //       }
 //       if(keystate[SDL_SCANCODE_A]){
 //         lightPositon -= (right*0.05f);
 //       }
 //       if(keystate[SDL_SCANCODE_D]){
 //         lightPositon += (right*0.05f);
 //       }
     }//end of large else
   }
}

void VertexShader(vec4 vertices, ivec2& projPos) {
  // std::cout << vertices << std::endl;
  vertices = vec4(cameraDirection*vec4(vertices - cameraPos));
  projPos.x = (FOCAL_LENGTH * (vertices.x)/(vertices.z)) + (SCREEN_WIDTH/2);
  projPos.y = (FOCAL_LENGTH * (vertices.y)/(vertices.z)) + (SCREEN_HEIGHT/2);
  // std::cout << projPos << std::endl;
}

void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result )
{
  int N = result.size();
  glm::vec2 step = glm::vec2(b-a) / float(glm::max(N-1,1));
  glm::vec2 current( a );
  for( int i=0; i<N; ++i ){
    result[i] = current;
    current += step;
  }
}

void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color ){
  ivec2 delta = glm::abs(a - b);
  int pixels = glm::max(delta.x, delta.y) + 1;
  vector<ivec2> line(pixels);
  Interpolate(a, b, line);
  for(uint i = 0; i < line.size(); i++){
    PutPixelSDL(screen, line[i].x, line[i].y, color);
  }
}

void DrawPolygonEdges(const vector<vec4>& vertices , screen* screen)
{
  int V = vertices.size();
  // Transform each vertex from 3D world position to 2D image position:
  vector<ivec2> projectedVertices(V);
  for(int i=0; i<V; ++i){
    VertexShader(vertices[i], projectedVertices[i]);
  }
  // Loop over all vertices and draw the edge from it to the next vertex:
  for(int i=0; i<V; ++i){
    int j = (i+1)%V; // The next vertex
    vec3 color(1, 1, 1);
    DrawLineSDL(screen, projectedVertices[i], projectedVertices[j], color);
  }
}

mat4 rotation(float yaw){
  mat4 R_y(cos(yaw), 0, sin(yaw), 0,
              0    , 1,    0    , 0,
          -sin(yaw), 0, cos(yaw), 0,
              0    , 0,    0    , 1);
  return R_y;
}

float edgeFunction(ivec2& a, ivec2& b, ivec2& c)
{
  // ivec2 temp = a;
  // a = b;
  // b = temp;
  return (c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x);
  // return (a.x - b.x) * (c.y - a.y) - (a.y - b.y) * (c.x - a.x);
}

//https://www.scratchapixel.com/lessons/3d-basic-rendering/rasterization-practical-implementation/rasterization-stage
void BarycentricCoordinates(vector<ivec2>& projectedVertices, int y, int x, bool& pointInTriangle){ // p is a pixel or point in triangle
  float u;
  float v;
  float w;

  ivec2 p(y, x);
  ivec2 v0 = projectedVertices[0];
  ivec2 v1 = projectedVertices[1];
  ivec2 v2 = projectedVertices[2];
  pointInTriangle = false;

  Barycentric(v0, v1, v2, p, u, v, w);
  // ivec2 p(y, x);
  // ivec2 v0 = projectedVertices[0];
  // ivec2 v1 = projectedVertices[1];
  // ivec2 v2 = projectedVertices[2];
  // float area = edgeFunction(v0, v1, v2); // area of the triangle multiplied by 2
  // float w0 = edgeFunction(v1, v0, p);
  // float w1 = edgeFunction(v0, v2, p);
  // float w2 = edgeFunction(v2, v1, p);
  // cout << "u: " << u << endl;
  // cout << "v: " << v << endl;
  // cout << "w: " << w << endl;

  // if point p is inside triangles defined by vertices v0, v1, v2
  if (u >= 0 && u <= 1 && v >= 0 && v <= 1 && w >= 0 && w <= 1) {
    pointInTriangle = true;
    // cout << "u: " << u << endl;
    // cout << "v: " << v << endl;
    // cout << "w: " << w << endl;
    // barycentric coordinates are the areas of the sub-triangles divided by the area of the main triangle
    // u /= area;
    // v /= area;
    // w /= area;
  }
}

// Compute barycentric coordinates (u, v, w) for
// point p with respect to triangle (a, b, c)
// http://www.r-5.org/files/books/computers/algo-list/realtime-3d/Christer_Ericson-Real-Time_Collision_Detection-EN.pdf
void Barycentric(ivec2 a, ivec2 b, ivec2 c, ivec2 p, float &u, float &v, float &w)
{
  vec2 e0 = b - a, e1 = c - a, e2 = p - a;
  float d00 = dot(e0, e0);
  float d01 = dot(e0, e1);
  float d11 = dot(e1, e1);
  float d20 = dot(e2, e0);
  float d21 = dot(e2, e1);
  float denom = d00 * d11 - d01 * d01;
  v = (d11 * d20 - d01 * d21) / denom;
  w = (d00 * d21 - d01 * d20) / denom;
  u = 1.0f - v - w;
}



// // // 1. Find max and min y-value of the polygon
// // //and compute the number of rows it occupies.
//
//   int maxVal = -numeric_limits<int>::max();
//   int minVal = +numeric_limits<int>::max();
//   for(int i = 0; i < vertexPixels.size(); i++){
//     maxVal = max(maxVal,vertexPixels.y);
//     minVal = min(minVal,vertexPixels.y);
//   }
//
//
//   int numOfRows = maxVal - minVal + 1;
//
//
// // // 2. Resize leftPixels and rightPixels
// // //so that they have an element for each row.
//
// for (int i =0; i <numOfRows; ++i){
//   leftPixels[i].x = +numeric_limits<int>::max();
//   rightPixels[i].x = -numeric_limits<int>::max();
//   }
// //
// //
// // // // 3. Initialize the x-coordinates in leftPixels
// // // to some really large value and the x-coordinates
// // // in rightPixels to some really small value.
// //
// // for (int i =0; i <numOfRows; ++i){
// //   leftPixels[i].x = +numeric_limits<int>::max();
// //   rightPixels[i].x = -numeric_limits<int>::max();
// // }
//
//
//
//
// // // 4. Loop through all edges of the polygon and use
// // linear interpolation to find the x-coordinate for
// // each row it occupies. Update the corresponding
// // values in rightPixels and leftPixels.
//
//  +
