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

  std::ostream& operator<<(std::ostream& os, glm::vec3& p)
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
#define SHOW_LIGHT true

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
void VertexShader(vec4& v, Pixel& p);
mat4 rotation(float yaw);
float getLightDepth(Pixel p);
void populateShadowBuffer();
void VertexShadowShader(vec4& vertex, Pixel& p);

/*
  ---------------------------------------------------
  VARIABLES
*/

const float pi  = 3.141592653589793238463;
vector<Triangle> triangles;
vector<Vertex> vertices;
vec4 cameraPos( 0, 0, -3.001,1);
glm::mat4 R;
float yaw = 0; // Yaw angle controlling camera rotation around y-axis
mat4 cameraDirection =  rotation(0);
vec3 lightPos(0,-0.5,2.5); // (0, -0.5, 2.5)
vec3 lightPower = 16.0f*vec3( 1, 1, 1 );
vec3 indirectLightPowerPerArea = 0.5f*vec3( 1, 1, 1 );
vec3 reflectanceGlobal = vec3(1,1,1);
vec4 torchPos = cameraPos;
mat4 torchDir = cameraDirection;
float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];
float shadowBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];
Pixel shadowPixels[SCREEN_HEIGHT][SCREEN_WIDTH];


/*
 ----------------------------------------------------
*/


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
    // std::cout << "Render time: " << dt << " ms." << std::endl;
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
         //Move Light Source
        if(keystate[SDL_SCANCODE_W]){
            std::cout << "Pos : " << lightPos << std::endl;
             lightPos += vec3(forward*0.05f);
           }
           if(keystate[SDL_SCANCODE_S]){
             lightPos -= vec3(forward*0.05f);
             std::cout << "Pos : " << lightPos << std::endl;
           }
           if(keystate[SDL_SCANCODE_A]){
             lightPos -= vec3(right*0.05f);
             std::cout << "Pos : " << lightPos << std::endl;
           }
           if(keystate[SDL_SCANCODE_D]){
             lightPos += vec3(right*0.05f);
             std::cout << "Pos : " << lightPos << std::endl;
           }
       }
   }
}


void populateShadowBuffer(){

    for(uint32_t i = 0; i < triangles.size(); i++){

      Triangle triangle = triangles[i];
      // Transform each vertex from 3D world position to 2D image position:

      vec3 normal  = vec3(triangle.normal.x,triangle.normal.y,triangle.normal.z);

      vector<Pixel> vertexShadowPixels(3);

      // Here the vertexShader will populate the vertexPixels[] values.
      VertexShadowShader(triangle.v0, vertexShadowPixels[0]);
      VertexShadowShader(triangle.v1, vertexShadowPixels[1]);
      VertexShadowShader(triangle.v2, vertexShadowPixels[2]);


      int maxX = -numeric_limits<int>::max();
      int minX = +numeric_limits<int>::max();
      int maxY = -numeric_limits<int>::max();
      int minY = +numeric_limits<int>::max();


      // Get the borders of the square for min_max values of the set of triangles.
      for(size_t i = 0; i < 3; i++){
        maxX = max(maxX,vertexShadowPixels[i].x);
        minX = min(minX,vertexShadowPixels[i].x);
        maxY = max(maxY,vertexShadowPixels[i].y);
        minY = min(minY,vertexShadowPixels[i].y);

      }
      for(int row = minY; row < maxY; row++){ // looping through the square
        if (row < 0 || row >= SCREEN_HEIGHT) continue;
        for(int col = minX; col < maxX; col++){
            if (col < 0 || col > SCREEN_WIDTH) continue;

            Pixel tPixel;

            tPixel.normal = normal;

            int y = row;
            int x = col;
            tPixel.shadow_row = row;
            tPixel.shadow_col = col;

            Pixel v0 = vertexShadowPixels[0];
            Pixel v1 = vertexShadowPixels[1];
            Pixel v2 = vertexShadowPixels[2];

            vec2 e0 = vec2((v1.x - v0.x),(v1.y - v0.y)),
                 e1 = vec2((v2.x - v0.x),(v2.y - v0.y)),
                 e2 = vec2((x - v0.x),(y - v0.y));
            float d00 = glm::dot(e0, e0);
            float d01 = glm::dot(e0, e1);
            float d11 = glm::dot(e1, e1);
            float d20 = glm::dot(e2, e0);
            float d21 = glm::dot(e2, e1);
            float denom = d00 * d11 - d01 * d01;
            float v = (d11 * d20 - d01 * d21) / denom;
            float w = (d00 * d21 - d01 * d20) / denom;
            float u = 1.0f - v - w;
            // if point p is inside triangles defined by vertices v0, v1, v2
            if (0 <= u && u <= 1 && 0 <= v && v <= 1 && 0 <= w && w <= 1) {
              tPixel.shadow_depth = v0.shadow_depth * u +  v1.shadow_depth * v + v2.shadow_depth * w;
              if(tPixel.shadow_depth > depthBuffer[row][col]){
                  shadowBuffer[row][col] = tPixel.shadow_depth;
                  tPixel.shadow_pos = (v0.shadow_pos * v0.shadow_depth * u
                              + v1.shadow_pos * v1.shadow_depth * v
                              + v2.shadow_pos * v2.shadow_depth * w) / tPixel.shadow_depth;
                }
            }
        }
      }
  }
}


void Draw(screen *screen)
{
    // lightPos = vec3(cameraPos.x, cameraPos.y, cameraPos.z);

  for(int y = 0; y < SCREEN_HEIGHT; y++){ // Set the depth buffer to max values
    for(int x = 0; x < SCREEN_WIDTH; x++){
      depthBuffer[y][x] = 0;//-numeric_limits<int>::max();
      shadowBuffer[y][x] = 0;
    }
  }
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
  populateShadowBuffer();

  for(uint32_t i = 0; i < triangles.size(); i++){

    Triangle triangle = triangles[i];
    // Transform each vertex from 3D world position to 2D image position:

    vec3 normal  = vec3(triangle.normal.x,triangle.normal.y,triangle.normal.z);

    vector<Pixel> vertexPixels(3);

    // Here the vertexShader will populate the vertexPixels[] values.
    VertexShader(triangle.v0, vertexPixels[0]);
    VertexShader(triangle.v1, vertexPixels[1]);
    VertexShader(triangle.v2, vertexPixels[2]);




    int maxX = -numeric_limits<int>::max();
    int minX = +numeric_limits<int>::max();
    int maxY = -numeric_limits<int>::max();
    int minY = +numeric_limits<int>::max();


    // Get the borders of the square for min_max values of the set of triangles.
    for(size_t i = 0; i < 3; i++){
      maxX = max(maxX,vertexPixels[i].x);
      minX = min(minX,vertexPixels[i].x);
      maxY = max(maxY,vertexPixels[i].y);
      minY = min(minY,vertexPixels[i].y);

    }
    for(int row = minY; row < maxY; row++){ // looping through the square
      if (row < 0 || row >= SCREEN_HEIGHT) continue;
      for(int col = minX; col < maxX; col++){
          if (col < 0 || col > SCREEN_WIDTH) continue;
          Pixel tPixel;
          tPixel.normal = normal;
          int y = row;
          int x = col;

          Pixel v0 = vertexPixels[0];
          Pixel v1 = vertexPixels[1];
          Pixel v2 = vertexPixels[2];

          vec2 e0 = vec2((v1.x - v0.x),(v1.y - v0.y)),
               e1 = vec2((v2.x - v0.x),(v2.y - v0.y)),
               e2 = vec2((x - v0.x),(y - v0.y));
          float d00 = glm::dot(e0, e0);
          float d01 = glm::dot(e0, e1);
          float d11 = glm::dot(e1, e1);
          float d20 = glm::dot(e2, e0);
          float d21 = glm::dot(e2, e1);
          float denom = d00 * d11 - d01 * d01;
          float v = (d11 * d20 - d01 * d21) / denom;
          float w = (d00 * d21 - d01 * d20) / denom;
          float u = 1.0f - v - w;
          // if point p is inside triangles defined by vertices v0, v1, v2
          if (0 <= u && u <= 1 && 0 <= v && v <= 1 && 0 <= w && w <= 1) {
            tPixel.zinv = v0.zinv * u +  v1.zinv * v + v2.zinv * w;
            if(tPixel.zinv > depthBuffer[row][col]){
                depthBuffer[row][col] = tPixel.zinv;
                tPixel.worldPos = (v0.worldPos * v0.zinv * u
                            + v1.worldPos * v1.zinv * v
                            + v2.worldPos * v2.zinv * w) / tPixel.zinv;
                tPixel.pos = (v0.pos * v0.zinv * u
                            + v1.pos * v1.zinv * v
                            + v2.pos * v2.zinv * w) / tPixel.zinv;
                // vec3 torchPos_v3 = vec3(torchPos.x, torchPos.y, torchPos.z);
                vec3 r_vec = lightPos - tPixel.worldPos;

                float length_r = glm::length(r_vec);
                float rNorm = glm::dot(glm::normalize(r_vec),normal);

                if (rNorm < 0) rNorm = 0;
                // dVal is the Power of the _incoming_ light.
                vec3 dVal = lightPower * (rNorm / ((float)( 4.0f * pi * length_r * length_r)));

                tPixel.illumination = dVal + indirectLightPowerPerArea;
                // printf("Pixel illumination : %f %f %f\n",tPixel.illumination.x ,tPixel.illumination.y ,tPixel.illumination.z  );
                vec3 pixelColour = triangle.color * tPixel.illumination;
                float lightDepth = getLightDepth(tPixel);
                if( lightDepth * 0.01f >  shadowBuffer[row][col]){
                    pixelColour = vec3(0.0f, 0.0f, 0.0f);
                }
                PutPixelSDL(screen, col, row, pixelColour);
              }
          }
        }
    }
  }


}

float getLightDepth(Pixel p){
    vec4 posV4 = vec4(p.pos.x, p.pos.y, p.pos.z, 0);
    vec4 point = torchDir*vec4(posV4 - torchPos);
    return 1.0f/point.z;
}

void VertexShadowShader(vec4& vertex, Pixel& p){

  vec4 point = torchDir*vec4(vertex - torchPos);
  p.x = (int) (FOCAL_LENGTH * point.x/point.z) + (SCREEN_WIDTH/2);
  p.y = (int) (FOCAL_LENGTH * point.y/point.z) + (SCREEN_HEIGHT/2);
  p.shadow_pos = vec3(point.x, point.y, point.z);
  p.shadow_depth = 1.0f/point.z;
}

// Uses the formula
/*
    D = P max(r.n, 0) / 4 pi r^2
    P is the power of the light source, r is a vector from the surface point to the light
    source and n̂ is the normal of the surface

*/

void VertexShader(vec4& vertex, Pixel& p){

  vec4 point = cameraDirection*vec4(vertex - cameraPos);
  p.x = (int) (FOCAL_LENGTH * point.x/point.z) + (SCREEN_WIDTH/2);
  p.y = (int) (FOCAL_LENGTH * point.y/point.z) + (SCREEN_HEIGHT/2);
  p.pos = vec3(point.x, point.y, point.z);
  p.worldPos = vec3(vertex.x,vertex.y,vertex.z);
  p.zinv = 1.0f/point.z;
}

mat4 rotation(float yaw){
  mat4 R_y(cos(yaw), 0, sin(yaw), 0,
              0    , 1,    0    , 0,
          -sin(yaw), 0, cos(yaw), 0,
              0    , 0,    0    , 1);
  return R_y;
}

//https://www.scratchapixel.com/lessons/3d-basic-rendering/rasterization-practical-implementation/rasterization-stage
/*void BarycentricCoordinates(vector<Pixel>& vertexPixels,int y, int x, bool& pointInTriangle, Pixel& pixel){ // p is a pixel or point in triangle
  float u;
  float v;
  float w;

  ivec2 p(y, x);
  Pixel v0 = vertexPixels[0];
  Pixel v1 = vertexPixels[1];
  Pixel v2 = vertexPixels[2];
  pointInTriangle = false;

  pixel.x = x;
  pixel.y = y;
  Barycentric(v0, v1, v2, p, u, v, w); // u,v,w represent distance from pixel in barycentric
  // if point p is inside triangles defined by vertices v0, v1, v2
  if (u >= 0 && u <= 1 && v >= 0 && v <= 1 && w >= 0 && w <= 1) {
    pointInTriangle = true;
    pixel.zinv = calculateDepth(v0.zinv, v1.zinv, v2.zinv, u,v,w); // calculates the depth via interpolation from u,v,w coordintes

    pixel.pos = calculatePixelPos(v0,v1,v2,u,v,w, pixel.zinv);
    getLightValuePixel(pixel);
  }
}*/

// Compute barycentric coordinates (u, v, w) for
// point p with respect to triangle (a, b, c)
// http://www.r-5.org/files/books/computers/algo-list/realtime-3d/Christer_Ericson-Real-Time_Collision_Detection-EN.pdf
/*void Barycentric(Pixel a, Pixel b, Pixel c, ivec2 p, float &u, float &v, float &w)
{
  vec2 e0 = vec2((b.x - a.x),(b.y - a.y)),
       e1 = vec2((c.x - a.x),(c.y - a.y)),
       e2 = vec2((p.x - a.x),(p.y - a.y));
  float d00 = glm::dot(e0, e0);
  float d01 = glm::dot(e0, e1);
  float d11 = glm::dot(e1, e1);
  float d20 = glm::dot(e2, e0);
  float d21 = glm::dot(e2, e1);
  float denom = d00 * d11 - d01 * d01;
  v = (d11 * d20 - d01 * d21) / denom;
  w = (d00 * d21 - d01 * d20) / denom;
  u = 1.0f - v - w;
}*/
