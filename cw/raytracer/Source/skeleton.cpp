#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include "limits"

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;


#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 320
#define FULLSCREEN_MODE false
#define CHECKING_KEY_STATE true



struct Intersection
{
  vec4 position;
  float distance;
  int triangleIndex;
};



/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update(vec4& cameraPos);
mat4 rotation(float yaw);
void Draw(screen* screen, vector<Triangle>& triangles, vec4& cameraPos);
bool ClosestIntersection(vec4 start,vec4 dir,const vector<Triangle>& triangles,Intersection& closestIntersection );
float yaw = 0;
const double pi =3.141592653589793238463;

int main( int argc, char* argv[] )
{
  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  vector<Triangle> triangles;
  LoadTestModel(triangles);


  vec4 cameraPos(0, 0, -3, 1.0); // TODO: Make structure for camera and all these things to it
  while( NoQuitMessageSDL() )
    {
      Update(cameraPos);
      Draw(screen, triangles, cameraPos);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen, vector<Triangle>& triangles, vec4& cameraPos)
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
  //vec3 colour(1.0,0.0,0.0);

  // vector<Triangle> triangles;
  // LoadTestModel(triangles);

  // <<<<<<<<<< This is the main draw loops, the rest is just so it compiles >>>>>>>>>>>
  float focalLength = SCREEN_WIDTH;
  // vec4 cameraPos(0, 0, -3, 1.0);
  bool intersection;
  Intersection triangleIntersection;

  for(int y = 0; y < screen->height; y++){ //int because size_t>0
    for(int x = 0; x < screen->width; x++){
      vec4 d(x- SCREEN_WIDTH/2, y - SCREEN_HEIGHT/2, focalLength, 1);
      intersection = ClosestIntersection(cameraPos, d, triangles, triangleIntersection);
      if(intersection){
        PutPixelSDL(screen, x, y, triangles[triangleIntersection.triangleIndex].color
          /(triangleIntersection.distance*100)); //gives depth by reducing color of pixels further away
      }
    }
  }

  /*for(int i=0; i<1000; i++)
  {
    uint32_t x = rand() % SCREEN_WIDTH;
    uint32_t y = rand() % SCREEN_HEIGHT;
    PutPixelSDL(screen, x, y, colour);
  }*/
}

/*Place updates of parameters here*/
void Update(vec4& cameraPos)
{
  static int t = SDL_GetTicks();

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
      if(keystate[SDL_SCANCODE_UP]){
        cameraPos.z += 0.05;
      }
      if(keystate[SDL_SCANCODE_DOWN]){
        cameraPos.z -=0.05;
      }
      if(keystate[SDL_SCANCODE_LEFT]){
        yaw = 0.04;
        cameraPos = rotation(yaw)*cameraPos;

      }
      if(keystate[SDL_SCANCODE_RIGHT]){
        yaw = -0.04;
        cameraPos = rotation(yaw)*cameraPos;
      }
    }//end of large else
  }
}

bool ClosestIntersection(vec4 start,vec4 dir,const vector<Triangle>& triangles,Intersection& closestIntersection ){
  bool intersection = false;
  closestIntersection.distance = std::numeric_limits<float>::max(); // take large initial value

  for (size_t i = 0; i < triangles.size(); i++){
    Triangle triangle = triangles[i];
    vec4 v0 = triangle.v0;
    vec4 v1 = triangle.v1;
    vec4 v2 = triangle.v2;

    vec3 e1 = vec3(v1.x-v0.x, v1.y-v0.y, v1.z-v0.z);//v1 - v0;
    vec3 e2 = vec3(v2.x-v0.x, v2.y-v0.y, v2.z-v0.z);//v2 - v0;
    vec3 b = vec3(start.x-v0.x, start.y-v0.y, start.z-v0.z);//start - v0;

    mat3 A(-vec3(dir), e1, e2);
    vec3 x = glm::inverse(A)*b;


    float t = x.x;
    float u = x.y;
    float v = x.z;
    if(u >= 0 && v >= 0 && (u+v) <= 1 && t>=0 && t<closestIntersection.distance){ //check if distance closer than current closestIntersection
      intersection = true;
      closestIntersection.position.x = t;
      closestIntersection.position.y = u;
      closestIntersection.position.z = v;
      closestIntersection.position.w = 0;
      closestIntersection.distance = t;
      closestIntersection.triangleIndex = i;
    }
  }
  return intersection;
}

mat4 rotation(float yaw){
  mat4 R_y(cos(yaw), 0, sin(yaw), 0,
              0    , 1,    0    , 0,
          -sin(yaw), 0, cos(yaw), 0,
              0    , 0,    0    , 1);
  return R_y;
}
