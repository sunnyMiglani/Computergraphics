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
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false


struct Intersection
{
  vec4 position;
  float distance;
  int triangleIndex;
};



/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
bool ClosestIntersection(vec3 start,vec3 dir,const vector<Triangle>& triangles,Intersection& closestIntersection );

int main( int argc, char* argv[] )
{
  vector<Triangle> triangles;
  LoadTestModel(triangles);


  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  while( NoQuitMessageSDL() )
    {
      Update();
      Draw(screen);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen)
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  vec3 colour(1.0,0.0,0.0);
  for(int i=0; i<1000; i++)
    {
      uint32_t x = rand() % screen->width;
      uint32_t y = rand() % screen->height;
      PutPixelSDL(screen, x, y, colour);
    }
}

/*Place updates of parameters here*/
void Update()
{
  static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
  std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/
}

bool ClosestIntersection(vec3 start,vec3 dir,const vector<Triangle>& triangles,Intersection& closestIntersection ){


  bool intersection = false;
  float m = std::numeric_limits<float>::max(); // largest value a float can take

  for (size_t i = 0; i < triangles.size(); i++){
    Triangle triangle = triangles[i];
    vec4 v0 = triangle.v0;
    vec4 v1 = triangle.v1;
    vec4 v2 = triangle.v2;

    vec3 e1 = vec3(v1.x-v0.x, v1.y-v0.y, v1.z-v0.z);//v1 - v0;
    vec3 e2 = vec3(v2.x-v0.x, v2.y-v0.y, v2.z-v0.z);//v2 - v0;
    vec3 b = vec3(start.x-v0.x, start.y-v0.y, start.z-v0.z);//start - v0;

    mat3 A(-dir, e1, e2);
    vec3 x = glm::inverse(A)*b;


    float t = x.x;
    float u = x.y;
    float v = x.z;
    if(u > 0 && v > 0 && (u+v) > 0 && t>0){
      intersection = true;
      closestIntersection.position.x = t; //not sure about position and distance
      closestIntersection.position.y = u; //Found openGl guide that says vec4 is x,y,z,w
      closestIntersection.position.z = v;
      closestIntersection.position.w = 0;
      closestIntersection.distance = start+t*dir;
      closestIntersection.triangleIndex = i;
    }
  }
  return intersection;
}

bool isInTrianglePlane(vec3 v0, vec3 e1, vec3 e2, float u, float v){
  vec3 point = v0 + u*e1 + v*e2;
  if(u < 1 && v < 1 && (u+v) < 1){
    return true;
  } else return false;
}
