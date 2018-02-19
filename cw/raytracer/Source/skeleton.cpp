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


#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 480
#define FULLSCREEN_MODE false
#define CHECKING_KEY_STATE true
#define SHADOW_RENDER false
#define NUM_RAYS 10


struct Intersection
{
  vec4 position;
  float distance;
  int triangleIndex;
};



/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update(vec4& cameraPos,mat4& cameraDirection);
mat4 rotation(float yaw);
void Draw(screen* screen, vector<Triangle>& triangles, vec4& cameraPos, mat4& cameraDirection);
bool ClosestIntersection(vec4 start,vec4 dir,const vector<Triangle>& triangles,Intersection& closestIntersection );
vec3 DirectLight(const Intersection& i,vector<Triangle>& triangles,bool& shadowPixel );
vec3 Dot_Prod_v3(vec3& a, vec3& b);
vec4 Dot_Prod_v4(vec4& a, vec4& b);
vec3 getAntiAliasingAvg(int x, int y, vector<Triangle>& triangles, vec4& cameraPos,
                        mat4& cameraDirection, vector<Intersection>& intersectionArray, bool& check_intersection);

/* ----------------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                            */
float yaw = 0;
vec4 lightPositon(0, -0.5, -0.7, 1);
vec4 lightColor = 14.f * vec4(1, 1, 1,1);
const float pi  = 3.141592653589793238463;
mat4 cameraDirection =  rotation(0);
vec4 cameraPos(0, 0, -3, 1); // TODO: Make structure for camera and all these things to it
vec3 indirectLight = 0.5f * vec3(1, 1, 1);
float focalLength = SCREEN_WIDTH;



int main( int argc, char* argv[] )
{
  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  vector<Triangle> triangles;
  LoadTestModel(triangles);

  while( NoQuitMessageSDL() )
    {
      Update(cameraPos,cameraDirection);
      Draw(screen, triangles, cameraPos, cameraDirection);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}


vec3 getAntiAliasingAvg(int x, int y, vector<Triangle>& triangles, vec4& cameraPos,
                        mat4& cameraDirection, vector<Intersection>& intersectionArray, bool &check_intersection){

  check_intersection = true;
  float offset = 0.5-1/(sqrt(NUM_RAYS)*2);
  float interval = 1/sqrt(NUM_RAYS);
  vec3 totalColor = vec3(0.0,0.0,0.0);
  vec3 avgColor;
  int index = 0;
  bool shadowPixel;
  bool intersection;
  Intersection triangleIntersection;
  int numForAvg = 0;

  for(float j = -offset; j <= offset; j += interval) {
    for(float i = -offset; i <= offset; i += interval) {
      vec4 d(x- SCREEN_WIDTH/2 + i, y - SCREEN_HEIGHT/2 + j, focalLength, 1);
      intersection = ClosestIntersection(cameraPos, cameraDirection*d, triangles, triangleIntersection);
      check_intersection = check_intersection && intersection;
      if(!intersection) { continue;}
      // if(!check_intersection){
      //   std::cout << "Found a non intersection!" << '\n';
      // }
      intersectionArray[index] = triangleIntersection;
      index += 1;

      vec3 shadedPixel = DirectLight(triangleIntersection,triangles,shadowPixel);
      if(shadowPixel){ shadedPixel = vec3(0,0,0); }
      numForAvg+=1;
      totalColor += triangles[triangleIntersection.triangleIndex].color*(shadedPixel+indirectLight);
    }
  }
  avgColor = vec3(totalColor.x / (numForAvg), totalColor.y / (numForAvg), totalColor.z / (numForAvg));
  return avgColor;

}





/*Place your drawing here*/
void Draw(screen* screen, vector<Triangle>& triangles, vec4& cameraPos, mat4& cameraDirection)
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  vector<Intersection> intersectionArray(NUM_RAYS);
  bool all_intersections;

  for(int y = 0; y < screen->height; y++){ //int because size_t>0
    for(int x = 0; x < screen->width; x++){

      vec3 avgColour = getAntiAliasingAvg(x, y, triangles, cameraPos, cameraDirection, intersectionArray, all_intersections);
      // if(all_intersections){
        PutPixelSDL(screen, x, y, avgColour);
      // }
    }
  }
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
        yaw += 0.04;
        cameraDirection = rotation(yaw);
      }
      if(keystate[SDL_SCANCODE_RIGHT]){
        yaw -= 0.04;
        cameraDirection = rotation(yaw);
      }
//Move Light Source
      if(keystate[SDL_SCANCODE_W]){
        lightPositon += (forward*0.05f);
      }
      if(keystate[SDL_SCANCODE_S]){
        lightPositon -= (forward*0.05f);
      }
      if(keystate[SDL_SCANCODE_A]){
        lightPositon -= (right*0.05f);
      }
      if(keystate[SDL_SCANCODE_D]){
        lightPositon += (right*0.05f);
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
      closestIntersection.position = v0 + u*vec4(e1.x,e1.y,e1.z,0) + v*vec4(e2.x,e2.y,e2.z,0);// = s + dir * t
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

vec3 Dot_Prod_v3(vec3& a, vec3& b){
  return vec3(a.x *b.x, a.y*b.y, a.z*b.z);
}

vec4 Dot_Prod_v4(vec4& a, vec4& b){
  return vec4(a.x *b.x, a.y*b.y, a.z*b.z, a.w * b.w);
}

vec3 DirectLight(const Intersection& i, vector<Triangle>& triangles, bool& shadowPixel){
  vec4 position = i.position;
  int triangleIndex = i.triangleIndex;

  vec4 r = normalize(lightPositon - position); // r -> Direction to light.
  float length_r = glm::length(lightPositon - position);

  vec4 normal = triangles[triangleIndex].normal;
  float rNorm = dot(r,normal);

  bool intersection;
  Intersection thisIntersection;
  intersection = ClosestIntersection(position+0.001f*r, r, triangles, thisIntersection);
  if(intersection){
    float distToClosestIntesect = thisIntersection.distance;
    if(length_r >= distToClosestIntesect){
      shadowPixel = true;
    }
    else shadowPixel = false;
  } else shadowPixel = false;


  rNorm = max(rNorm,0.f);
  vec4 D = (lightColor*rNorm)/(float)(4*pi*length_r*length_r);

  // printf("D: (%f),(%f),(%f) \n",D.x,D.y,D.z);
  return vec3(D.x,D.y,D.z);
}
