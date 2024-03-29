#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include "limits"
#include <omp.h>
//#include <glm/gtx/rotate_vector.hpp>
#include <glm/ext.hpp>


using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;


#define SCREEN_WIDTH 720
#define SCREEN_HEIGHT 720
#define FULLSCREEN_MODE false
#define CHECKING_KEY_STATE true
#define SHADOW_RENDER false
#define NUM_RAYS 25
#define NUM_LIGHT_RAYS 128
#define NUM_PHOTONS 500000
#define NUM_BOUNCES 5



struct Intersection
{
  vec4 position;
  float distance; // potato
  int triangleIndex;
};

struct Photon
{
  vec3 color;
  vec3 position;
  vec3 dir;
  float norm_length;
  Intersection p_intersection;
  vec3 triangle_normal;
};



/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update(vec4& cameraPos,mat4& cameraDirection);
mat4 rotation(float yaw);
void Draw(screen* screen, vector<Triangle>& triangles, vec4& cameraPos, mat4& cameraDirection);
bool ClosestIntersection(vec4 start,vec4 dir,const vector<Triangle>& triangles,Intersection& closestIntersection );
vec3 DirectLight(const Intersection& i,vector<Triangle>& triangles,bool& shadowPixel, const vec4& lightPos);
vec3 Dot_Prod_v3(vec3& a, vec3& b);
vec4 Dot_Prod_v4(vec4& a, vec4& b);
vec3 getAntiAliasingAvg(int x, int y, vector<Triangle>& triangles, vec4& cameraPos,
                        mat4& cameraDirection, vector<Intersection>& intersectionArray, bool& check_intersection);
vec3 AreaLight(const Intersection& i, vector<Triangle>& triangles, bool& shadowPixel);
void GenAreaLight(vector<vec4>& lightPositionArr, const vec4& lightPosition);
void BouncePhotons(vector<Triangle> triangles);
void CastPhotons(vector<Triangle> triangles);
void InitCastPhotons(vector<Triangle> triangles);
vec3 GatherPhotons(vec4 rayPos, Triangle& triangles);
vec3 toVec3(vec4 vector);
vec4 toVec4(vec3 vector);
/* ----------------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                            */
float yaw = 0;
vec4 lightPosition(0, -0.5, -0.7, 1);
vector<vec4> lightPositionArr(NUM_LIGHT_RAYS);
vec4 lightColor = 14.f * vec4(1, 1, 1, 1);
const float pi  = 3.141592653589793238463f;
mat4 cameraDirection =  rotation(0);
vec4 cameraPos(0, 0, -3, 1);
vec3 indirectLight = 0.5f * vec3(1, 1, 1);
float focalLength = SCREEN_WIDTH;
float offset = 0.5-1/(sqrt(NUM_RAYS)*2);
float interval = 1/sqrt(NUM_RAYS);
float offset_l = 0.5-1/(sqrt(NUM_LIGHT_RAYS)*2);
float interval_l = 1/sqrt(NUM_LIGHT_RAYS);
vector<Photon> PhotonList(NUM_PHOTONS);


int main( int argc, char* argv[] )
{
  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  vector<Triangle> triangles;
  LoadTestModel(triangles);

  GenAreaLight(lightPositionArr, lightPosition);

  InitCastPhotons(triangles);

  //while( NoQuitMessageSDL() )
    {
      Update(cameraPos,cameraDirection);
      Draw(screen, triangles, cameraPos, cameraDirection);
      SDL_Renderframe(screen);
    }
  Update(cameraPos,cameraDirection);
  std::cin.get();

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}


vec3 getAntiAliasingAvg(int x, int y, vector<Triangle>& triangles, vec4& cameraPos,
                        mat4& cameraDirection, vector<Intersection>& intersectionArray, bool &check_intersection){

  check_intersection = true;

  vec3 totalColor = vec3(0.f,0.f,0.f);
  vec3 avgColor;
  int index = 0;
  bool shadowPixel;
  bool intersection;
  Intersection triangleIntersection;
  float numForAvg = 0.f;
  vec3 shadedPixelTotal;

  for(float j = -offset; j <= offset; j += interval) {
    for(float i = -offset; i <= offset; i += interval) {
      vec4 d(x- SCREEN_WIDTH/2 + i, y - SCREEN_HEIGHT/2 + j, focalLength, 1);
      intersection = ClosestIntersection(cameraPos, cameraDirection*d, triangles, triangleIntersection);
      if(!intersection) { continue;}

      //DIRECT LIGHT
      // shadedPixelTotal = vec3(0,0,0);
      // for(int idx = 0; idx < NUM_LIGHT_RAYS; idx++){
      //   vec4 lightPos = lightPositionArr[idx];
      //   vec3 tmpShadedPixel = DirectLight(triangleIntersection,triangles,shadowPixel,lightPos);
      //   shadedPixelTotal.x += tmpShadedPixel.x;
      //   shadedPixelTotal.y += tmpShadedPixel.y;
      //   shadedPixelTotal.z += tmpShadedPixel.z;
      // }
      // vec3 shadedPixel = vec3(shadedPixelTotal.x / (NUM_LIGHT_RAYS), shadedPixelTotal.y / (NUM_LIGHT_RAYS), shadedPixelTotal.z / (NUM_LIGHT_RAYS));
      vec3 directLight = AreaLight(triangleIntersection,triangles,shadowPixel);

      numForAvg+=1;

      vec3 photonLight = GatherPhotons(triangleIntersection.position, triangles[triangleIntersection.triangleIndex]);
      totalColor += triangles[triangleIntersection.triangleIndex].color*(indirectLight*directLight) + photonLight;
    }
  }
  avgColor = totalColor / numForAvg;
  return avgColor;

}


/*Place your drawing here*/
void Draw(screen* screen, vector<Triangle>& triangles, vec4& cameraPos, mat4& cameraDirection)
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  vector<Intersection> intersectionArray(NUM_RAYS);
  bool all_intersections;

  #pragma omp parallel for collapse(2)
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
        lightPosition += (forward*0.05f);
      }
      if(keystate[SDL_SCANCODE_S]){
        lightPosition -= (forward*0.05f);
      }
      if(keystate[SDL_SCANCODE_A]){
        lightPosition -= (right*0.05f);
      }
      if(keystate[SDL_SCANCODE_D]){
        lightPosition += (right*0.05f);
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

vec3 DirectLight(const Intersection& i, vector<Triangle>& triangles, bool& shadowPixel, const vec4& lightPos){
  vec4 position = i.position;
  int triangleIndex = i.triangleIndex;

  vec4 r = normalize(lightPos - position); // r -> Direction to light.
  float length_r = glm::length(lightPos - position);

  vec4 normal = triangles[triangleIndex].normal;
  float rNorm = dot(r,normal);
  rNorm = max(rNorm,0.f);

  vec4 D(0,0,0,0);

  bool intersection;
  Intersection thisIntersection;
  intersection = ClosestIntersection(position+0.001f*r, r, triangles, thisIntersection);
  if(intersection){
    float distToClosestIntesect = thisIntersection.distance;
    if(length_r >= distToClosestIntesect){
      shadowPixel = true;
      D = vec4(0,0,0,0);
    }
    else {
      shadowPixel = false;
      D = (lightColor*rNorm)/(float)(4*pi*length_r*length_r);
    }
  } else {
    shadowPixel = false;
    D = (lightColor*rNorm)/(float)(4*pi*length_r*length_r);
  }

  return vec3(D.x,D.y,D.z);
}

vec3 AreaLight(const Intersection& i, vector<Triangle>& triangles, bool& shadowPixel){
  vec4 position = i.position;
  int triangleIndex = i.triangleIndex;
  vec3 totalPixelIntensity = vec3(0.f, 0.f, 0.f);

  for(int i = 0; i < NUM_LIGHT_RAYS; i++){
    vec4 lightPos = lightPositionArr[i];

    vec4 r = glm::normalize(lightPos - position); // r -> Direction to light.
    float length_r = glm::length(lightPos - position);

    vec4 normal = glm::normalize(triangles[triangleIndex].normal);
    float rNorm = dot(r,normal);
    rNorm = max(rNorm,0.f);

    vec4 D;

    bool intersection;
    Intersection thisIntersection;
    intersection = ClosestIntersection(position+0.001f*r, r, triangles, thisIntersection);
    if(intersection){
      float distToClosestIntersect = thisIntersection.distance;
      if(length_r > distToClosestIntersect){
        shadowPixel = true;
        D = vec4(0,0,0,0);
      } else {
        shadowPixel = false;
        D = (lightColor*rNorm)/(float)(4*pi*length_r*length_r);
      }
    } else {
      shadowPixel = false;
      D = (lightColor*rNorm)/(float)(4*pi*length_r*length_r);
    }

    totalPixelIntensity += toVec3(D);
  }
  vec3 avgPixelIntensity = totalPixelIntensity / (float) NUM_LIGHT_RAYS;
  return avgPixelIntensity;
}

void GenAreaLight(vector<vec4>& lightPositionArr, const vec4& lightPosition){ // TODO: make like positions random
  vec4 tmpLightPosition;
  int index = 0;
  float min = -0.33;
  float max = 0.33;
  for(int i = 0; i < NUM_LIGHT_RAYS; i++){ //Can also randomize light positions
    float offset_x = glm::linearRand(min, max);
    float offset_z = glm::linearRand(min, max);
    lightPositionArr[i] = vec4(lightPosition.x + offset_x, lightPosition.y, lightPosition.z + offset_z, lightPosition.w);
  }
  // for(float j = -offset_l; j <= offset_l; j += interval_l) {
  //   for(float i = -offset_l; i <= offset_l; i += interval_l) {
  //     tmpLightPosition.x = lightPosition.x + j;
  //     tmpLightPosition.y = lightPosition.y;
  //     tmpLightPosition.z = lightPosition.z + i;
  //     tmpLightPosition.w = lightPosition.w;
  //     lightPositionArr[index] = tmpLightPosition;
  //     index += 1;
  //   }
  // }
}

void InitCastPhotons(vector<Triangle> triangles){
  float min = -0.99f, max = 0.99f;

  for(int i = 0; i < NUM_PHOTONS; i++){
    vec3 color = vec3(1.f,1.f,1.f); // vec3(lightColor);
    vec3 position = toVec3(lightPosition);
    vec3 randPhotonDir = glm::linearRand<float>(glm::vec3(-1.0f, -1.0f, -1.0f), glm::vec3(1.0f, 1.0f, 1.0f));
    randPhotonDir = glm::normalize(randPhotonDir);

    Intersection thisIntersection;
    bool initialRay;

    for(int bounce = 0; bounce < NUM_BOUNCES; bounce++){
      if(ClosestIntersection(toVec4(position + randPhotonDir*0.001f), toVec4(randPhotonDir), triangles, thisIntersection)){
        Photon thisPhoton;
        position = toVec3(thisIntersection.position);
        thisPhoton.position = position;
        color.x *= triangles[thisIntersection.triangleIndex].color.x;
        color.y *= triangles[thisIntersection.triangleIndex].color.y;
        color.z *= triangles[thisIntersection.triangleIndex].color.z;
        thisPhoton.color = color;
        thisPhoton.triangle_normal = toVec3(triangles[thisIntersection.triangleIndex].normal);
        randPhotonDir = thisPhoton.triangle_normal;
        randPhotonDir = glm::rotateX(randPhotonDir, glm::linearRand(min, max) * (pi/2.f)); //0.99 so that can't intersect with itself
        randPhotonDir = glm::rotateY(randPhotonDir, glm::linearRand(min, max) * (pi/2.f));
        randPhotonDir = glm::rotateZ(randPhotonDir, glm::linearRand(min, max) * (pi/2.f));
        randPhotonDir = normalize(randPhotonDir);

        if(!initialRay) PhotonList.push_back(thisPhoton);
        initialRay = false;
      } else break;
    }

  }
}

vec3 GatherPhotons(vec4 rayPos, Triangle& triangle){
  //GLOBAL ILLUMINATION
  vec3 totalColor = vec3(0.f,0.f,0.f);
  int numNeighbors = 0;
  //float searchRadius = 0.4f;
  float searchRadius2 = 0.16f; //Squared the searchRadius to avoid expensive SQRT with glm::distance2
  for(size_t p = 0; p < PhotonList.size(); p++){
    Photon& thisPhoton = PhotonList[p];
    float distanceToPhoton = glm::distance2(thisPhoton.position, toVec3(rayPos));
    float dot = glm::dot(thisPhoton.triangle_normal, toVec3(triangle.normal));
    if(distanceToPhoton < searchRadius2 && dot > 0.90f){
      totalColor += thisPhoton.color;
      numNeighbors++;
    }
  }
  float numNeighbors_f = (float)numNeighbors;
  vec3 avgColor = vec3((totalColor)/(numNeighbors_f));
  return avgColor;
}

vec3 toVec3(vec4 vector){
  return vec3(vector.x, vector.y, vector.z);
}
vec4 toVec4(vec3 vector){
  return vec4(vector.x, vector.y, vector.z, 1.0f);
}
