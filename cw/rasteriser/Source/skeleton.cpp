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

/*
  ---------------------------------------------------
  VARIABLES
*/

vector<Triangle> triangles;
vec4 cameraPos( 0, 0, -3.001,1);
glm::mat4 R;
float yaw = 0; // Yaw angle controlling camera rotation around y-axis
mat4 cameraDirection =  rotation(0);

void VertexShader(vec4 vertices, ivec2& projPos) {
  // std::cout << vertices << std::endl;
  vertices = vec4(cameraDirection*vec4(vertices - cameraPos));
  projPos.x = (FOCAL_LENGTH * (vertices.x)/(vertices.z)) + (SCREEN_WIDTH/2);
  projPos.y = (FOCAL_LENGTH * (vertices.y)/(vertices.z)) + (SCREEN_HEIGHT/2);
  // std::cout << projPos << std::endl;
}

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
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
  for(uint32_t i=0; i<triangles.size(); ++i)
  {
    vector<vec4> vertices(3);
    vector<ivec2> vecProjPos(3);
    vertices[0] = triangles[i].v0;
    vertices[1] = triangles[i].v1;
    vertices[2] = triangles[i].v2;

    DrawPolygonEdges(vertices, screen);

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
  for(int i = 0; i < line.size(); i++){
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
