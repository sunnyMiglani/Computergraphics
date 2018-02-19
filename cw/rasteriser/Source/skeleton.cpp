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


#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false
#define FOCAL_LENGTH (SCREEN_HEIGHT/2)


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
void VertexShader( vertices, projPos );

/*
  ---------------------------------------------------
  VARIABLES
*/

vector<Triangle> triangles;
vec4 cameraPos( 0, 0, -3.001,1 );



void VertexShader( vertices, projPos ){
  float x = (FOCAL_LENGTH * (vertices.x)/(vertices.z)) + (SCREEN_WIDTH/vertices.z);
  float y = (FOCAL_LENGTH * (vertices.y)/(vertices.z)) + (SCREEN_HEIGHT/vertices.z);
  projPos.x = x;
  projPos.y = y;
}





int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  LoadTestModel(triangles);


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

void Draw()
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
  for( uint32_t i=0; i<triangles.size(); ++i )
  {
    vector<vec4> vertices(3);
    vertices[0] = triangles[i].v0;
    vertices[1] = triangles[i].v1;
    vertices[2] = triangles[i].v2;
    for(int v=0; v<3; ++v)
    {
      ivec2 projPos; // x and y
      VertexShader( vertices[v], projPos );
      vec3 color(1,1,1);
      PutPixelSDL( screen, projPos.x, projPos.y, color);
      }
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
