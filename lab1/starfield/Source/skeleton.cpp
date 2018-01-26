#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModel.h"
#include <stdint.h>

using namespace std;
using glm::vec3;
using glm::mat3;

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false


/* ----------------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                            */
int t; // moved to Update function as a static int

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
void Interpolate(float a, float b, vector<float>& result);

int main( int argc, char* argv[] )
{

  vector<float> result(10);
  Interpolate(5,14,result);
  for(int i = 0; i < result.size(); ++i){
    cout << result[i] << " ";
  }

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  t = SDL_GetTicks();	/*Set start value for timer.*/

  // while( NoQuitMessageSDL() )
  //   {
  //     Draw(screen);
  //     Update();
  //     SDL_Renderframe(screen);
  //   }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen)
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  vec3 colour(0.1,0.2,0.7);
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
  /* Compute frame time */
  // static int t = SDL_GetTicks();
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
  std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/
}


void Interpolate(float a, float b, vector<float>& result){
  if(result.size() == 1){
    result[0] = a;
    return;
  }
  float range = (b-a);
  float step = range/(result.size()-1);
  float insertvalue = a;
  result[0] = a;
  result[result.size()-1] = b;
  for (float i = 1; i <= result.size() - 1; i++){
    // Need to set insertValue as the next value to be put into result
    insertvalue += step;
    result[i] = insertvalue;

  }
}
