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
int t;

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
void Interpolate_f(float a, float b, vector<float>& result);
void Interpolate(vec3 a, vec3 b, vector<vec3>& result);
void test_Interpolation();

void test_Interpolation(){
  vector<vec3> result(4);
  vec3 a(1,4,9.2);
  vec3 b(4,1,9.8);
  Interpolate(a,b,result);
  for(int i = 0; i < result.size(); ++i){
    cout << "("
         << result[i].x <<", "
         << result[i].y << ","
         << result[i].z << ")";

  }
}



int main( int argc, char* argv[] )
{
  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  t = SDL_GetTicks();	/*Set start value for timer.*/

  while( NoQuitMessageSDL() )
    {
      Draw(screen);
      Update();
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

  vec3 topLeft(1, 0, 0); //Red
  vec3 topRight(0, 1, 0); //Green
  vec3 bottomRight(0, 0, 1); //Blue
  vec3 bottomLeft(1, 1, 0); //Yellow

  vector<vec3> leftSide( SCREEN_HEIGHT );
  vector<vec3> rightSide( SCREEN_HEIGHT );
  Interpolate(topLeft, bottomLeft, leftSide);
  Interpolate(topRight, bottomRight, rightSide);
  vector<vec3> screen_row(SCREEN_WIDTH);


  //vec3 colour(1.0,0.0,0.0);
  // for(int i=0; i<1000; i++)
  //   {
  //     uint32_t x = rand() % screen->width;
  //     uint32_t y = rand() % screen->height;
  //     PutPixelSDL(screen, x, y, colour);
  //   }
  for(int row = 0; row < SCREEN_HEIGHT; row++){
    Interpolate(leftSide[row], rightSide[row], screen_row);
    for(int col = 0; col < SCREEN_WIDTH; col++){
      PutPixelSDL(screen, row, col, screen_row[col]);
    }
  }
}

/*Place updates of parameters here*/
void Update()
{
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
  std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/
}



void Interpolate_f(float a, float b, vector<float>& result){
  if(result.size() == 1){
    result[0] = (a+b)/2;
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

void Interpolate(vec3 a, vec3 b, vector<vec3>& result){
    float a0 = a.x;
    float a1 = a.y;
    float a2 = a.z;

    float b0 = b.x;
    float b1 = b.y;
    float b2 = b.z;

    int size = result.size();

    vector<float> resultX(size);
    vector<float> resultY(size);
    vector<float> resultZ(size);

    Interpolate_f(a0,b0,resultX);
    Interpolate_f(a1,b1,resultY);
    Interpolate_f(a2,b2,resultZ);


    for(int i = 0; i < size; i++){
      vec3 temp;// = (resultX[i], resultY[i], resultZ[i]);
      temp.x = resultX[i];
      temp.y = resultY[i];
      temp.z = resultZ[i];
      result[i] = temp;
    }
  }
