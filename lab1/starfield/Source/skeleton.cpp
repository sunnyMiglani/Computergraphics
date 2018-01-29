#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModel.h"
#include <stdint.h>

//using namespace std;
using std::cout;
using std::endl;
using glm::vec3;
using glm::mat3;

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false
#define f_length SCREEN_HEIGHT/2
#define STAR_VELOCITY 0.2

/* ----------------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                            */
 int t;

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update(std::vector<vec3>& stars);
void Draw(screen* screen, std::vector<vec3>& stars);
void Interpolate_f(float a, float b, std::vector<float>& result);
void Interpolate(vec3 a, vec3 b, std::vector<vec3>& result);
void test_Interpolation();
void DrawColour();
void StarField();
void init_stars(std::vector<vec3>& stars);
void updateValues(std::vector<vec3>& stars, float dt);

void test_Interpolation(){
  std::vector<vec3> result(4);
  vec3 a(1,4,9.2);
  vec3 b(4,1,9.8);
  Interpolate(a,b,result);
  for(uint i = 0; i < result.size(); ++i){
    cout << "("
         << result[i].x <<","
         << result[i].y << ","
         << result[i].z << ")";

  }
}



int main( int argc, char* argv[] )
{
  std::vector<vec3> stars(1000);
  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  t = SDL_GetTicks();	/*Set start value for timer.*/
  init_stars(stars);

  while( NoQuitMessageSDL() )
    {
      Update(stars);
      Draw(screen,stars);
      SDL_Renderframe(screen);
      cout << "hello?" <<endl;
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}



float getRandNumNeg(){
  float x = float(rand()/ float(RAND_MAX));
  // 0 < x <= 1
  // Convert to -1 to 1 --> Times the x by 2, then minus 1.
  x*=2;
  x-=1;
  //printf("X : %f",x);
  // printf("%f is returned by randNeg()\n",x);
  if (x > 1 || x < -1){printf("RandNegFailed \n");}
  return x;

}

float getRandNum(){
  float x = float(rand()/ float(RAND_MAX));
  if (x > 1 || x < 0){printf("###Rand Failed### \n");}
  // printf("%f is returned by rand()\n",x);
  return x;
}

float getUValue(float x_val,float z_val){
  float ans = (f_length * (x_val/z_val)) + SCREEN_WIDTH/2;
  // printf("%f is U\n",ans);
  return ans;
}

float getVValue(float y_val, float z_val){
  float ans = (f_length * (y_val/z_val)) + SCREEN_HEIGHT/2;
  // printf("%f is V\n",ans)
  return ans;
}

void updateValues(std::vector<vec3>& stars, float dt){
  float velocity = 0.2;
  for(int i = 0; i < 1000; i++){
    vec3& star = stars[i];
    float cur_val = star.z;
    float nxt_val = cur_val - velocity * dt * 0.001;
    if (nxt_val > 1 || nxt_val < 0) {
      star.x = getRandNumNeg();
      star.y = getRandNumNeg();
      star.z = getRandNum();
    }
    else star.z = nxt_val;
    printf("nxt_val : %f, new nxt_val : %f  and dt : %f \n",nxt_val,star.z,dt);
  }

  return;
}


void init_stars(std::vector<vec3>& stars){
  for(int i = 0 ; i < 1000; i++){
      stars[i].x = getRandNumNeg();
      stars[i].y = getRandNumNeg();
      stars[i].z = getRandNum();
  }
}

/*Place your drawing here*/
void Draw(screen* screen, std::vector<vec3>& stars)
{

  /* Clear buffer */
  vec3 colour(1,1,1);
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
   for(int i=0; i<1000; i++)
   {
     float u = getUValue(stars[i].x, stars[i].z);
     float v = getVValue(stars[i].y, stars[i].z);
     cout << "u: " << u << "v: " << v << endl;
     PutPixelSDL(screen, u, v ,colour);
   }
   cout << "Render finished" << endl;
}



/*Place updates of parameters here*/
void Update(std::vector<vec3>& stars)
{
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
  cout << "Render time: " << dt << " ms." << endl;
  /* Update variables*/
  updateValues(stars, dt);

}



void Interpolate_f(float a, float b, std::vector<float>& result){
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

void Interpolate(vec3 a, vec3 b, std::vector<vec3>& result){
    float a0 = a.x;
    float a1 = a.y;
    float a2 = a.z;

    float b0 = b.x;
    float b1 = b.y;
    float b2 = b.z;

    int size = result.size();

    std::vector<float> resultX(size);
    std::vector<float> resultY(size);
    std::vector<float> resultZ(size);

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

  void DrawColour(){
    screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
    vec3 topLeft(1, 0, 0); //Red
    vec3 topRight(0, 1, 0); //Green
    vec3 bottomRight(0, 0, 1); //Blue
    vec3 bottomLeft(1, 1, 0); //Yellow

    std::vector<vec3> leftSide( SCREEN_HEIGHT );
    std::vector<vec3> rightSide( SCREEN_HEIGHT );
    Interpolate(topLeft, bottomLeft, leftSide);
    Interpolate(topRight, bottomRight, rightSide);
    std::vector<vec3> screen_row(SCREEN_WIDTH);


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
        PutPixelSDL(screen, col, row, screen_row[col]);
      }
    }
  }
