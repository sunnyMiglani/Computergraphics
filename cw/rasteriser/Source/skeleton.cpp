#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <glm/gtc/matrix_transform.hpp>

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

#define SCREEN_WIDTH 420
#define SCREEN_HEIGHT 420
#define FULLSCREEN_MODE false
#define CHECKING_KEY_STATE true
#define FOCAL_LENGTH (SCREEN_HEIGHT*0.98)
#define SHOW_LIGHT true

#define EDGE_THRESHOLD_MIN 0.0312
#define EDGE_THRESHOLD_MAX 0.125

#define ITERATIONS (12)
#define SUBPIXEL_QUALITY 0.75



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
bool getLightDepth(Pixel p);
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
vec3 indirectLightPowerPerArea = 0.7f*vec3( 1, 1, 1 );
vec3 reflectanceGlobal = vec3(1,1,1);
vec3 torchPos = vec3(1,1,1);
mat4 torchDir = cameraDirection;
float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];
float shadowBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];
Pixel shadowPixels[SCREEN_HEIGHT][SCREEN_WIDTH];
vec3 screenBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];
mat4 inverseCameraRotation; // = glm::inverse(cameraDirection);
bool renderShadow = false;
bool antiAlias = true;

float QUALITY[] = { 1.5f,2.0f, 2.0f, 2.0f, 2.0f, 4.0f, 8.0f };


struct Camera {


    static float fovy;
    static float aspectRatio;
    static float near;
    static float far;
    vec3 cameraPos;
    vec3 cameraDir;
    vec3 cameraUp;


    mat4 viewMatrix;
    mat4 projectionMatrix;


    void computeViewMatrix(){
        viewMatrix = glm::lookAt(cameraPos, cameraPos + cameraDir, cameraUp );
    }

    void computeProjectionMatrix(){
        projectionMatrix =  glm::perspective(fovy, aspectRatio, near, far);
    }
};

Camera myCamera;
Camera shadowCamera;


/*
 ----------------------------------------------------
*/


mat4 rotation(float yaw){
  mat4 R_y(cos(yaw), 0, sin(yaw), 0,
              0    , 1,    0    , 0,
          -sin(yaw), 0, cos(yaw), 0,
              0    , 0,    0    , 1);
  return R_y;
}



void initScene(){
    myCamera.cameraPos = vec3(0.f, 0.f, -5.f);
    myCamera.cameraDir = vec3(0.f, 0.f, 1.f);
    myCamera.fovy = glm::radians(60.f);
    myCamera.aspectRatio = (float)(SCREEN_WIDTH/SCREEN_HEIGHT);
    myCamera.near = 0.01f;
    myCamera.far = 1000.f;
//

    shadowCamera.cameraPos = vec3(0.f, 1.f, -2.5f);//myCamera.cameraPos;//vec3(0.f,1.f,0.f)//vec3(0,-0.5,2.5);
    shadowCamera.cameraDir = myCamera.cameraDir; //vec3(0.3f, -1.f, 0.2f);
    shadowCamera.cameraUp = myCamera.cameraUp; //vec3(0.f, 0.f, 1.f);
    shadowCamera.fovy = glm::radians(80.f);
    shadowCamera.aspectRatio = (float)(SCREEN_WIDTH/SCREEN_HEIGHT);
    shadowCamera.near = 0.01f;
    shadowCamera.far = 100.f;

    // myCamera = shadowCamera;

}


int main(int argc, char* argv[])
{
    initScene();

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


/*
    FxAA : Fast approximate anti aliasing
    http://blog.simonrodriguez.fr/articles/30-07-2016_implementing_fxaa.html



*/
float rgb2luma(vec3 rgb){
    return sqrt(dot(rgb, vec3(0.299, 0.587, 0.114)));
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
         // if(keystate[SDL_SCANCODE_UP]){
         //   myCamera.cameraPos += (myCamera.cameraDir * 0.05f);
         // }
         // if(keystate[SDL_SCANCODE_DOWN]){
         //   myCamera.cameraPos -= (myCamera.cameraDir * 0.05f);
         // }
         // if(keystate[SDL_SCANCODE_LEFT]){
         //   yaw -= 0.004;
         //   shadowCamera.cameraDir = vec3(rotation(yaw) * vec4(shadowCamera.cameraDir, 0));
         // }
         // if(keystate[SDL_SCANCODE_RIGHT]){
         //   yaw += 0.004;
         //   shadowCamera.cameraDir = vec3(rotation(yaw) * vec4(shadowCamera.cameraDir, 0));
         // }
         //Move Light Source
        if(keystate[SDL_SCANCODE_UP]){
            shadowCamera.cameraPos += (shadowCamera.cameraUp)  * 0.05f;
            std::cout << "Pos : " << shadowCamera.cameraPos << std::endl;
           }
           if(keystate[SDL_SCANCODE_DOWN]){
             shadowCamera.cameraPos -= (shadowCamera.cameraUp)  * 0.05f;
             std::cout << "Pos : " << shadowCamera.cameraPos << std::endl;
           }
           if(keystate[SDL_SCANCODE_LEFT]){
             shadowCamera.cameraPos -= vec3(shadowCamera.cameraDir *0.05f);
             std::cout << "Pos : " << shadowCamera.cameraPos << std::endl;
           }
           if(keystate[SDL_SCANCODE_RIGHT]){
             shadowCamera.cameraPos += vec3(shadowCamera.cameraDir *0.05f);
             std::cout << "Pos : " << shadowCamera.cameraPos << std::endl;
           }
        if(keystate[SDL_SCANCODE_G]){
            std::cout << "Shifting cams " << '\n';
            renderShadow = !renderShadow;
        }
        if(keystate[SDL_SCANCODE_B]){
            std::cout << "Removed AntiAliasing" << '\n';
            antiAlias != antiAlias;
        }
       }
   }

   myCamera.computeViewMatrix();
   myCamera.computeProjectionMatrix();

   shadowCamera.computeViewMatrix();
   shadowCamera.computeProjectionMatrix();
}


void populateShadowBuffer(){

    for(uint32_t i = 0; i < triangles.size(); i++){

      Triangle triangle = triangles[i];
      // Transform each vertex from 3D world position to 2D image position:

      vec3 normal  = vec3(triangle.normal);
      if(dot(myCamera.cameraDir, normal) > 0){
          // Facing in the same direction, so ignore
          continue;
      }
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
            if (col < 0 || col >= SCREEN_WIDTH) continue;

            Pixel tPixel;

            tPixel.normal = normal;

            int y = row;
            int x = col;

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

            if (0 <= u && u <= 1 && 0 <= v && v <= 1 && 0 <= w && w <= 1) {
              tPixel.zinv = v0.zinv * u +  v1.zinv * v + v2.zinv * w; // interpolate the zinv
              if(tPixel.zinv > shadowBuffer[row][col]){
                  shadowBuffer[row][col] = tPixel.zinv; // apply the zinv in the buffer
                  tPixel.worldPos = (v0.worldPos * v0.zinv * u
                              + v1.worldPos * v1.zinv * v
                              + v2.worldPos * v2.zinv * w) / tPixel.zinv;
                          }
                      }
                  }
              }
          }
      }

void applyAntiAliasing(){

    for(int row = 0; row < SCREEN_HEIGHT; row++){ // looping through the square
      for(int col = 0; col < SCREEN_WIDTH; col++){
        vec3 colourCentre = screenBuffer[col][row];
        float lumaCentre = rgb2luma(colourCentre);
        float lumaUp = 0;
        float lumaDown = 0;
        float lumaLeft = 0;
        float lumaRight = 0;

        float currentMin = 1000000;
        float currentMax = -100000;

        if(row-1 >= 0){
            lumaUp = rgb2luma(screenBuffer[col][row-1]);
            currentMin = (currentMin < lumaUp) ? lumaUp:currentMin;
            currentMax = (currentMax > lumaUp) ? lumaUp:currentMax;
        }
        // else { continue; }
        if(row+1 < SCREEN_HEIGHT ){
            lumaDown = rgb2luma(screenBuffer[col][row+1]);
            currentMin = (currentMin < lumaDown) ? lumaDown:currentMin;
            currentMax = (currentMax > lumaDown) ? lumaDown:currentMax;
        }
        // else { continue; }
        if(col-1 >= 0){
            lumaLeft = rgb2luma(screenBuffer[col-1][row]);
            currentMin = (currentMin < lumaLeft) ? lumaLeft:currentMin;
            currentMax = (currentMax > lumaLeft) ? lumaLeft:currentMax;
        }
        // else { continue; }
        if(col+1 < SCREEN_WIDTH){
            lumaRight = rgb2luma(screenBuffer[col+1][row]);
            currentMin = (currentMin < lumaRight) ? lumaRight:currentMin;
            currentMax = (currentMax > lumaRight) ? lumaRight:currentMax;
        }
        // else { continue; }
        float lumaRange = currentMax - currentMin;

        if(lumaRange < max(EDGE_THRESHOLD_MIN, currentMax * EDGE_THRESHOLD_MAX)){
            continue;
        }
        std::cout << "Made it past the contiues " << '\n';

        float lumaDownLeft = rgb2luma(screenBuffer[col-1][row-1]);
        float lumaUpRight = rgb2luma(screenBuffer[col+1][row+1]);
        float lumaUpLeft = rgb2luma(screenBuffer[col-1][row+1]);
        float lumaDownRight = rgb2luma(screenBuffer[col][row-1]);


        float lumaDownUp = lumaDown + lumaUp;
        float lumaLeftRight = lumaLeft + lumaRight;

        float lumaLeftCorners = lumaDownLeft + lumaUpLeft;
        float lumaDownCorners = lumaDownLeft + lumaDownRight;
        float lumaRightCorners = lumaDownRight + lumaUpRight;
        float lumaUpCorners = lumaUpRight + lumaUpLeft;

        float edgeHorizontal =  abs(-2.0 * lumaLeft + lumaLeftCorners)  + abs(-2.0 * lumaCentre + lumaDownUp ) * 2.0    + abs(-2.0 * lumaRight + lumaRightCorners);
        float edgeVertical =    abs(-2.0 * lumaUp + lumaUpCorners)      + abs(-2.0 * lumaCentre + lumaLeftRight) * 2.0  + abs(-2.0 * lumaDown + lumaDownCorners);

        bool isHorizontal = (edgeHorizontal >= edgeVertical);


        float luma1 = isHorizontal ? lumaDown : lumaLeft;
        float luma2 = isHorizontal ? lumaUp : lumaRight;

        float gradient1 = luma1 - lumaCentre;
        float gradient2 = luma2 - lumaCentre;

        bool is1Steepest = abs(gradient1) >= abs(gradient2);

        float gradientScaled = 0.25*max(abs(gradient1),abs(gradient2));


        float stepLength = isHorizontal ? SCREEN_WIDTH : SCREEN_HEIGHT;

        float lumaLocalAverage = 0.0;

        if(is1Steepest){
            stepLength = - stepLength;
            lumaLocalAverage = 0.5*(luma1 + lumaCentre);
        } else {
            lumaLocalAverage = 0.5*(luma2 + lumaCentre);
        }

        vec2 currentUv = vec2(col,row);
        if(isHorizontal){
            currentUv.y += stepLength * 0.5;
        } else {
            currentUv.x += stepLength * 0.5;
        }

        vec2 offset = isHorizontal ? vec2(SCREEN_HEIGHT,0.0) : vec2(0.0,SCREEN_WIDTH);
        // Compute UVs to explore on each side of the edge, orthogonally. The QUALITY allows us to step faster.
        vec2 uv1 = currentUv - offset;
        vec2 uv2 = currentUv + offset;

        // Read the lumas at both current extremities of the exploration segment, and compute the delta wrt to the local average luma.
        float lumaEnd1 = rgb2luma(screenBuffer[int(uv1.y)][int(uv1.x)]);
        float lumaEnd2 = rgb2luma(screenBuffer[int(uv2.y)][int(uv2.x)]);
        lumaEnd1 -= lumaLocalAverage;
        lumaEnd2 -= lumaLocalAverage;

        // If the luma deltas at the current extremities are larger than the local gradient, we have reached the side of the edge.
        bool reached1 = abs(lumaEnd1) >= gradientScaled;
        bool reached2 = abs(lumaEnd2) >= gradientScaled;
        bool reachedBoth = reached1 && reached2;

        // If the side is not reached, we continue to explore in this direction.
        if(!reached1){
            uv1 -= offset;
        }
        if(!reached2){
            uv2 += offset;
        }
        if(!reachedBoth){

        for(int i = 2; i < ITERATIONS; i++){
            // If needed, read luma in 1st direction, compute delta.
            if(!reached1){
                lumaEnd1 = rgb2luma(screenBuffer[int(uv1.y)][int(uv1.x)]);
                lumaEnd1 = lumaEnd1 - lumaLocalAverage;
            }
            // If needed, read luma in opposite direction, compute delta.
            if(!reached2){
                lumaEnd2 = rgb2luma(screenBuffer[int(uv2.y)][int(uv2.x)]);
                lumaEnd2 = lumaEnd2 - lumaLocalAverage;
            }
            // If the luma deltas at the current extremities is larger than the local gradient, we have reached the side of the edge.
            reached1 = abs(lumaEnd1) >= gradientScaled;
            reached2 = abs(lumaEnd2) >= gradientScaled;
            reachedBoth = reached1 && reached2;

            // If the side is not reached, we continue to explore in this direction, with a variable quality.
            if(!reached1){
                uv1 -= offset;
            }
            if(!reached2){
                uv2 += offset;
            }

            // If both sides have been reached, stop the exploration.
            if(reachedBoth){ break;}
            }
        }
        // Compute the distances to each extremity of the edge.
        float distance1 = isHorizontal ? (screenBuffer[col][row].x - uv1.x) : (screenBuffer[col][row].y - uv1.y);
        float distance2 = isHorizontal ? (uv2.x - screenBuffer[col][row].x) : (uv2.y - screenBuffer[col][row].y);

        // In which direction is the extremity of the edge closer ?
        bool isDirection1 = distance1 < distance2;
        float distanceFinal = min(distance1, distance2);

        // Length of the edge.
        float edgeThickness = (distance1 + distance2);

        // UV offset: read in the direction of the closest side of the edge.
        float pixelOffset = - distanceFinal / edgeThickness + 0.5;

        bool isLumaCentreSmaller = lumaCentre < lumaLocalAverage;

        bool correctVariation = ((isDirection1 ? lumaEnd1 : lumaEnd2) < 0.0) != isLumaCentreSmaller;

        float finalOffset = correctVariation ? pixelOffset : 0.0;
        // Sub-pixel shifting
        // Full weighted average of the luma over the 3x3 neighborhood.
        float lumaAverage = (1.0/12.0) * (2.0 * (lumaDownUp + lumaLeftRight) + lumaLeftCorners + lumaRightCorners);
        // Ratio of the delta between the global average and the Centre luma, over the luma range in the 3x3 neighborhood.
        float subPixelOffset1 = glm::clamp(abs(lumaAverage - lumaCentre)/lumaRange,0.0f,1.0f);
        float subPixelOffset2 = (-2.0 * subPixelOffset1 + 3.0) * subPixelOffset1 * subPixelOffset1;
        // Compute a sub-pixel offset based on this delta.
        float subPixelOffsetFinal = subPixelOffset2 * subPixelOffset2 * SUBPIXEL_QUALITY;

        // Pick the biggest of the two offsets.
        finalOffset = max(finalOffset,subPixelOffsetFinal);
        // Compute the final UV coordinates.
        vec2 finalUv = vec2(col,row);
        if(isHorizontal){
            finalUv.y += finalOffset * stepLength;
        } else {
            finalUv.x += finalOffset * stepLength;
        }

        // Read the color at the new UV coordinates, and use it.
        vec3 finalColor = screenBuffer[int(finalUv.y)][int(finalUv.x)];//texture(screenTexture,finalUv).rgb;
        screenBuffer[col][row] = finalColor;
    }
}
}



void putAllPixels(screen *screen){
    for(int row = 0; row < SCREEN_HEIGHT; row++){
      for(int col = 0; col < SCREEN_WIDTH; col++){
          PutPixelSDL(screen,col,row,screenBuffer[col][row]);

      }
    }
}

void Draw(screen *screen)
{
    // lightPos = vec3(cameraPos.x, cameraPos.y, cameraPos.z);

  for(int y = 0; y < SCREEN_HEIGHT; y++){ // Set the depth buffer to max values
    for(int x = 0; x < SCREEN_WIDTH; x++){
      depthBuffer[y][x] = 0;//-numeric_limits<int>::max();
      shadowBuffer[y][x] = 0; // 1 is far, -1 is near
    }
  }
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  populateShadowBuffer();

  for(uint32_t i = 0; i < triangles.size(); i++){

    Triangle triangle = triangles[i];
    // Transform each vertex from 3D world position to 2D image position:

    vec3 normal  = vec3(triangle.normal);
    if(dot(myCamera.cameraDir, normal) > 0){
        // Facing in the same direction, so ignore
        continue;
    }

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

      vertexPixels[i].projectPos /= vertexPixels[i].projectPos.w;
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
            tPixel.zinv = v0.zinv * u +  v1.zinv * v + v2.zinv * w; // interpolate the zinv
            if(tPixel.zinv > depthBuffer[row][col]){
                depthBuffer[row][col] = tPixel.zinv;
                tPixel.worldPos = (v0.worldPos * v0.zinv * u
                            + v1.worldPos * v1.zinv * v
                            + v2.worldPos * v2.zinv * w) / tPixel.zinv;

                // tPixel.viewPos = (v0.viewPos * v0.zinv * u
                //             + v1.viewPos * v1.zinv * v
                //             + v2.viewPos * v2.zinv * w) / tPixel.zinv;

                vec3 r_vec = shadowCamera.cameraPos - tPixel.worldPos;

                float length_r = 0.5+ glm::length(r_vec);
                float rNorm = glm::dot(glm::normalize(r_vec),normal);

                if (rNorm < 0) rNorm = 0;
                // dVal is the Power of the _incoming_ light.
                vec3 dVal = lightPower * (rNorm / ((float)( 4.0f * pi * length_r * length_r)));

                tPixel.illumination = dVal + indirectLightPowerPerArea;
                vec3 pixelColour = triangle.color * tPixel.illumination;
                bool isLit = getLightDepth(tPixel);


                if(isLit){
                    pixelColour = pixelColour/5.0f;
                }

                if(renderShadow == false){
                    screenBuffer[col][row] = pixelColour;
                 // PutPixelSDL(screen, col, row, pixelColour );//vec3(shadowBuffer[row][col])* vec3(0.5) + vec3(0.5)
                }
                else{
                    screenBuffer[col][row] = vec3(shadowBuffer[row][col] * 0.5 + 0.5);
                    // PutPixelSDL(screen, col, row, vec3(shadowBuffer[row][col] * 0.5 + 0.5));//vec3(shadowBuffer[row][col])

                }
            }
        }
    }
}
}
    if(antiAlias) {
        applyAntiAliasing();
    }
    putAllPixels(screen);
}


bool getLightDepth(Pixel p){


    vec4 posV4 = shadowCamera.projectionMatrix * (shadowCamera.viewMatrix * vec4(p.worldPos,1));
    posV4 = posV4 / posV4.w;
    int x = (int) (FOCAL_LENGTH * posV4.x) + (SCREEN_WIDTH/2);
    int y = (int) (FOCAL_LENGTH * -posV4.y) + (SCREEN_HEIGHT/2);
    if (y < 0 || y >= SCREEN_HEIGHT) return -1;
    if (x < 0 || x >= SCREEN_WIDTH) return -1;


    float shadowDepth = shadowBuffer[y][x];

    return (shadowDepth > (posV4.z));
}

void VertexShadowShader(vec4& vertex, Pixel& p){

    vec4 point = shadowCamera.viewMatrix * (vertex);
    p.viewPos = vec3(point); // view coordintes
    point =  shadowCamera.projectionMatrix * point;
    p.x = (int) (FOCAL_LENGTH * point.x/point.w) + (SCREEN_WIDTH/2);
    p.y = (int) (FOCAL_LENGTH * -point.y/point.w) + (SCREEN_HEIGHT/2);
    p.worldPos = vec3(vertex);
    p.zinv = 1.0f/point.z;
    p.projectPos = point;
}

// Uses the formula
/*
    D = P max(r.n, 0) / 4 pi r^2
    P is the power of the light source, r is a vector from the surface point to the light
    source and n̂ is the normal of the surface

*/
//
void VertexShader(vec4& vertex, Pixel& p){

  vec4 point = myCamera.viewMatrix * (vertex);
  p.viewPos = vec3(point); // view coordintes
  point =  myCamera.projectionMatrix * point;
  p.x = (int) (FOCAL_LENGTH * point.x/point.w) + (SCREEN_WIDTH/2);
  p.y = (int) (FOCAL_LENGTH * -point.y/point.w) + (SCREEN_HEIGHT/2);
  p.worldPos = vec3(vertex);  // world coordinates
  p.zinv = 1.0f/point.z;
  p.projectPos = point;
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
