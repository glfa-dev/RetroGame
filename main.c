#include <stdio.h>
#include <stdbool.h>
#include <SDL.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define SCREEN_W        640
#define SCREEN_H        480
#define P_MOVE_SPEED    1.0f
#define P_ROT_SPEED     1.0f
#define P_FRICTION      16.0f
#define P_RADIUS        0.2f
#define NEAR_PLANE      0.01f
#define MOUSE_SENS      60.0f

#define LERP(x, y, a)   (x + a * (y - x))

#define MAP_SIZE        24

static const int c_map[MAP_SIZE][MAP_SIZE] =
{
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {1,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,1,1,1,0,1,1,1},
  {1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1},
  {1,1,1,1,0,1,1,1,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1},
  {1,1,0,0,0,0,0,0,1,1,0,1,0,1,0,1,1,1,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,1,0,1,0,1},
  {1,1,0,0,0,0,0,0,1,1,0,1,0,1,0,1,1,1,1,1,0,1,1,1},
  {1,1,1,1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0,1,1,1},
  {1,1,1,1,0,1,1,1,1,1,1,1,0,0,1,0,1,1,0,0,0,0,0,1},
  {1,1,0,0,0,0,0,1,1,1,0,0,0,0,0,0,1,1,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,1,1,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,1,1,1,0,0,0,1,1},
  {1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,1,0,1,0,1},
  {1,1,0,0,0,0,0,1,1,1,0,0,0,1,1,0,1,0,1,0,0,0,1,1},
  {1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,1,0,1,0,1,0,1,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,1,0,1,0,1,0,1,0,1},
  {1,1,0,0,0,0,0,1,1,1,0,0,0,1,1,0,1,0,1,0,0,0,1,1},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

typedef struct Vec2
{
    union
    {
        struct
        {
            float x;
            float y;
        };
        float f[2];
    };
}Vec2;

typedef struct AABB
{
    Vec2  min;
    Vec2  max;
}AABB;

typedef struct Player
{
    Vec2  origin;
    Vec2  velocity;
    float angle;
}Player;

double GetTime(void)
{
    double freq = (double)SDL_GetPerformanceFrequency();
    double time = (double)SDL_GetPerformanceCounter();
    return time / freq;
}

Vec2 Vec2_Rotate(Vec2 v, float angle)
{
    float _cos = SDL_cosf(angle);
    float _sin = SDL_sinf(angle);

    Vec2 result =
    {
        .x = v.x * _cos - v.y * _sin,
        .y = v.x * _sin + v.y * _cos
    };

    return result;
}

float Vec2_Dot(Vec2 v1, Vec2 v2)
{
    return (v1.x * v2.x) + (v1.y * v2.y);
}

Vec2 Vec2_Normalize(Vec2 v)
{
    float mag = SDL_sqrtf(Vec2_Dot(v, v));
    v.x = v.x / mag;
    v.y = v.y / mag;
    return v;
}

void ClearDepthBuffer(float* depthBuffer)
{
    for (int i = 0; i < SCREEN_W; i++)
    {
        depthBuffer[i] = INFINITY;
    }
}

void ClearColorBuffer(uint32_t* colorBuffer)
{
    SDL_memset(colorBuffer, 0xFF666666,
        sizeof(uint32_t) * SCREEN_W * SCREEN_H);
}

SDL_Surface* LoadTexture(const char* filename)
{
    int x, y, n;
    uint32_t* data = (uint32_t*)stbi_load(filename, &x, &y, &n, 4);

    if (data)
    {
        SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormat(0, x, y, 32, SDL_PIXELFORMAT_ABGR8888);
        if (surface)
        {
            uint32_t* pixels = surface->pixels;
            for (int i = 0; i < (x * y); i++)
            {
                pixels[i] = data[i];
            }
            return surface;
        }
    }
    
    return NULL;
}

int SDL_main(int argc, char* argv[])
{
    SDL_Init(SDL_INIT_EVERYTHING);

    SDL_Window* window = SDL_CreateWindow("MiniShooter",
        SDL_WINDOWPOS_CENTERED, 
        SDL_WINDOWPOS_CENTERED,
        SCREEN_W, SCREEN_H,
        SDL_WINDOW_HIDDEN);

    SDL_SetWindowFullscreen(window,
        SDL_WINDOW_FULLSCREEN_DESKTOP);

    SDL_Renderer* renderer = SDL_CreateRenderer(window,
        -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    SDL_RenderSetLogicalSize(renderer,
        SCREEN_W, SCREEN_H);

    SDL_Texture* renderTexture = SDL_CreateTexture(renderer,
        SDL_PIXELFORMAT_ABGR8888,
        SDL_TEXTUREACCESS_STREAMING,
        SCREEN_W, SCREEN_H);

    SDL_Surface* grassTex = LoadTexture("assets/grass.png");
    SDL_assert(grassTex->w == 64 && grassTex->h == 64);

    SDL_Surface* ceilTex = LoadTexture("assets/grid.png");
    SDL_assert(ceilTex->w == 64 && ceilTex->h == 64);

    SDL_Surface* brickTex = LoadTexture("assets/bricks.png");
    SDL_assert(brickTex->w == 64 && brickTex->h == 64);

    SDL_ShowWindow(window);

    SDL_SetRelativeMouseMode(SDL_TRUE);

    Player player =
    {
        .origin.x   = 1.5f,
        .origin.y   = 1.5f,
        .velocity.x = 0.0f,
        .velocity.y = 0.0f,
        .angle      = 1.5f
    };

    uint32_t* colorBuffer = SDL_malloc(SCREEN_W * SCREEN_H * sizeof(uint32_t));
    float*    depthBuffer = SDL_malloc(sizeof(float) * SCREEN_W);

    double lastTime = GetTime();
    double currTime = lastTime;

    bool isRunning = true;
    while (isRunning)
    {
        float forward   = 0.0f;
        float right     = 0.0f;
        float rotation  = 0.0f;

        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            switch (event.type)
            {
            case SDL_QUIT:
            {
                isRunning = false;
                break;
            }
            default:
                break;
            }
        }

        lastTime = currTime;
        currTime = GetTime();

        float dt = (float)(currTime - lastTime);

        // Update player
        {
            const uint8_t* keystate = SDL_GetKeyboardState(NULL);

            // Player input:
            {
                if (keystate[SDL_SCANCODE_ESCAPE])
                    isRunning = false;

                if (keystate[SDL_SCANCODE_W] || keystate[SDL_SCANCODE_UP])
                    forward += 1.0f;
                if (keystate[SDL_SCANCODE_S] || keystate[SDL_SCANCODE_DOWN])
                    forward -= 1.0f;
                if (keystate[SDL_SCANCODE_A])
                    right -= 1.0f;
                if (keystate[SDL_SCANCODE_D])
                    right += 1.0f;

                if (keystate[SDL_SCANCODE_LEFT])
                    rotation -= 1.0f;
                if (keystate[SDL_SCANCODE_RIGHT])
                    rotation += 1.0f;
            }

            int mouseX, mouseY;
            SDL_GetRelativeMouseState(&mouseX, &mouseY);

            rotation += (float)mouseX * MOUSE_SENS / SCREEN_W;

            player.angle += rotation * P_ROT_SPEED * dt;

            player.velocity.x += (SDL_cosf(player.angle) * forward - SDL_sinf(player.angle) * right) * P_MOVE_SPEED;
            player.velocity.y += (SDL_sinf(player.angle) * forward + SDL_cosf(player.angle) * right) * P_MOVE_SPEED;

            // Velocity clipping (collision):
            {
                Vec2 lastPos = player.origin;
                player.origin.x += player.velocity.x * dt;
                player.origin.y += player.velocity.y * dt;

                int mapX = (int)SDL_floorf(player.origin.x);
                int mapY = (int)SDL_floorf(player.origin.y);

                AABB aabbs[9];

                int numAABBs = 0;
                for (int i = -1; i <= 1; i++)
                {
                    for (int j = -1; j <= 1; j++)
                    {
                        int x = i + mapX;
                        int y = j + mapY;

                        if ((x >= MAP_SIZE || x < 0) || (y >= MAP_SIZE || y < 0))
                        {
                            continue;
                        }

                        if (c_map[x][y] > 0)
                        {
                            aabbs[numAABBs].min.x = x;
                            aabbs[numAABBs].min.y = y;

                            aabbs[numAABBs].max.x = x + 1;
                            aabbs[numAABBs].max.y = y + 1;

                            numAABBs++;
                        }
                    }
                }

                for (int iteration = 0; iteration < 4; iteration++)
                {
                    for (int i = 0; i < numAABBs; i++)
                    {
                        // Find the closest point on AABB

                        Vec2 q;
                        for (int j = 0; j < 2; j++) // LOL this is so bad
                        {
                            float v = player.origin.f[j];
                            if (v < aabbs[i].min.f[j]) v = aabbs[i].min.f[j];
                            if (v > aabbs[i].max.f[j]) v = aabbs[i].max.f[j];
                            q.f[j] = v;
                        }

                        // Find the intersection

                        Vec2 v;
                        v.x = q.x - player.origin.x;
                        v.y = q.y - player.origin.y;

                        float sqr = Vec2_Dot(v, v);
                        if (sqr <= (P_RADIUS * P_RADIUS))
                        {
                            // Clip the velocity

                            Vec2 n;
                            n.x = player.origin.x - q.x;
                            n.y = player.origin.y - q.y;
                            n = Vec2_Normalize(n);

                            float penetration = SDL_fabsf(P_RADIUS - SDL_sqrtf(sqr));
                            player.origin.x += n.x * penetration;
                            player.origin.y += n.y * penetration;
                        }
                    }
                }

                player.velocity.x = (player.origin.x - lastPos.x) / dt;
                player.velocity.y = (player.origin.y - lastPos.y) / dt;
            }

            player.velocity.x -= dt * P_FRICTION * player.velocity.x;
            player.velocity.y -= dt * P_FRICTION * player.velocity.y;
        }

        SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
        SDL_RenderClear(renderer);

        const float c_aspect = (float)SCREEN_W / SCREEN_H;
        const int   c_texWidth = 64;
        const int   c_texHeight = 64;

        {
            Vec2 forward = { 1.0f, 0.0f };
            Vec2 right   = { 0.0f, 1.0f };

            forward = Vec2_Rotate(forward, player.angle);
            right   = Vec2_Rotate(right, player.angle);

            // Draw the floor
            for (int y = (SCREEN_H / 2) + 1; y < SCREEN_H; y++)
            {
                const Vec2 rayDir1 =
                {
                    .x = forward.x - right.x / c_aspect,
                    .y = forward.y - right.y / c_aspect
                };

                const Vec2 rayDir2 =
                {
                    .x = forward.x + right.x / c_aspect,
                    .y = forward.y + right.y / c_aspect
                };

                const float posZ = 0.5f * SCREEN_H;
                const float p = (float)y - posZ;

                const float rowDistance = posZ / p;

                const float floorStepX = rowDistance * (rayDir2.x - rayDir1.x) / (float)SCREEN_W;
                const float floorStepY = rowDistance * (rayDir2.y - rayDir1.y) / (float)SCREEN_W;

                float floorX = player.origin.x + rowDistance * rayDir1.x;
                float floorY = player.origin.y + rowDistance * rayDir1.y;

                const uint32_t* texels1 = grassTex->pixels;
                const uint32_t* texels2 = ceilTex->pixels;

                for (int x = 0; x < SCREEN_W; x++)
                {
                    int texX = (int)(c_texWidth  * ((floorX - SDL_floorf(floorX)))) & (c_texWidth  - 1);
                    int texY = (int)(c_texHeight * ((floorY - SDL_floorf(floorY)))) & (c_texHeight - 1);

                    floorX += floorStepX;
                    floorY += floorStepY;

                    uint32_t texel1 = texels1[texX + texY * c_texHeight];
                    colorBuffer[x + y * SCREEN_W] = texel1;

                    uint32_t texel2 = texels2[texX + texY * c_texHeight];
                    colorBuffer[x + (SCREEN_H - y - 1) * SCREEN_W] = texel2;
                }
            }

            // Draw the walls
            for (int x = 0; x < SCREEN_W; x++)
            {
                const float camX = ((2.0f * x / (float)SCREEN_W) - 1.0f);

                const Vec2 rayDir =
                {
                    .x = forward.x + right.x * camX / c_aspect,
                    .y = forward.y + right.y * camX / c_aspect
                };

                const float deltaDistX = (rayDir.x == 0.0f) ? 0.0f : SDL_fabsf(1.0f / rayDir.x);
                const float deltaDistY = (rayDir.y == 0.0f) ? 0.0f : SDL_fabsf(1.0f / rayDir.y);

                int stepX;
                int stepY;

                bool hit = false;
                int side;

                float sideDistX;
                float sideDistY;

                int mapX = (int)player.origin.x;
                int mapY = (int)player.origin.y;

                if (rayDir.x < 0.0f)
                {
                    stepX = -1;
                    sideDistX = (player.origin.x - mapX) * deltaDistX;
                }
                else
                {
                    stepX = 1;
                    sideDistX = (mapX + 1.0f - player.origin.x) * deltaDistX;
                }

                if (rayDir.y < 0.0f)
                {
                    stepY = -1;
                    sideDistY = (player.origin.y - mapY) * deltaDistY;
                }
                else
                {
                    stepY = 1;
                    sideDistY = (mapY + 1.0f - player.origin.y) * deltaDistY;
                }

                while (!hit)
                {
                    if (sideDistX < sideDistY)
                    {
                        sideDistX += deltaDistX;
                        mapX += stepX;
                        side = 0;
                    }
                    else
                    {
                        sideDistY += deltaDistY;
                        mapY += stepY;
                        side = 1;
                    }

                    if (mapX >= MAP_SIZE || mapX < 0)
                        break;

                    if (mapY >= MAP_SIZE || mapY < 0)
                        break;

                    hit = (c_map[mapX][mapY] > 0);
                }

                if (!hit)
                    continue;

                float wallDist = (side == 0)
                    ? (sideDistX - deltaDistX)
                    : (sideDistY - deltaDistY);

                float lineHeight = 1.0f / wallDist;

                // Draw the slice
                {
                    float wallX = (side == 0)
                        ? (player.origin.y + wallDist * rayDir.y)
                        : (player.origin.x + wallDist * rayDir.x);

                    wallX = wallX - SDL_floorf(wallX);

                    int lineHeightPixels = (int)(lineHeight * SCREEN_H);
                    int y1 = (SCREEN_H - lineHeightPixels) / 2;
                    int y2 = SCREEN_H - y1;

                    int sy1 = SDL_max(y1, 0);
                    int sy2 = SDL_min(y2, SCREEN_H);

                    // Texture mapping:

                    int texX = (int)(wallX * (float)c_texWidth);
                    if (side == 0 && rayDir.x > 0) texX = c_texWidth - texX - 1;
                    if (side == 1 && rayDir.y < 0) texX = c_texWidth - texX - 1;

                    float step = (float)c_texHeight / (y2 - y1);

                    const uint32_t* texels = brickTex->pixels;

                    for (int y = sy1; y < sy2; y++)
                    {
                        int texY = (int)((y - y1) * step);
                        uint32_t texel = texels[texX * c_texHeight + texY];
                        if (side == 1) texel = ((texel >> 1) & 0xFF7F7F7F);
                        colorBuffer[x + y * SCREEN_W] = texel;
                    }

                    depthBuffer[x] = wallDist;
                }
            }
        }

        SDL_UpdateTexture(renderTexture, NULL, colorBuffer,
            sizeof(uint32_t) * SCREEN_W);

        SDL_RenderCopy(renderer, renderTexture, NULL, NULL);
        SDL_RenderPresent(renderer);
    }

    SDL_FreeSurface(brickTex);
    SDL_FreeSurface(ceilTex);
    SDL_FreeSurface(grassTex);

    SDL_free(depthBuffer);
    SDL_free(colorBuffer);

    SDL_DestroyTexture(renderTexture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    SDL_Quit();
    
    return 0;
}