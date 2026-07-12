#include <cmath>
#include "raylib.h"
#include<vector>
#if defined(PLATFORM_DESKTOP)
#define GLSL_VERSION            330
#else   // PLATFORM_ANDROID, PLATFORM_WEB
#define GLSL_VERSION            100
#endif
void drawFullCube(Vector3 cubePosition, float height, float width, float length, Color color) {
    DrawCube(cubePosition, height, width, length, color);
    DrawCubeWires(cubePosition, height, width, length, MAROON);
}
Vector3 operator *(Vector3 a, float b) {
    return { a.x * b, a.y * b, a.z * b };
}
Vector3 operator +(Vector3 a, Vector3 b) {
    return { a.x + b.x, a.y + b.y, a.z + b.z };
}
Vector3 operator -(Vector3 a, Vector3 b) {
    return { a.x- b.x, a.y - b.y, a.z - b.z };
}
float distance(Vector3 a, Vector3 b) {
    return sqrt(pow((b.x - a.x), 2) + pow((b.y - a.y), 2) + pow((b.z - a.z), 2));
}
struct Particle {
    Vector3 cur_pos;
    Vector3 prev_pos;
    Vector3 vel;

    float mass_inv = 1.0f;
};
struct Constraint {
    Particle* p1;
    Particle* p2;
    float rest_length = 2.0f;
};
struct Cloth {
    std::vector<Particle> particles;
    std::vector<Constraint> constraints;

};
//czy lepiej jedna funkcja rozwiazujaca caly czain czy pojdeyncze?
void integrate_linear(std::vector<Particle>& particles, std::vector<Constraint> constraints, float dt) {
    Vector3 gravity = { 0.0f, -10.0f, 0.0f };
   
   
 
    for (Particle& particle : particles) {
        if (particle.mass_inv == 0.0f) continue;

        particle.prev_pos = particle.cur_pos;
        particle.vel = particle.vel + gravity * dt;
        particle.cur_pos = particle.cur_pos + particle.vel * dt;

    }
    
}
void keep_on_chain(std::vector<Particle>& particles, std::vector<Constraint> constraints, float dt) {
    
    for (Constraint& constraint : constraints) {

        float sum_mass_inv = constraint.p1->mass_inv + constraint.p2->mass_inv;

        float dist = distance(constraint.p1->cur_pos, constraint.p2->cur_pos);
        Vector3 substract_vectors = constraint.p1->cur_pos - constraint.p2->cur_pos;

        Vector3 norm = { substract_vectors.x / dist, substract_vectors.y / dist, substract_vectors.z / dist };
        float delta = (constraint.rest_length - dist);

        if (constraint.p1->mass_inv != 0.0f) {
            constraint.p1->cur_pos = constraint.p1->cur_pos + norm * (delta * (constraint.p1->mass_inv / sum_mass_inv));
        }
        if (constraint.p2->mass_inv != 0.0f) {
            constraint.p2->cur_pos = constraint.p2->cur_pos - norm * (delta * (constraint.p2->mass_inv / sum_mass_inv));
        }
       

    }
    for (Particle& particle : particles) {
        if (particle.mass_inv == 0.0f) continue;
        particle.vel = (particle.cur_pos - particle.prev_pos) * (1.0f / dt) * 0.99f;
    }
   

}


Cloth create_mesh() {
    Cloth chain;
    int num_of_particles = 5;
    int num_of_rows = 5;
    //dodajemy particle do chaina
    for (int j = 0; j < num_of_rows; j++)
    {
        for (int i = 0; i < num_of_particles; i++) {
            Particle p1;
            (i == 0) ? p1.mass_inv = 0.0f : p1.mass_inv = 1.0f;
            p1.cur_pos = { 1.0f * i , -(float)i, -j * 2.0f + 0.2f * j };
            //   p1.prev_pos = { (float)i, (float)i + 3, 0.0f };
            p1.vel = { 0.0f, 0.0f, 0.0f };

            chain.particles.push_back(p1);
        }
    }



    //dodajemy constrainty pomiedzy kulkami
    for (int i = 0; i < num_of_rows; i++)
    {
        for (int j = 0;j < num_of_particles - 1; j++) {
            Constraint c;
            c.p1 = &chain.particles[j + num_of_particles * i];
            c.p2 = &chain.particles[j + num_of_particles * i + 1];
            c.rest_length = 2.0f;
            chain.constraints.push_back(c);
        }
    }
    for (int i = 0; i < num_of_rows - 1; i++)
    {
        for (int j = 0; j < num_of_particles; j++) {
            Constraint c;
            c.p1 = &chain.particles[j + num_of_particles * i];
            c.p2 = &chain.particles[j + (num_of_particles) * (i + 1)];
            c.rest_length = 2.0f;
            chain.constraints.push_back(c);

        }
    }
    return chain;
}
Cloth create_circle() {
    Cloth chain;
    int num_of_particles = 12;   
    int num_of_rows = 5;      
    float radius = 2.0f;

   
    for (int j = 0; j < num_of_rows; j++)
    {
        for (int i = 0; i < num_of_particles; i++) {
            Particle p1;
            (j == 0) ? p1.mass_inv = 0.0f : p1.mass_inv = 1.0f;   
            float angle = 2.0f * PI * i / num_of_particles;
            p1.cur_pos = { radius * cosf(angle), -(float)j, radius * sinf(angle) };
           
            p1.vel = { 0.0f, 0.0f, 0.0f };
            chain.particles.push_back(p1);
        }
    }
 
    for (int i = 0; i < num_of_rows; i++)
    {
        for (int j = 0; j < num_of_particles; j++) {
            Constraint c;
            c.p1 = &chain.particles[j + num_of_particles * i];
            c.p2 = &chain.particles[(j + 1) % num_of_particles + num_of_particles * i];
            c.rest_length = distance(c.p1->cur_pos, c.p2->cur_pos);
            chain.constraints.push_back(c);
        }
    }
   
    for (int i = 0; i < num_of_rows - 1; i++)
    {
        for (int j = 0; j < num_of_particles; j++) {
            Constraint c;
            c.p1 = &chain.particles[j + num_of_particles * i];
            c.p2 = &chain.particles[j + num_of_particles * (i + 1)];
            c.rest_length = distance(c.p1->cur_pos, c.p2->cur_pos);
            chain.constraints.push_back(c);
        }
    }
    return chain;
}
Cloth create_chain() {
    Cloth chain;
    int num_of_particles = 5;
    //dodajemy particle do chaina
    for (int i = 0; i < num_of_particles; i++) {
        Particle p1;
        (i == 0) ? p1.mass_inv = 0.0f : p1.mass_inv = 1.0f;
        p1.cur_pos = { (float)i , (float)i + 3, (i % 2 == 0) ? 1.0f : 2.0f };
        p1.prev_pos = { (float)i, (float)i + 3, 0.0f };
        p1.vel = { 0.0f, 0.0f, 0.0f };

        chain.particles.push_back(p1);
    }
    //dodajemy constrainty pomiedzy kulkami
    for (int i = 0; i < num_of_particles - 1; i++) {
        Constraint c;
        c.p1 = &chain.particles[i];
        c.p2 = &chain.particles[i + 1];
        c.rest_length = distance(c.p1->cur_pos, c.p2->cur_pos);
        chain.constraints.push_back(c);
    }
    return chain;
}
int main(void)
{
    

   const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "");

    Camera3D camera = { 0 };
    camera.position ={ 10.0f, 2.0f, 10.0f };
    camera.target = { 0.0f, 0.0f, 0.0f };    
    camera.up = { 0.0f, 1.0f, 0.0f };       
    camera.fovy = 45.0f;                             
    camera.projection = CAMERA_PERSPECTIVE;            
    /*
    Vector3 cubePosition = { 0.0f, 0.0f, 0.0f };
    Vector3 spherePosition = { 0.0f, 3.0f, 0.0f };
    */
    UpdateCamera(&camera, CAMERA_FREE);
    
    /*
    Vector3 gravity = { 0.0f, -1.0f, 0.0f };
    Vector3 velocity = {0.0f, 0.0f, 0.0f };
    Vector3 spring = { 0.0f, 1.0f, 0.0f };
    float k = 0.2f;
    Vector3 acceleration = { 0.0f, 1.0f, 0.0f };
    */
    SetTargetFPS(60);           
  
    float delay = 0.99f;
    //dodaæ accumulated_time
    Cloth chain = create_circle();
   

    
    while (!WindowShouldClose())     
    {
        UpdateCamera(&camera, CAMERA_FREE);
        float dt = GetFrameTime();
        if (dt > 0.0f) {
            integrate_linear(chain.particles, chain.constraints, dt);
            keep_on_chain(chain.particles, chain.constraints, dt);
        }

      

        //rysowanie
        /*
        float dist =distance(cubePosition, spherePosition);
        if (cubePosition.y > spherePosition.y) dist = -dist;
        acceleration = gravity  + (spring * dist) ;
       
        velocity = velocity + acceleration*dt;
      
        velocity = velocity * delay;
        cubePosition = cubePosition + velocity ;

        delay *= 0.999f;
        */
        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(camera);
       
        for (Constraint constraint : chain.constraints) {
            DrawLine3D(constraint.p1->cur_pos, constraint.p2->cur_pos, RED);
        }
        for (Particle& particle : chain.particles) {
            DrawSphere(particle.cur_pos, 0.2f, BLUE);
        }

    
        /*
        drawFullCube(cubePosition, 2.0f, 2.0f, 2.0f, RED);
       

        
        DrawSphere(spherePosition, 0.2f, GREEN);
        */
        DrawGrid(10, 1.0f);

        EndMode3D();

        DrawRectangle(10, 10, 320, 93, Fade(SKYBLUE, 0.5f));
        DrawRectangleLines(10, 10, 320, 93, BLUE);
        DrawText(TextFormat("Dist: %.2f"), 40, 80, 10, DARKGRAY);

        EndDrawing();
      
    }

    CloseWindow();       
    return 0;
}