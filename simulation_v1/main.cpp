#include <cmath>
#include "raylib.h"
#include <vector>
#include <span>

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
Vector3 operator /(Vector3 a, float b) {
    return { a.x / b, a.y / b, a.z / b };
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

// Elastic Distance
struct Constraint_Distance {
    Particle* p1 = nullptr; int16_t i1 = 0;
    Particle* p2 = nullptr; int i2 = 0;
    float rest_length = 2.0f;
};

struct Constraint_Bend {
    int16_t i1 = 0;
    int16_t i2 = 0; 
    float min_length = 2.0f;
};

// Long Range Attachment
struct Constraint_LRA {
    int16_t i1 = 0; // closest fixed particle
    int16_t i2 = 0; // any dynamic particle
    float max_length = 2.0f;
};

struct Cloth {
    std::vector<Particle> particles;
    std::vector<Constraint_Distance> constraints;
    std::vector<Constraint_Bend> bend;
    std::vector<Constraint_LRA> lra;

};
//czy lepiej jedna funkcja rozwiazujaca caly czain czy pojdeyncze?
void integrate_linear(std::span<Particle> particles, Vector3 gravity, Vector3 wind, float dt) { 
    for (Particle& particle : particles) {
        if (particle.mass_inv == 0.0f) continue;

        particle.prev_pos = particle.cur_pos;
        particle.vel = particle.vel + gravity * dt + wind;
        particle.cur_pos = particle.cur_pos + particle.vel * dt;

    }
    
}
void solve_distance(std::span<Particle> particles, std::span<const Constraint_Distance> constraints, float compliance, float dt) {
    constexpr float epsilon = 1.0e-8;

    for (const Constraint_Distance& constraint : constraints) {
        Particle& p1 = particles[constraint.i1];
        Particle& p2 = particles[constraint.i2];
  

        const float w1 = p1.mass_inv;
        const float w2 = p2.mass_inv;
        const float w_total = w1 + w2;

        if (w_total < epsilon) {
            continue;
        }

        const float dist = distance(p1.cur_pos, p2.cur_pos);
        
        Vector3 substract_vectors = p1.cur_pos - p2.cur_pos;
        Vector3 norm = substract_vectors / dist;
        
        const float delta = (constraint.rest_length - dist);
        const float lambda = delta / (w_total + compliance / dt / dt);
        Vector3 dx = norm * lambda;

        p1.cur_pos = p1.cur_pos + dx * w1;
        p2.cur_pos = p2.cur_pos - dx * w2;
    }
}

void solve_bend(std::vector<Particle>& particles, std::vector<Constraint_Bend> constraints, float compliance, float dt) {
    constexpr float epsilon = 1.0e-8;

    for (Constraint_Bend& constraint : constraints) {
        Particle& p1 = particles[constraint.i1];
        Particle& p2 = particles[constraint.i2];


        const float w1 = p1.mass_inv;
        const float w2 = p2.mass_inv;
        const float w_total = w1 + w2;

        if (w_total < epsilon) {
            continue;
        }

        const float dist = distance(p1.cur_pos, p2.cur_pos);
        if (dist < constraint.min_length) {
            Vector3 substract_vectors = p1.cur_pos - p2.cur_pos;
            Vector3 norm = substract_vectors / dist;

            const float delta = (constraint.min_length - dist);
            const float lambda = delta / (w_total + compliance / dt / dt);
            Vector3 dx = norm * lambda;

            p1.cur_pos = p1.cur_pos + dx * w1;
            p2.cur_pos = p2.cur_pos - dx * w2;
        }
     
    }
}


void solve_lra(std::vector<Particle>& particles, std::vector<Constraint_LRA> constraints) {
    constexpr float epsilon = 1.0e-8;

    for (Constraint_LRA& constraint : constraints) {
        Particle& p1 = particles[constraint.i1];
        Particle& p2 = particles[constraint.i2];


        const float w1 = p1.mass_inv;
        const float w2 = p2.mass_inv;
        const float w_total = w1 + w2;

        if (w_total < epsilon) {
            continue;
        }

        const float dist = distance(p1.cur_pos, p2.cur_pos);
        if (dist > constraint.max_length) {
            Vector3 substract_vectors = p1.cur_pos - p2.cur_pos;
            Vector3 norm = substract_vectors / dist;

            const float delta = (constraint.max_length - dist);
            const float lambda = delta / (w_total );
            Vector3 dx = norm * lambda;

            p1.cur_pos = p1.cur_pos + dx * w1;
            p2.cur_pos = p2.cur_pos - dx * w2;
        }

    }
}

void solve_collision_sphere(std::vector<Particle>& particles /*, spheres */) {

}

void derive_velocity(std::vector<Particle>& particles, float dt) {
    for (Particle& particle : particles) {
        if (particle.mass_inv == 0.0f) continue;
        particle.vel = (particle.cur_pos - particle.prev_pos) / dt;// *0.99f;
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
            p1.cur_pos = { 1.0f * i , -(float)i, (i%2 == 0) ?  - j * 2.0f + 0.2f * j : j * 2.0f + 0.2f * j };
            //   p1.prev_pos = { (float)i, (float)i + 3, 0.0f };
            p1.vel = { 0.0f, 0.0f, 0.0f };

            chain.particles.push_back(p1);
        }
    }



    // constrainty pomiedzy kulkami
    for (int i = 0; i < num_of_rows; i++)
    {
        for (int j = 0;j < num_of_particles - 1; j++) {
            Constraint_Distance c;
            c.p1 = &chain.particles[j + num_of_particles * i];
            c.p2 = &chain.particles[j + num_of_particles * i + 1];
            c.rest_length = 2.0f;
            chain.constraints.push_back(c);
        }
    }
    for (int i = 0; i < num_of_rows - 1; i++)
    {
        for (int j = 0; j < num_of_particles; j++) {
            Constraint_Distance c;
            c.p1 = &chain.particles[j + num_of_particles * i];
            c.p2 = &chain.particles[j + (num_of_particles) * (i + 1)];
            c.rest_length = 2.0f;
            chain.constraints.push_back(c);

        }
    }
    return chain;
}


void create_mesh2(Cloth& chain) {
    int num_of_particles = 15;
    int num_of_rows = 15;
    //dodajemy particle do chaina
    for (int j = 0; j < num_of_rows; j++)
    {
        for (int i = 0; i < num_of_particles; i++) {
            Particle p1;
            (i == 0 && (j == 0 || j == num_of_rows - 1)) ? p1.mass_inv = 0.0f : p1.mass_inv = 1.0f;
            p1.cur_pos = { 0.0f , -(float)i*2.0f, -j * 2.0f };
            //   p1.prev_pos = { (float)i, (float)i + 3, 0.0f };
            p1.vel = { 0.0f, 0.0f, 0.0f };

            chain.particles.push_back(p1);
        }
    }



    // constrainty pomiedzy kulkami
    for (int i = 0; i < num_of_rows; i++)
    {
        for (int j = 0;j < num_of_particles - 1; j++) {
            Constraint_Distance c;
            c.i1 = j + num_of_particles * i;
            c.i2 = j + num_of_particles * i + 1;
            c.rest_length = 2.0f;
            chain.constraints.push_back(c);
        }
    }
    for (int i = 0; i < num_of_rows - 1; i++)
    {
        for (int j = 0; j < num_of_particles; j++) {
            Constraint_Distance c;
            c.i1 = j + num_of_particles * i;
            c.i2 = j + (num_of_particles) * (i + 1);
            c.rest_length = 2.0f;
            chain.constraints.push_back(c);

        }
    }
    //te ukosne
    for (int i = 0; i < num_of_rows - 1; i++)
    {
        for (int j = 0; j < num_of_particles-1; j++) {
            Constraint_Distance c;
            c.i1 = j + num_of_particles * i;
            c.i2 = j + (num_of_particles) * (i + 1) + 1;
            c.rest_length = sqrt(8);
          //  c.strong = 0.1;
            chain.constraints.push_back(c);

        }
    }

    //bend

    for (int i = 0; i < num_of_rows - 1; i++)
    {
        for (int j = 0; j < num_of_particles - 1; j++) {
          //  if (chain.particles[i * num_of_particles + j].mass_inv == 0.0f) continue;
            Constraint_Bend c;
            c.i1 = j + num_of_particles * i+1;
            c.i2 = j + (num_of_particles) * (i + 1);
            chain.bend.push_back(c);
           
            //chain.constraints.push_back(c);

        }
    }
    //LRA

    for (int i = 0; i < num_of_rows ; i++)
    {
        for (int j = 0; j < num_of_particles; j++) {
              if (chain.particles[i * num_of_particles + j].mass_inv == 0.0f) continue;
            Constraint_LRA c;
            int min_dist = INT16_MAX;
            for (int z = 0; z < num_of_rows; z++) {
                for (int s = 0; s < num_of_particles; s++) {
                    if (chain.particles[z * num_of_particles + s].mass_inv == 0.0f &&
                        min_dist > distance(chain.particles[z * num_of_particles + s].cur_pos, chain.particles[i * num_of_particles + j].cur_pos)) {
                        min_dist = distance(chain.particles[z * num_of_particles + s].cur_pos, chain.particles[i * num_of_particles + j].cur_pos);
                        c.i1 = z * num_of_particles + s;
                    }
                }
            }

            c.i2 = j + num_of_particles * i;
            c.max_length = distance(chain.particles[c.i1].cur_pos, chain.particles[c.i2].cur_pos);
            chain.lra.push_back(c);

            //chain.constraints.push_back(c);

        }
    }
   


}

Cloth create_mesh3() {
    Cloth chain;
    int num_of_particles = 15;
    int num_of_rows = 15;
    //dodajemy particle do chaina
    for (int j = 0; j < num_of_rows; j++)
    {
        for (int i = 0; i < num_of_particles; i++) {
            Particle p1;
            (i == 0) ? p1.mass_inv = 0.0f : p1.mass_inv = 1.0f;
            p1.cur_pos = { 1.0f * i , -(float)i, (i % 2 == 0) ? -j * 2.0f + 0.2f * j : j * 2.0f + 0.2f * j };
            //   p1.prev_pos = { (float)i, (float)i + 3, 0.0f };
            p1.vel = { 0.0f, 0.0f, 0.0f };

            chain.particles.push_back(p1);
        }
    }



    // constrainty pomiedzy kulkami
    for (int i = 0; i < num_of_rows; i++)
    {
        for (int j = 0;j < num_of_particles - 1; j++) {
            Constraint_Distance c;
            c.p1 = &chain.particles[j + num_of_particles * i];
            c.p2 = &chain.particles[j + num_of_particles * i + 1];
            c.rest_length = 2.0f;
            chain.constraints.push_back(c);
        }
    }
    for (int i = 0; i < num_of_rows - 1; i++)
    {
        for (int j = 0; j < num_of_particles; j++) {
            Constraint_Distance c;
            c.p1 = &chain.particles[j + num_of_particles * i];
            c.p2 = &chain.particles[j + (num_of_particles) * (i + 1)];
            c.rest_length = 2.0f;
            chain.constraints.push_back(c);

        }
    }
    //te ukosne
    for (int i = 0; i < num_of_rows - 1; i++)
    {
        for (int j = 0; j < num_of_particles - 1; j++) {
            Constraint_Distance c;
            c.p1 = &chain.particles[j + num_of_particles * i];
            c.p2 = &chain.particles[j + (num_of_particles) * (i + 1) + 1];
            c.rest_length = sqrt(8);
            chain.constraints.push_back(c);

        }
    }
    for (int i = 0; i < num_of_rows - 1; i++)
    {
        for (int j = 0; j < num_of_particles - 1; j++) {
            Constraint_Distance c;
            c.p1 = &chain.particles[j + num_of_particles * i + 1];
            c.p2 = &chain.particles[j + (num_of_particles) * (i + 1)];
            c.rest_length = sqrt(8);
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
        Constraint_Distance c;
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
    camera.position ={ 30.0f, -15.0f, 20.0f };
    camera.target = { 0.0f, -15.0f, 0.0f };    
    camera.up = { 0.0f, 1.0f, 0.0f };       
    camera.fovy = 45.0f;                             
    camera.projection = CAMERA_PERSPECTIVE;            
    /*
    Vector3 cubePosition = { 0.0f, 0.0f, 0.0f };
    Vector3 spherePosition = { 0.0f, 3.0f, 0.0f };
    */
   // UpdateCamera(&camera, CAMERA_FREE);
    
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
    Cloth chain;
    create_mesh2(chain);

    
    Vector3 gravity = {0.0f, -20.0f, 0.0f};
    Vector3 wind = {0.1f, 0.1f, 0.0f};

    while (!WindowShouldClose())     
    {
        //UpdateCamera(&camera, CAMERA_FREE);
        float dt = GetFrameTime();

        // update wind vector
        {
            // wind = random direction * random length, lots of sinuses works nice
        }

        if (dt > 0.0f) {
            integrate_linear(chain.particles, gravity, wind, dt);
            solve_distance(chain.particles, chain.constraints, 0.001f, dt);
            solve_bend(chain.particles, chain.bend, 0.001f, dt);
            solve_lra(chain.particles, chain.lra);
            derive_velocity(chain.particles, dt);
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
       
        //distance RED
        for (Constraint_Distance& constraint : chain.constraints) {
            DrawLine3D(chain.particles[constraint.i1].cur_pos, chain.particles[constraint.i2].cur_pos, RED);
        }
        //bend BLUE
        for (Constraint_Bend& constraint : chain.bend) {
            DrawLine3D(chain.particles[constraint.i1].cur_pos, chain.particles[constraint.i2].cur_pos, BLUE);
        } 
        //lra
        for (Constraint_LRA& constraint : chain.lra) {
            DrawLine3D(chain.particles[constraint.i1].cur_pos, chain.particles[constraint.i2].cur_pos, BLACK);
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