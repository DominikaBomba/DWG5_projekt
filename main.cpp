#include <cmath>
#include "raylib.h"
#include <vector>
#include <span>
#include "rlgl.h"

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

Vector3 operator +(Vector3 a, float b) {
    return { a.x + b, a.y + b, a.z + b };
}
Vector3 operator -(Vector3 a, Vector3 b) {
    return { a.x- b.x, a.y - b.y, a.z - b.z };
}
float mag2(Vector3 a) {
    return a.x * a.x + a.y * a.y + a.z * a.z;
}
float mag(Vector3 a) {
    return sqrt(mag2(a));
}
float distance(Vector3 a, Vector3 b) {
    return mag(a - b);
}
Vector3 norm(Vector3 a) {
    return a / mag(a);
}
Vector3 norm_safe(Vector3 a) {
    const float len = mag(a);
    return len > 0.0f ? a / len : a;
}
struct Particle {
    Vector3 cur_pos;
    Vector3 prev_pos;
    Vector3 vel;
    Color color = BLUE;
    float mass_inv = 1.0f;
};

// Elastic Distance
struct Constraint_Distance {
    Particle* p1 = nullptr; int16_t i1 = 0;
    Particle* p2 = nullptr; int i2 = 0;
    float rest_length = 0.5f;
};
struct Sphere {
    Vector3 pos;
    float radius;


};

struct Constraint_Bend {
    int16_t i1 = 0;
    int16_t i2 = 0; 
    float min_length = 2.0f;
};

// Long Range Attachment
struct Constraint_LRA {
    int16_t i1 = 0; 
    int16_t i2 = 0; 
    float max_length = 1.0f;
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

        if (p2.mass_inv < epsilon) {
            continue;
        }

        const Vector3 diff = p1.cur_pos - p2.cur_pos;
        const float dist = mag(diff);
        if (dist > constraint.max_length) {
            Vector3 norm = diff / dist;

            const float delta = (constraint.max_length - dist);
            Vector3 dx = norm * delta;

            p2.cur_pos = p2.cur_pos - dx;
        }

    }
}

void solve_collision_sphere(std::vector<Particle>& particles , Sphere& sphere) {
    for (Particle& particle : particles) {
        float dist = (distance(particle.cur_pos, sphere.pos));
        if (dist < sphere.radius + 0.2f ) {
            Vector3 substract_vectors = particle.cur_pos - sphere.pos;
            Vector3 norm = substract_vectors / (dist);

            const float delta = (sphere.radius - dist) + 0.2f;
            const float lambda = delta ;
            Vector3 dx =  norm * lambda ;

            particle.cur_pos = (particle.cur_pos + dx);
           
        }
    }
}

void derive_velocity(std::vector<Particle>& particles, float dt) {
    for (Particle& particle : particles) {
        if (particle.mass_inv == 0.0f) continue;
        particle.vel = (particle.cur_pos - particle.prev_pos) / dt *0.99f;
    }
}

void solve_collision_particles(std::vector<Particle>& particles, float dt) {
    for (int i = 0; i < particles.size(); i++) {
        for (int j = 0; j < particles.size(); j++) {
            if (i == j) continue;
            constexpr float epsilon = 1.0e-8;

            Particle& particle1 = particles[i];
            Particle& particle2 = particles[j];
            Vector3 diff = particle1.cur_pos - particle2.cur_pos;

            const float w1 = particle1.mass_inv;
            const float w2 = particle2.mass_inv;
            float dist = (mag(diff));
            const float w_total = w1 + w2;

            if (w_total < epsilon) {
                continue;
            }

            if (dist <0.6f) {
              
              
                Vector3 norm = diff / dist;

                const float delta = (0.6f  - dist);
                const float lambda = delta / (w_total / dt / dt);
                Vector3 dx = norm * lambda;

                particle1.cur_pos = particle1.cur_pos + dx * w1;
                particle2.cur_pos = particle2.cur_pos - dx * w2;
                particle1.color = GREEN;
                particle2.color = GREEN;

                
            }
            else{
                particle1.color = BLUE;
                particle2.color = BLUE;
            }
        }
    }
}


Cloth create_mesh() {
    Cloth chain;
    int num_of_particles = 5;
    int num_of_columns = 5;
    //dodajemy particle do chaina
    for (int j = 0; j < num_of_columns; j++)
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
    for (int i = 0; i < num_of_columns; i++)
    {
        for (int j = 0;j < num_of_particles - 1; j++) {
            Constraint_Distance c;
            c.p1 = &chain.particles[j + num_of_particles * i];
            c.p2 = &chain.particles[j + num_of_particles * i + 1];
            c.rest_length = 2.0f;
            chain.constraints.push_back(c);
        }
    }
    for (int i = 0; i < num_of_columns - 1; i++)
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
    int num_of_columns = 22;
    //dodajemy particle do chaina
    for (int j = 0; j < num_of_columns; j++)
    {
        for (int i = 0; i < num_of_particles; i++) {
            Particle p1;
            (j == 0)? p1.mass_inv = 0.0f : p1.mass_inv = 1.0f;
            p1.cur_pos = { -10.0f , -(float)i/2.0f+5.0f , (float)j/2.0f - num_of_columns/2  };
            //   p1.prev_pos = { (float)i, (float)i + 3, 0.0f };
            p1.vel = { 0.0f, 0.0f, 0.0f };

            chain.particles.push_back(p1);
        }
    }



    // constrainty pomiedzy kulkami
    for (int i = 0; i < num_of_columns; i++)
    {
        for (int j = 0;j < num_of_particles - 1; j++) {
            Constraint_Distance c;
            c.i1 = j + num_of_particles * i;
            c.i2 = j + num_of_particles * i + 1;
            c.rest_length = 0.5f;
            chain.constraints.push_back(c);
        }
    }
    for (int i = 0; i < num_of_columns - 1; i++)
    {
        for (int j = 0; j < num_of_particles; j++) {
            Constraint_Distance c;
            c.i1 = j + num_of_particles * i;
            c.i2 = j + (num_of_particles) * (i + 1);
            c.rest_length =0.5f;
            chain.constraints.push_back(c);

        }
    }
    //te ukosne

    
    for (int i = 0; i < num_of_columns - 1; i++)
    {
        for (int j = 0; j < num_of_particles-1; j++) {
            Constraint_Distance c;
            c.i1 = j + num_of_particles * i;
            c.i2 = j + (num_of_particles) * (i + 1) + 1;
            c.rest_length = sqrt(1)/2;
          //  c.strong = 0.1;
            chain.constraints.push_back(c);

        }
    }

    //bend

    for (int i = 0; i < num_of_columns - 1; i++)
    {
        for (int j = 0; j < num_of_particles - 1; j++) {
          //  if (chain.particles[i * num_of_particles + j].mass_inv == 0.0f) continue;
            Constraint_Bend c;
            c.i1 = j + num_of_particles * i + 1;
            c.i2 = j + (num_of_particles) * (i + 1);
            Vector3 diff = chain.particles[j + num_of_particles * i + 1].cur_pos - chain.particles[j + (num_of_particles) * (i + 1)].cur_pos;

            c.min_length = mag(diff) / 2;
            chain.bend.push_back(c);
           
            //chain.constraints.push_back(c);

        }
    }
    //LRA

    /*for (int i = 0; i < num_of_columns * num_of_particles; i++)
    {
        if (chain.particles[i].mass_inv == 0.0f){
             continue;
        }
        Constraint_LRA c;
        float min_dist = FLT_MAX;
        for (int z = 0; z < num_of_columns * num_of_particles; z++) {


            if (chain.particles[z].mass_inv == 0.0f &&
                min_dist > distance(chain.particles[z].cur_pos, chain.particles[i].cur_pos)) {

                min_dist = distance(chain.particles[z].cur_pos, chain.particles[i].cur_pos);
                c.i1 = z;
            }
        }
        c.i2 =  i;
        c.max_length = distance(chain.particles[c.i1].cur_pos, chain.particles[c.i2].cur_pos);
        chain.lra.push_back(c);

    }*/

    const int num_particles_total = num_of_columns * num_of_particles;
    for (int i_dynamic = 0; i_dynamic < num_particles_total; ++i_dynamic)
    {
        const Particle& particle_a = chain.particles[i_dynamic];

        // skip fixed particles
        if (particle_a.mass_inv == 0.0f)
            continue;

        // find closest fixed particle
        float min_dist = FLT_MAX;
        int i_closest_fixed = -1;
        for (int i_fixed = 0; i_fixed < num_particles_total; ++i_fixed)
        {
            const Particle& particle_b = chain.particles[i_fixed];

            // skip dynamic particles
            if (particle_b.mass_inv != 0.0f)
                continue;

            const float dist = distance(particle_a.cur_pos, particle_b.cur_pos);
            if (dist < min_dist)
            {
                min_dist = dist;
                i_closest_fixed = i_fixed;
            }
        }

        if (i_closest_fixed == -1)
            break;  // wtf?!

        Constraint_LRA lra;
        lra.i1 = i_closest_fixed;
        lra.i2 = i_dynamic;
        lra.max_length = min_dist;
        
        chain.lra.push_back(lra);
    }

    /*for (int i = 0; i < num_of_columns; i++)
    {
        for (int j = 0; j < num_of_particles; j++) {
              if (chain.particles[i * num_of_particles + j].mass_inv == 0.0f) continue;
            Constraint_LRA c;
            int min_dist = INT16_MAX;
            for (int z = 0; z < num_of_columns; z++) {
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
   */


}

Cloth create_mesh3() {
    Cloth chain;
    int num_of_particles = 15;
    int num_of_columns = 15;
    //dodajemy particle do chaina
    for (int j = 0; j < num_of_columns; j++)
    {
        for (int i = 0; i < num_of_particles; i++) {
            Particle p1;
            (i == 0 && (j == 0 || j == num_of_columns -1) )? p1.mass_inv = 0.0f : p1.mass_inv = 1.0f;
            p1.cur_pos = { -10.0f , -(float)j / 2.0f + 5.0f, (float)i / 2.0f - num_of_particles / 2 };//   p1.prev_pos = { (float)i, (float)i + 3, 0.0f };
            p1.vel = { 0.0f, 0.0f, 0.0f };

            chain.particles.push_back(p1);
        }
    }



    // constrainty pomiedzy kulkami
    for (int i = 0; i < num_of_columns; i++)
    {
        for (int j = 0;j < num_of_particles - 1; j++) {
            Constraint_Distance c;
            c.p1 = &chain.particles[j + num_of_particles * i];
            c.p2 = &chain.particles[j + num_of_particles * i + 1];
            c.rest_length = 2.0f;
            chain.constraints.push_back(c);
        }
    }
    for (int i = 0; i < num_of_columns - 1; i++)
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
    for (int i = 0; i < num_of_columns - 1; i++)
    {
        for (int j = 0; j < num_of_particles - 1; j++) {
            Constraint_Distance c;
            c.p1 = &chain.particles[j + num_of_particles * i];
            c.p2 = &chain.particles[j + (num_of_particles) * (i + 1) + 1];
            c.rest_length = sqrt(8);
            chain.constraints.push_back(c);

        }
    }
    for (int i = 0; i < num_of_columns - 1; i++)
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
/*
// Mesh, vertex data and vao/vbo
typedef struct Mesh {
    int vertexCount;        // Number of vertices stored in arrays
    int triangleCount;      // Number of triangles stored (indexed or not)

    // Vertex attributes data
    float *vertices;        // Vertex position (XYZ - 3 components per vertex) (shader-location = 0)
    float *texcoords;       // Vertex texture coordinates (UV - 2 components per vertex) (shader-location = 1)
    float *texcoords2;      // Vertex texture second coordinates (UV - 2 components per vertex) (shader-location = 5)
    float *normals;         // Vertex normals (XYZ - 3 components per vertex) (shader-location = 2)
    float *tangents;        // Vertex tangents (XYZW - 4 components per vertex) (shader-location = 4)
    unsigned char *colors;  // Vertex colors (RGBA - 4 components per vertex) (shader-location = 3)
    unsigned short *indices; // Vertex indices (in case vertex data comes indexed)

    // Skin data for animation
    int boneCount;          // Number of bones (MAX: 256 bones)
    unsigned char *boneIndices; // Vertex bone indices, up to 4 bones influence by vertex (skinning) (shader-location = 6)
    float *boneWeights;     // Vertex bone weight, up to 4 bones influence by vertex (skinning) (shader-location = 7)

    // Runtime animation vertex data (CPU skinning)
    // NOTE: In case of GPU skinning, not used, pointers are NULL
    float *animVertices;    // Animated vertex positions (after bones transformations)
    float *animNormals;     // Animated normals (after bones transformations)

    // OpenGL identifiers
    unsigned int vaoId;     // OpenGL Vertex Array Object id
    unsigned int *vboId;    // OpenGL Vertex Buffer Objects id (default vertex data)
} Mesh;*/
Mesh create_mesh_render(int particle_num, int num_of_columns) {
    Mesh mesh = { 0 };
    mesh.vertexCount = particle_num * num_of_columns;
    mesh.triangleCount = (particle_num - 1) *(num_of_columns - 1)* 2;

    mesh.vertices = (float*)MemAlloc(mesh.vertexCount * 3 * 4);
    mesh.texcoords = (float*)MemAlloc(mesh.vertexCount * 2 * 4);
    
    mesh.indices = (unsigned short*)MemAlloc(mesh.triangleCount * 3 * 2);


    //tex cords (x,y) 
    for (int i = 0; i < num_of_columns; i++) {
        for (int j = 0; j < particle_num; j++) {
            
            mesh.texcoords[(i * particle_num + j) * 2 + 1] = (float)j / (float)(particle_num - 1);
            mesh.texcoords[(i * particle_num + j) * 2 ] = (float)i / (float)(num_of_columns - 1);

        }
    }
    //triangles
    int index = 0;
    for (int i = 0; i < num_of_columns-1; i++) {
        for (int j = 0; j < particle_num-1; j++) {

            //traingle 1
            mesh.indices[index] = i * particle_num + j;
            index++;
            mesh.indices[index] = i * particle_num + j+1;
            index++;
            mesh.indices[index] = (i+1) * particle_num + j;
            index++;
           
            //triangle 2
            mesh.indices[index] = (i  ) * particle_num + j +1;
            index++;
            mesh.indices[index] = (i + 1) * particle_num + j + 1;
            index++;
            mesh.indices[index] = (i + 1 ) * particle_num + j ;
            index++;


        }
    }
    UploadMesh(&mesh, true);
    return mesh;



}

void update_mesh_render(Mesh& mesh, std::vector<Particle>& particles) {
    for (int k = 0;  k < mesh.vertexCount; k++) {
        mesh.vertices[k * 3 + 0] = particles[k].cur_pos.x;
        mesh.vertices[k * 3 + 1] = particles[k].cur_pos.y;
        mesh.vertices[k * 3 + 2] = particles[k].cur_pos.z;
    }
    
    UpdateMeshBuffer(mesh, 0, mesh.vertices, mesh.vertexCount * 3 * sizeof(float), 0);


    
}
Matrix matrix_identity() {
    Matrix m = { 0 };
    m.m0 = 1.0f; m.m5 = 1.0f; m.m10 = 1.0f; m.m15 = 1.0f;
    return m;
}

//CULLING!
int main(void)
{
    Sphere sphere;
    sphere.pos = { 0.0f, 0.0f, -5.0f };
    sphere.radius = 3.0f;
   
  
    Vector3 sphere_velocity = {10.0f, 0.0f, 1.0f };
   const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "");


    Texture2D texture = LoadTexture("nice.png");
    GenTextureMipmaps(&texture);
    SetTextureFilter(texture, TEXTURE_FILTER_TRILINEAR);


    if (texture.id == 0) TraceLog(LOG_ERROR, "cant load");
    Camera3D camera = { 0 };
    camera.position ={ -30.0f, 2.0f, -10.0f };
    camera.target = { 0.0f,0.0f, 0.0f };    
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



    Mesh mesh = create_mesh_render(15, 22);
    Material clothMat = LoadMaterialDefault();
    SetMaterialTexture(&clothMat, MATERIAL_MAP_DIFFUSE, texture);
    update_mesh_render(mesh, chain.particles);

    
    Vector3 gravity = {0.0f, -20.0f, 0.0f};
    Vector3 wind = {0.0f, 1.0f, 5.0f};
    float sim_time = 0.0f;
    while (!WindowShouldClose())     
    {
        if (IsKeyDown(KEY_DOWN)) {
            camera.position.y -= 0.5f;
            camera.target.y -= 0.5f;
        }
        if (IsKeyDown(KEY_UP)) {
            camera.position.y += 0.5f;
            camera.target.y += 0.5f;
        }
        //UpdateCamera(&camera, CAMERA_FREE);
        float dt = GetFrameTime();
        sim_time += dt;

        // update wind vector
        {
            // wind = random direction * random length, lots of sinuses works nice
        }

        wind = { sinf(sim_time * 2) , sinf(sim_time / 0.5 - 0.3) * 2, 5.0f + sinf(sim_time * 2)  };
        if (dt > 0.0f) {
         
                integrate_linear(chain.particles, gravity, wind, dt);
                solve_distance(chain.particles, chain.constraints, 0.005f, dt);
               solve_lra(chain.particles, chain.lra);
                solve_bend(chain.particles, chain.bend, 0.001f, dt);
                solve_collision_sphere(chain.particles, sphere);
                solve_collision_particles(chain.particles, dt);

                derive_velocity(chain.particles, dt);
            
                update_mesh_render(mesh, chain.particles);
        }
        if (IsKeyDown(KEY_LEFT)) {
            sphere.pos.x -= 0.2f;
        }
        if (IsKeyDown(KEY_RIGHT)) {
            sphere.pos.x += 0.2f;
        }    

        //nie moga z sasiadami sie przecinac


      //  sphere.pos.x +=  sphere_velocity.x * sinf(sim_time/2)*dt;
      

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
       
        ClearBackground(BLACK);

   
        BeginMode3D(camera);
        rlDisableBackfaceCulling();
      
        DrawMesh(mesh, clothMat, matrix_identity());
        rlEnableBackfaceCulling();

        //distance RED
        for (Constraint_Distance& constraint : chain.constraints) {
          // DrawLine3D(chain.particles[constraint.i1].cur_pos, chain.particles[constraint.i2].cur_pos, RED);
        }
        //bend BLUE
        for (Constraint_Bend& constraint : chain.bend) {
           // DrawLine3D(chain.particles[constraint.i1].cur_pos, chain.particles[constraint.i2].cur_pos, BLUE);
        } 
        //lra
        for (Constraint_LRA& constraint : chain.lra) {
          // DrawLine3D(chain.particles[constraint.i1].cur_pos, chain.particles[constraint.i2].cur_pos, BLACK);
        }

        for (Particle& particle : chain.particles) {
          //  DrawSphere(particle.cur_pos, 0.2f, particle.color);
        }

       
        /*
        drawFullCube(cubePosition, 2.0f, 2.0f, 2.0f, RED);
       

        
        DrawSphere(spherePosition, 0.2f, GREEN);
        */
        DrawGrid(100, 1.0f);
        DrawSphere(sphere.pos, sphere.radius, BLUE);
        EndMode3D();
       

        
        DrawRectangle(10, 10, 320, 93, Fade(SKYBLUE, 0.5f));
        DrawRectangleLines(10, 10, 320, 93, BLUE);
      
        DrawText(TextFormat("Dist: %.2f"), 40, 80, 10, DARKGRAY);
      //  DrawTextureEx(texture, { 650, 10 }, 0.0f, 100.0f / texture.width, WHITE);
        EndDrawing();
      
    }

    CloseWindow();       
    return 0;

}