#include <stdlib.h>
#include <vector>
#include <iostream>
#include <string>

/**
 * 9.81 m/s^2 
 */
double ACCELERATION_DUE_TO_EARTH_GRAVITY = -9.81;

class Vec3 {
    public:
        double x;
        double y;
        double z;

        Vec3(double input_x, double input_y, double input_z) {
            x = input_x;
            y = input_y;
            z = input_z;
        };

        void print(std::string label = "") {
            if (!label.empty()) {
                std::cout << label << ": ";
            }
            std::cout << "(x: " << x << " y: " << y << " z: " << z << ")" << std::endl;
        }
};

class Vec2 {
    public:
        double x;
        double y;

        Vec2(double input_x, double input_y) {
            x = input_x;
            y = input_y;
        };

         void print(std::string label = "") {
            if (!label.empty()) {
                std::cout << label << ": ";
            }
            std::cout << "(x: " << x << " y: " << y << ")" << std::endl;
        }
};

class Particle {
    public:
        double mass;
        Vec3 *position;
        Vec3 *velocity;

        Particle(double input_mass, Vec3 &input_position, Vec3 &input_velocity) {
            mass = input_mass;
            position = new Vec3(input_position.x, input_position.y, input_position.z);
            velocity = new Vec3(input_velocity.x, input_velocity.y, input_velocity.z);
        };

        void print() {
            std::cout << "Position: " << "(" << position->x << ", " << position->y << ", " << position->z << ")" << std::endl;
            std::cout << "Velocity: " << "(" << velocity->x << ", " << velocity->y << ", " << velocity->z << ")" << std::endl;
        };
};

class Simulation {
    public:
        /**
         * Represents a simulation. 
         */
        Simulation(double rate) {
            step_rate_s = rate;
            elapsed_time = 0;
        };

        /**
         * Adds a particle to the sim. 
         */
        void add_particle(Particle p) {
            particles.push_back(p);
        };

        /**
         * Prints all particles in the sim
         */
        void print_particles() {
            for (int i = 0; i < particles.size(); i++) {
                Particle p = particles.at(i);
                p.print();
                std::cout << std::endl;
            }
        }

        /**
         * Computes the step number
         */
        uint step_num() {
            return uint(elapsed_time/step_rate_s) + 1;
        };

        /**
         * Runs a single step of the simulation
         */
        void run_step() {
            for (size_t i = 0; i < particles.size(); i++) {
                Particle p = particles.at(i);
                Vec2 force = compute_force(p);
                // F = ma, a = F/m
                Vec2 accel = Vec2(force.x/p.mass, force.y/p.mass);
                // v = a(dt)
                p.velocity->x += accel.x * step_rate_s;
                p.velocity->y += accel.y * step_rate_s;
                // pos = v(dt)
                p.position->x += p.velocity->x * step_rate_s;
                p.position->y += p.velocity->y * step_rate_s;
            }

            std::cout << "Step " << step_num() << " (current time " << elapsed_time << ")" << ":" << std::endl;
            print_particles();
            std::cout << std::endl;
            elapsed_time += step_rate_s;
        };

    private:
        /**
         * Rate for the simulation to run.
         */
        double step_rate_s;

        /**
         * Current time in seconds, since the beginning of the simulation
         */
        double elapsed_time;

        /**
         * List of particles to simulate. 
         */
        std::vector<Particle> particles;

        /**
         * Computes the force on a particle. 
         */
        Vec2 compute_force(Particle &particle) {
            /**
             * Currently we only support the force of gravity. 
             */
            return Vec2(0, particle.mass * ACCELERATION_DUE_TO_EARTH_GRAVITY);
        };
};

std::vector<Particle> initialize_particles() {
    std::vector<Particle> particles;
    Vec3 pos_one = Vec3(20.0, 15.0, 16.7);
    Vec3 vel_one = Vec3(0, 0, 0);

    particles.push_back(
        Particle(1.0, pos_one, vel_one)
    );
    std::cout << std::endl;
    return particles;
}

int main(int argc, char* argv[]) {
    std::vector<Particle> particles = initialize_particles();
    Simulation sim = Simulation(1.0);
    for (size_t i = 0; i < particles.size(); i++) {
        sim.add_particle(particles.at(i));
    }

    std::cout << "Step 0 (current time 0):" << std::endl;
    sim.print_particles();
    std::cout << std::endl;

    for (int i = 0; i < 2; i++) {
        sim.run_step();
    }
    return 0;
}