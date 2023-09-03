#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

const float global_damp = 0.00005f;
namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes):
    masses(num_nodes), springs(num_nodes-1)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D delta = (end - start) / (num_nodes - 1);
        Vector2D cur_pos = start;
        for(int i = 0; i < num_nodes; ++i) {
            masses[i] = new Mass(cur_pos, node_mass, false);
            if(i > 0) {
                springs[i-1] = new Spring(masses[i-1], masses[i], k);
            }
            cur_pos += delta;
        }
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    Rope::~Rope() {
        for(Spring* s : springs) {
            delete s;
        }
        for(Mass* m : masses) {
            delete m;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            auto s_force = s->k * (s->m2->position - s->m1->position) / (s->m2->position - s->m1->position).norm()
                          * ((s->m2->position - s->m1->position).norm() - s->rest_length);
            s->m1->forces += s_force;
            s->m2->forces += -s_force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;
                // TODO (Part 4): Add global Verlet damping
                m->forces -= 100 * global_damp * m->velocity;
                Vector2D a = m->forces / m->mass;
                m->velocity += a * delta_t;
                // 半隐式欧拉法
                m->position += m->velocity * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            auto s_force = s->k * (s->m2->position - s->m1->position) / (s->m2->position - s->m1->position).norm()
                          * ((s->m2->position - s->m1->position).norm() - s->rest_length);
            s->m1->forces += s_force;
            s->m2->forces += -s_force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity * m->mass;
                Vector2D a = m->forces / m->mass;
                Vector2D cur_pos = m->position;
                // TODO (Part 4): Add global Verlet damping
                m->position += (1 - global_damp) * (m->position - m->last_position) + a * delta_t * delta_t;
                m->last_position = cur_pos;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
