#include <iostream>
#include <vector>
#include <algorithm>
#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }
        Vector2D step = (end - start) / (num_nodes - 1);
        if(start.x > end.x)
        {
            swap(start,end);
            step = -step;
        }
        int cnt = 0;
        for(Vector2D i = start;i.x <= end.x;i+=step)
        {
            masses.push_back(new Mass(i,node_mass,false));
            if(cnt)
                springs.push_back(new Spring(masses[cnt-1],masses[cnt],k));
            cnt++;
        }
        reverse(masses.begin(),masses.end());
        reverse(springs.begin(),springs.end());
        for (auto &i : pinned_nodes)
           masses[i]->pinned = true;
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        // for (auto &s : springs)
        // {
        //     // TODO (Part 2): Use Hooke's law to calculate the force on a node
        //     Vector2D pos1 = s->m1->position;
        //     Vector2D pos2 = s->m2->position;
        //     float len = (pos2-pos1).norm();
        //     s->m1->forces += s->k * (pos2 - pos1)/ len * (len - s->rest_length);
        //     s->m2->forces += -s->k * (pos2 - pos1)/ len * (len - s->rest_length);
        // }

        // for (auto &m : masses)
        // {
        //     if (!m->pinned)
        //     {
        //         // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
        //         m->forces += gravity;
        //         Vector2D a = m->forces / m->mass;
        //         Vector2D temp_v = m->velocity;
        //         m->velocity = m->velocity + a * delta_t;
        //         m->last_position = m->position;
        //         m->position = m->last_position + m->velocity * delta_t;
        //         // TODO (Part 2): Add global damping
        //         m->position *= 0.99995;
        //     }

        //     // Reset all forces on each mass
        //     m->forces = Vector2D(0, 0);
        // }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        const float damping_factor = 0.00005;
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D pos1 = s->m1->position;
            Vector2D pos2 = s->m2->position;
            float len = (pos2-pos1).norm();
            s->m1->forces += s->k * (pos2 - pos1)/ len * (len - s->rest_length);
            s->m2->forces += -s->k * (pos2 - pos1)/ len * (len - s->rest_length);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity;
                Vector2D a = m->forces / m->mass;
                m->velocity = m->velocity + a*delta_t;
                m->position = m->position + (m->position - m->last_position)*(1 - damping_factor) + a * delta_t * delta_t;
                m->last_position = temp_position; 
                // TODO (Part 4): Add global Verlet damping
            }
        }
    }
}
