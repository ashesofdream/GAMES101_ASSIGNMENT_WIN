#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        auto rope_vec  = end - start;
        auto rope_unit = rope_vec/(num_nodes-1);
        auto rope_unit_mass = node_mass/static_cast<float>(num_nodes);
       
        masses.push_back(new Mass(start,rope_unit_mass,false));
        cout<<"origin mas position:"<<start<<endl;
        start += rope_unit;
        for(int i = 1; i < num_nodes ; ++i){
            masses.push_back(new Mass(start,rope_unit_mass,false));
            springs.push_back(new Spring(masses[i-1],masses[i],k));
            start += rope_unit;
        }
        //masses.emplace_back(start,rope_unit_mass,false);
        for(const auto& i:pinned_nodes){
            masses[i]->pinned = true;
        }

//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            auto dir_m2_m1 = (s->m1->position - s->m2->position);
            auto m1_m2_len = dir_m2_m1.norm();
            auto force_m2 = s->k*dir_m2_m1/m1_m2_len*(m1_m2_len-s->rest_length);
            s->m2->forces += force_m2;
            s->m1->forces +=  -force_m2;
        }

        for (auto &m : masses)
        {
            float k_d = 0.1;
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                //m->forces -= m->forces*k_d;
                auto a = m->forces/m->mass+gravity;
                m->velocity = m->velocity+a*delta_t;
                //m->position = m->position + m->velocity*delta_t; //显示欧拉
                m->position = m->position + (m->velocity+a*delta_t)*delta_t;//半隐
                // TODO (Part 2): Add global damping
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
            // auto dir_m2_m1 = (s->m1->position - s->m2->position);
            // auto m1_m2_len = dir_m2_m1.norm();
            // auto force_m2 = s->k*dir_m2_m1/m1_m2_len*(m1_m2_len-s->rest_length);
            // s->m2->forces += force_m2;
            // s->m1->forces +=  -force_m2;
            auto vec_m2_m1 = s->m1->position - s->m2->position;
            auto m1_m2_len = vec_m2_m1.norm();
            auto dir_m2_m1 = vec_m2_m1/m1_m2_len;

            if(s->m1->pinned&&s->m2->pinned) continue;
            else if(s->m1->pinned){
                s->m2->position += dir_m2_m1*(m1_m2_len-s->rest_length);
            }else if(s->m2->pinned){
                s->m1->position -= dir_m2_m1*(m1_m2_len-s->rest_length);
            }else{
                auto dis = dir_m2_m1*(m1_m2_len-s->rest_length)/2;
                s->m2->position += dis;
                s->m1->position -= dis;
            }
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                auto a = (m->forces)/m->mass+gravity ;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->position = temp_position +(1 - 0.00005)*(temp_position-m->last_position) + a*delta_t*delta_t;
                m->last_position = temp_position;
                // TODO (Part 4): Add global Verlet damping
                
            }
            m->forces =Vector2D();
        }

   
    }
 
  
}
