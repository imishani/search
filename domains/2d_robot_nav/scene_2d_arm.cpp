/*
 * Copyright (C) 2023, Itamar Mishani
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * \file   scene_2d_arm.cpp
 * \author Michelle Liu (mmliu@cmu.edu)
 * \date   3/6/24
*/

#include <search/common/scene_interface.hpp>
#include <box2d/box2d.h>
#include <vector>
using namespace std;

class Scene2DArm : public ims::SceneInterface {
public:
    explicit Scene2DArm(vector<float> link_lengths_, const vector<b2PolygonShape> obstacle_polygons_,
                        vector<b2Transform> obstacle_transforms_, float angle_incr_=0.01 * b2_pi) : ims::SceneInterface(){

        link_lengths = link_lengths_;
        angle_incr = angle_incr_;

        num_links = link_lengths.size();
        polygons.resize(num_links);
        transforms.resize(num_links);

        float x_pos = 0.0f;
        for (int i = 0; i < link_lengths.size(); i++) {
            addLink(polygons[i], transforms[i], link_lengths[i], x_pos);
            x_pos += link_lengths[i];
        }

        obstacle_polygons = obstacle_polygons_;
        obstacle_transforms = obstacle_transforms_;
    }

    vector<float> link_lengths;
    vector<b2PolygonShape> polygons;
    vector<b2Transform> transforms;
    vector<b2PolygonShape> obstacle_polygons;
    vector<b2Transform> obstacle_transforms;
    size_t num_links;
    float angle_incr;


    void addLink(b2PolygonShape &m_polygon, b2Transform &m_transform, float link_len, float x_pos) {
        m_polygon.SetAsBox(link_len/2, 0.5f, b2Vec2 (link_len/2, 0.0f), 0.0f);
        m_transform.Set(b2Vec2(x_pos, 0.0f), 0.0f);
    }

    bool isColliding() {
        size_t points = 0;
        for (int i = 0; i < num_links; i++) {
            for (int j = i+2; j < num_links; j++) {  // don't need to check adjacent links
                b2Manifold manifold = createManifold(polygons[i], transforms[i], polygons[j], transforms[j]);
                points += manifold.pointCount;
            }
            for (int j = 0; j < obstacle_polygons.size(); j++) {
                b2Manifold manifold = createManifold(polygons[i], transforms[i],
                                                     obstacle_polygons[j], obstacle_transforms[j]);
                points += manifold.pointCount;
            }
        }
        return points != 0;
    }

    vector<float> getAngles() {

        vector<float> angles(num_links);
        float total_angle = 0.0f;
        for (int i = 0; i < num_links; i++) {
            float th =  transforms[i].q.GetAngle()/b2_pi;
            angles[i] = total_angle - th;
            total_angle -= th;
        }
        return angles;
    }

    bool setAngles(vector<double> angles) {

        transforms[0].Set(transforms[0].p, angles[0]/180.0f*b2_pi);

        for (int i = 0; i < num_links-1; i++) {
            float link_lenA = link_lengths[i];

            float a_angle = transforms[i].q.GetAngle();
            float b_angle = a_angle + angles[i+1]/180.0f*b2_pi;
            float b_x = transforms[i].p.x + link_lenA*cos(a_angle);
            float b_y = transforms[i].p.y + link_lenA*sin(a_angle);
            b2Vec2 b_position = b2Vec2(b_x, b_y);
            transforms[i+1].Set(b_position, b_angle);
        }

        return true;
    }

    bool rotate(int link_idx, int direction) {  // direction: -1 or 1

        float delta = angle_incr*direction;
        float new_angle = transforms[link_idx].q.GetAngle() + delta;
        transforms[link_idx].Set(transforms[link_idx].p, new_angle);

        for (int i {link_idx}; i < num_links-1; i++) {
            float link_lenA = link_lengths[i];

            float a_angle = transforms[i].q.GetAngle();
            float b_angle = transforms[i+1].q.GetAngle() + delta;
            float b_x = transforms[i].p.x + link_lenA*cos(a_angle);
            float b_y = transforms[i].p.y + link_lenA*sin(a_angle);
            b2Vec2 b_position = b2Vec2(b_x, b_y);
            transforms[i+1].Set(b_position, b_angle);
        }

        return true;
    }

    b2Manifold createManifold(b2PolygonShape polygonA, b2Transform transformA,
                              b2PolygonShape polygonB, b2Transform transformB) {
        b2Manifold manifold;
        b2CollidePolygons(&manifold, &polygonA, transformA, &polygonB, transformB);

        b2WorldManifold worldManifold;
        worldManifold.Initialize(&manifold, transformA, polygonA.m_radius, transformB, polygonB.m_radius);

        return manifold;
    }
};

