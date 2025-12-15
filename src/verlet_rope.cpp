#include "verlet_rope.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/rigid_body2d.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>

void VerletRope2D::_bind_methods() {}
VerletRope2D::VerletRope2D() {}
VerletRope2D::~VerletRope2D() {}

void VerletRope2D::_ready() {
    if (Engine::get_singleton()->is_editor_hint()) {
        return;
    }

    float segmentLen = m_totalLength / m_segmentCount;
    for (int i = 0; i < m_segmentCount; ++i) {
        RopeSegment seg;
        seg.currentPos = get_global_position() + Vector2(0, i * segmentLen);
        seg.oldPos = seg.currentPos;
        seg.isFixed = false;
        m_segments.push_back(seg);
        m_drawPoints.push_back(Vector2());
    }
}

void VerletRope2D::set_attach_points(Node2D* pStart, Node2D* pEnd) {
    m_pStartNode = pStart;
    m_pEndNode = pEnd;
}

void VerletRope2D::_physics_process(double delta) {
    if (Engine::get_singleton()->is_editor_hint()) {
        return;
    }

    simulate_particles(delta);
    
    for (int i = 0; i < m_simulationIterations; ++i) {
        apply_constraints();
    }

    apply_pull_force();

    queue_redraw();
}

void VerletRope2D::simulate_particles(double delta) {
    for (auto& seg : m_segments) {
        if (seg.isFixed) continue;

        Vector2 velocity = seg.currentPos - seg.oldPos;
        seg.oldPos = seg.currentPos;
        
        seg.currentPos += velocity + m_gravity * (delta * delta);
    }
}

void VerletRope2D::apply_constraints() {
    if (m_pStartNode && !m_segments.empty()) {
        m_segments[0].currentPos = m_pStartNode->get_global_position();
        m_segments[0].isFixed = true;
    }
    
    if (m_pEndNode && !m_segments.empty()) {
        m_segments.back().currentPos = m_pEndNode->get_global_position();
        m_segments.back().isFixed = true;
    }

    float targetDist = m_totalLength / (m_segments.size() - 1);

    for (size_t i = 0; i < m_segments.size() - 1; ++i) {
        RopeSegment& node1 = m_segments[i];
        RopeSegment& node2 = m_segments[i + 1];

        Vector2 delta = node2.currentPos - node1.currentPos;
        float dist = delta.length();
        
        if (dist == 0) continue;

        float error = dist - targetDist;
        Vector2 correction = delta.normalized() * error * m_stiffness;

        if (!node1.isFixed && !node2.isFixed) {
            node1.currentPos += correction * 0.5f;
            node2.currentPos -= correction * 0.5f;
        } else if (node1.isFixed && !node2.isFixed) {
            node2.currentPos -= correction;
        } else if (!node1.isFixed && node2.isFixed) {
            node1.currentPos += correction;
        }
    }
}

void VerletRope2D::apply_pull_force() {
    if (!m_pStartNode || !m_pEndNode) return;

    Vector2 pullVector = m_pStartNode->get_global_position() - m_pEndNode->get_global_position();
    float currentDist = pullVector.length();

    if (currentDist > m_totalLength * 1.05f) {
        
        RigidBody2D* pSledBody = Object::cast_to<RigidBody2D>(m_pEndNode);
        
        if (pSledBody) {
            Vector2 forceDir = pullVector.normalized();
            float stretch = currentDist - m_totalLength;
            float forceMag = stretch * 500.0f;

            pSledBody->apply_central_impulse(forceDir * forceMag * get_physics_process_delta_time());
        }
    }
}

void VerletRope2D::_draw() {
    if (m_segments.size() < 2) return;

    Vector2 globalPos = get_global_position();

    PackedVector2Array godotPoints;
    godotPoints.resize(m_segments.size());

    for (size_t i = 0; i < m_segments.size(); ++i) {
        godotPoints[i] = m_segments[i].currentPos - globalPos;
    }

    draw_polyline(godotPoints, m_lineColor, m_lineWidth, true);
}