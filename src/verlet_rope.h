#pragma once

#include <godot_cpp/classes/node2d.hpp>
#include <godot_cpp/core/math.hpp>
#include <vector>

using namespace godot;

class VerletRope2D : public Node2D {
    GDCLASS(VerletRope2D, Node2D)

public:
    VerletRope2D();
    ~VerletRope2D();

    void _ready() override;
    void _physics_process(double delta) override;
    void _draw() override;

    void set_attach_points(Node2D* pStart, Node2D* pEnd);
    void simulate_particles(double delta);
    void apply_constraints();
    void apply_pull_force();

private:
    struct RopeSegment {
        Vector2 currentPos;
        Vector2 oldPos;
        bool isFixed;
    };

    std::vector<RopeSegment> m_segments;
    std::vector<Vector2> m_drawPoints;

    Node2D* m_pStartNode = nullptr;
    Node2D* m_pEndNode = nullptr;

    int m_segmentCount = 15;
    float m_totalLength = 100.0f;
    Vector2 m_gravity = Vector2(0.0f, 980.0f);
    int m_simulationIterations = 50;
    float m_stiffness = 0.9f;
    float m_lineWidth = 2.0f;
    Color m_lineColor = Color(0.6f, 0.4f, 0.2f);

protected:
    static void _bind_methods();
};
