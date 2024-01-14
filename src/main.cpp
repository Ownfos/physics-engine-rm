#include "SFML/Graphics.hpp"
#include "imgui.h"
#include "imgui-SFML.h"
#include "Circle.h"
#include "ConvexPolygon.h"
#include "Gizmo.h"
#include "World.h"

using namespace physics;

/*
TODO:
1. move poly vs poly to ConvexPolygon.cpp
2. remove Rigidbody::checkcollision
3. change CollisionPair to use CollisionInfo as member variable

- implement friction
- implement damping
- implement spring
- implement object grapping using spring
- add description about the math stuff used to derive equation for impulse magnitude...
*/

std::shared_ptr<Rigidbody> CreateObject(std::shared_ptr<ICollider> collider)
{
    auto default_mat = MaterialProperties{
        .restitution = 1.0f,
        .static_friction = 0.8f,
        .dynamic_friction = 0.5f
    };

    // Approximate mass and inertia based on the size.
    const auto area = collider->Area();
    const auto mass = area;
    const auto inertia = area * area;

    return std::make_shared<Rigidbody>(collider, default_mat, mass, inertia);
}

int main()
{
    sf::RenderWindow window(sf::VideoMode(800, 600), "physics!");
    window.setFramerateLimit(60);
    if (!ImGui::SFML::Init(window)) return -1;

    auto gizmo = Gizmo();
    auto world = World();

    auto object1 = CreateObject(std::make_shared<Circle>(10.0f));
    object1->Transform().SetPosition({100, 110});
    world.AddObject(object1);

    auto object2 = CreateObject(std::make_shared<Circle>(20.0f));
    object2->Transform().SetPosition({150, 150});
    object2->MakeObjectStatic();
    world.AddObject(object2);

    auto object3 = CreateObject(std::make_shared<ConvexPolygon>(std::vector<Vec3>{
        {-50.0f, -50.0f},
        {50.0f, -50.0f},
        {50.0f, 50.0f}
    }));
    object3->Transform().SetPosition({100, 200});
    world.AddObject(object3);

    auto object4 = CreateObject(std::make_shared<ConvexPolygon>(std::vector<Vec3>{
        {-100.0f, -100.0f},
        {100.0f, -100.0f},
        {100.0f, 100.0f},
        {-100.0f, 100.0f}
    }));
    object4->Transform().SetPosition({300, 200});
    world.AddObject(object4);

    sf::Clock deltaClock;
    while (window.isOpen())
    {
        // Handle window events
        sf::Event event;
        while (window.pollEvent(event))
        {
            ImGui::SFML::ProcessEvent(window, event);
            if (event.type == sf::Event::Closed)
                window.close();
        }

        auto delta_time = deltaClock.restart();
        ImGui::SFML::Update(window, delta_time);

        // UI
        ImGui::Begin("test window");
        static std::shared_ptr<Rigidbody> picked_object;
        static Vec3 offset;
        static float drag_force = 2;
        static Vec3 obj_to_mouse;
        ImGui::SliderFloat("drag force", &drag_force, 1, 10);
        auto [x, y] = ImGui::GetMousePos();
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        {
            if (picked_object = world.PickObject({x, y}))
            {
                offset = picked_object->Collider()->Transform().LocalPosition({x, y});
            }
        }
        else if (ImGui::IsMouseDown(ImGuiMouseButton_Left))
        {
            if (picked_object)
            {
                auto rotated_offset = offset;
                rotated_offset.Rotate(picked_object->Transform().Rotation());
                obj_to_mouse = Vec3{x, y} - picked_object->Collider()->Transform().GlobalPosition(offset);
                obj_to_mouse.Normalize();

                // Prevent division by zero if we try to drag a static object.
                if (picked_object->InverseMass() > 0.0f)
                {
                    picked_object->ApplyImpulse(rotated_offset, obj_to_mouse * drag_force / picked_object->InverseMass(), delta_time.asSeconds());
                }
            }
        }
        else if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
        {
            picked_object.reset();
        }
        ImGui::End();

        // Update
        world.CheckCollisions();
        world.ResolveCollisions(delta_time.asSeconds());
        world.Update(delta_time.asSeconds());


        // Render
        window.clear(sf::Color::White);

        // Draw all objects.
        for (auto& object : world.Objects())
        {
            auto& shape = object->Collider()->SFMLShape();
            shape.setFillColor(sf::Color::Transparent);
            shape.setOutlineColor(sf::Color::Black);
            shape.setOutlineThickness(2);
            window.draw(shape);
        }

        // Draw gizmo for object dragging.
        if (picked_object)
        {
            window.draw(gizmo.Point({x, y}));

            const auto picked_point = picked_object->Transform().GlobalPosition(offset);
            window.draw(gizmo.Point(picked_point));
            window.draw(gizmo.Direction(picked_point, obj_to_mouse, sf::Color::Blue));
        }

        // Draw contact points for all collisions.
        for (const auto& collision : world.Collisions())
        {
            for (const auto& contact : collision.contacts)
            {
                window.draw(gizmo.Point(contact, sf::Color::Red));
                window.draw(gizmo.Direction(contact, collision.normal));
            }
        }

        // Draw GUI.
        ImGui::SFML::Render(window);

        // Update screen.
        window.display();
    }
    ImGui::SFML::Shutdown();

    return 0;
}