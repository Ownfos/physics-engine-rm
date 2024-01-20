#include "SFML/Graphics.hpp"
#include "imgui-SFML.h"
#include "Circle.h"
#include "ConvexPolygon.h"
#include "Gizmo.h"
#include "world.h"
#include "UI.h"
#include "ObjectDragger.h"

using namespace physics;

/*
TODO:
- implement spring
- implement dynamic object creation using GUI and mouse clicks
- add description about the math stuff used to derive equation for impulse magnitude...
*/

std::shared_ptr<Rigidbody> CreateObject(std::shared_ptr<ICollider> collider)
{
    auto default_mat = MaterialProperties{
        .restitution = 0.7f,
        .static_friction = 0.6f,
        .dynamic_friction = 0.3f
    };

    // Approximate mass and inertia based on the size.
    // Note: according to parallel-axis theorem, I = Icm + md^2.
    const auto area = collider->Area();
    const auto mass = area;
    const auto inertia = area * area + mass * collider->CenterOfMass().SquaredMagnitude();

    return std::make_shared<Rigidbody>(collider, default_mat, mass, inertia);
}

int main()
{
    sf::RenderWindow window(sf::VideoMode(800, 600), "physics!");
    window.setFramerateLimit(60);
    if (!ImGui::SFML::Init(window)) return -1;

    auto gizmo = Gizmo();
    auto ui = UI();
    auto world = std::make_shared<World>();
    auto dragger = std::make_shared<ObjectDragger>(world);
    ui.SetMouseAction(dragger);

    auto object1 = CreateObject(std::make_shared<Circle>(20.0f));
    object1->Transform().SetPosition({100, 310});
    // object1->SetInertia(0.0f);
    world->AddObject(object1);

    auto object2 = CreateObject(std::make_shared<ConvexPolygon>(std::vector<Vec3>{
        {-20.0f, -20.0f},
        {20.0f, -20.0f},
        {20.0f, 20.0f},
        {-20.0f, 20.0f}
    }));
    object2->Transform().SetPosition({150, 400});
    world->AddObject(object2);

    auto object3 = CreateObject(std::make_shared<ConvexPolygon>(std::vector<Vec3>{
        {-50.0f, -50.0f},
        {50.0f, -50.0f},
        {50.0f, 50.0f}
    }));
    object3->Transform().SetPosition({500, 400});
    world->AddObject(object3);

    auto object4 = CreateObject(std::make_shared<ConvexPolygon>(std::vector<Vec3>{
        {-400.0f, -30.0f},
        {400.0f, -30.0f},
        {400.0f, 30.0f},
        {-400.0f, 30.0f}
    }));
    object4->Transform().SetPosition({400, 500});
    object4->MakeObjectStatic();
    world->AddObject(object4);

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

        // UI and object dragging.
        auto delta_time = deltaClock.restart();
        ImGui::SFML::Update(window, delta_time);
        ui.Update();

        auto time_step = delta_time.asSeconds() * ui.TimeScale();

        // Update
        world->CheckCollisions();
        if (ui.IsUpdateRequired())
        {
            if (ui.IsCollisionEnabled())
            {
                world->ResolveCollisions(time_step);
            }

            if (dragger->IsObjectSelected())
            {
                dragger->ApplyDraggingForce(ui.DragStrength(), time_step);
            }

            if (ui.IsGravityEnabled())
            {
                for (auto& object : world->Objects())
                {
                    if (object->InverseMass() > 0.0f)
                    {
                        object->ApplyImpulse({}, Vec3{0, ui.GravityStrength() / object->InverseMass()}, time_step);
                    }
                }
            }

            world->ConfigureDamping(ui.LinearDamping(), ui.AngularDamping());
            world->Update(time_step);
        }

        // Prepare rendering.
        window.clear(sf::Color::White);

        // Draw all objects.
        for (auto& object : world->Objects())
        {
            // Collider.
            auto& shape = object->Collider()->SFMLShape();
            shape.setFillColor(sf::Color::Transparent);
            shape.setOutlineColor(sf::Color::Black);
            shape.setOutlineThickness(2);
            window.draw(shape);

            // Orientation.
            window.draw(gizmo.Direction(
                object->Transform().Position(),
                object->Transform().GlobalDirection({1, 0})
            ));
        }

        // Draw gizmo for object dragging.
        if (dragger->IsObjectSelected())
        {
            window.draw(gizmo.Point(ui.MousePosition()));

            window.draw(gizmo.Point(dragger->PickedPoint()));
            window.draw(gizmo.Direction(dragger->PickedPoint(), dragger->DragVector(), sf::Color::Blue));
        }

        // Draw contact points for all collisions.
        for (const auto& collision : world->Collisions())
        {
            for (const auto& contact : collision.info.contacts)
            {
                window.draw(gizmo.Point(contact, sf::Color::Red));
                window.draw(gizmo.Direction(contact, collision.info.normal));
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