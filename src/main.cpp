#include "SFML/Graphics.hpp"
#include "imgui.h"
#include "imgui-SFML.h"
#include "Circle.h"
#include "ConvexPolygon.h"
#include "Gizmo.h"
#include "World.h"

using namespace physics;

/*
rigidbody
- collider
    - shape ::= circle | polygon
- displacement
- velocity
- material
    - inverse mass
    - inverse inertia
    - static friction
    - dynamic friction
    - restitution

collision: (rigidbody, rigidbody) -> collision_info

collision_info
- objects in contact
- list of contact points (global coordinate)
- penetration depth
- contact normal (minimum translation vector)

TODO:
- implement circle-circle collision check
- render contact points
- implement impulse resolution
- test impulse resolution with circle-circle collision
- implement circle-polygon collision check
- implement polygon-polygon collision check
- implement spring
- implement object grapping using spring
*/

int main()
{
    sf::RenderWindow window(sf::VideoMode(800, 600), "physics!");
    window.setFramerateLimit(60);
    if (!ImGui::SFML::Init(window)) return -1;


    auto gizmo = Gizmo();

    // Initialize world.
    auto world = World();

    auto bouncy_mat = MaterialProperties{
        .restitution = 0.9f,
        .static_friction = 0.8f,
        .dynamic_friction = 0.5f
    };

    auto shape1 = std::make_shared<Circle>(10.0f);
    auto object1 = std::make_shared<Rigidbody>(shape1, bouncy_mat, 10, 10);
    object1->SetPosition({100, 110});
    world.AddObject(object1);

    auto shape2 = std::make_shared<Circle>(20.0f);
    auto object2 = std::make_shared<Rigidbody>(shape2, bouncy_mat, 0, 0);
    object2->SetPosition({100, 150});
    world.AddObject(object2);

    // This comment block is kept as an example usage of ConvexPolygon class.
    // auto object2 = Rigidbody(std::make_shared<ConvexPolygon>(std::vector<Vec3>{
    //     {-50.0f, -50.0f},
    //     {50.0f, -50.0f},
    //     {50.0f, 50.0f}
    // }));

    sf::Clock deltaClock;
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            ImGui::SFML::ProcessEvent(window, event);
            if (event.type == sf::Event::Closed)
                window.close();
        }

        auto delta_time = deltaClock.restart();
        ImGui::SFML::Update(window, delta_time);
        ImGui::Begin("test window");
        ImGui::Text("height: %f", object1->Position().y);
        // if (ImGui::Button("push down"))
        {
            // Approximate gravity
            object1->ApplyImpulse({}, {0, 9.8f / object1->InverseMass()}, delta_time.asSeconds());
        }
        ImGui::End();


        world.CheckCollisions();
        world.ResolveCollisions(delta_time.asSeconds());
        world.Update(delta_time.asSeconds());

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