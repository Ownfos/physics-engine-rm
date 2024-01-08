#include "SFML/Graphics.hpp"
#include "imgui.h"
#include "imgui-SFML.h"
#include "Circle.h"
#include "ConvexPolygon.h"
#include "Rigidbody.h"
#include "Gizmo.h"

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
    if (!ImGui::SFML::Init(window)) return -1;

    auto object1 = Rigidbody(std::make_shared<Circle>(10.0f), 0, 0);
    auto object2 = Rigidbody(std::make_shared<Circle>(20.0f), 10, 10);
    object2.SetPosition({150, 100});

    auto gizmo = Gizmo();

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
        static float x_offset = 0;
        ImGui::SliderFloat("x offset", &x_offset, -100.0f, 100.0f);
        static float y_offset = 0;
        ImGui::SliderFloat("rotation", &y_offset, -100.0f, 100.0f);
        if (ImGui::Button("reset position"))
        {
            x_offset = 0;
            y_offset = 0;
            object2.SetPosition({150, 100});
        }
        ImGui::End();

        object1.SetPosition({100 + x_offset, 100 + y_offset});

        window.clear(sf::Color::White);

        // draw polygon
        {
            auto& shape = object1.SFMLShape();
            shape.setFillColor(sf::Color::Transparent);
            shape.setOutlineColor(sf::Color::Black);
            shape.setOutlineThickness(1);
            window.draw(shape);
        }

        // draw circle
        {
            auto& shape = object2.SFMLShape();
            shape.setFillColor(sf::Color::Transparent);
            shape.setOutlineColor(sf::Color::Black);
            shape.setOutlineThickness(1);
            window.draw(shape);
        }

        // draw contact point if collision occurred
        if (auto collision = object1.CheckCollision(object2))
        {
            for (const auto& contact : collision->contacts)
            {
                window.draw(gizmo.Point(contact, sf::Color::Red));
                window.draw(gizmo.Direction(contact, collision->normal));

                object2.ApplyImpulse(
                    (contact - object2.Position()),
                    collision->normal,
                    delta_time.asSeconds()
                );
            }
        }
        object2.Update(delta_time.asSeconds());


        ImGui::SFML::Render(window);
        window.display();
    }
    ImGui::SFML::Shutdown();

    return 0;
}