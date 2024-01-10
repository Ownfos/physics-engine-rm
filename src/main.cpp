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
- implement circle-polygon collision check
- implement polygon-polygon collision check
- implement friction
- implement damping
- implement spring
- implement object grapping using spring
- add description about the math stuff used to derive equation for impulse magnitude...
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
        .restitution = 1.0f,
        .static_friction = 0.8f,
        .dynamic_friction = 0.5f
    };

    auto shape1 = std::make_shared<Circle>(10.0f);
    auto object1 = std::make_shared<Rigidbody>(shape1, bouncy_mat, 10.0f, 10.0f);
    object1->SetPosition({100, 110});
    world.AddObject(object1);

    auto shape2 = std::make_shared<Circle>(20.0f);
    auto object2 = std::make_shared<Rigidbody>(shape2, bouncy_mat, 0.0f, 0.0f);
    object2->SetPosition({100, 150});
    world.AddObject(object2);

    auto shape3 = std::make_shared<ConvexPolygon>(std::vector<Vec3>{
        {-50.0f, -50.0f},
        {50.0f, -50.0f},
        {50.0f, 50.0f}
    });
    auto object3 = std::make_shared<Rigidbody>(shape3, bouncy_mat, 20.0f, 20.0f);
    object3->SetPosition({100, 150});
    world.AddObject(object3);

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
        static float polygon_rotation = 0.0f;
        if (ImGui::SliderFloat("polygon rotation", &polygon_rotation, 0.0f, 180.0f))
        {
            object3->SetRotation(deg2rad(polygon_rotation));
        }
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left))
        {
            auto [x, y] = ImGui::GetMousePos();
            ImGui::Text("mouse: (%f, %f)", x, y);
            if (auto object = world.PickObject({x, y}))
            {
                ImGui::Text("rad: %f", object->Collider()->BoundaryRadius());
                object->SetPosition({x, y});
            }
        }
        ImGui::End();


        world.CheckCollisions();
        // world.ResolveCollisions(delta_time.asSeconds());
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