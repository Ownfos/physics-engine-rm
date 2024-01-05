#include "SFML/Graphics.hpp"
#include "imgui.h"
#include "imgui-SFML.h"
#include "Circle.h"
#include "ConvexPolygon.h"

using namespace physics;

int main()
{
    sf::RenderWindow window(sf::VideoMode(800, 600), "physics!");
    if (!ImGui::SFML::Init(window)) return -1;

    auto circle = Circle(4);
    auto triangle = ConvexPolygon({
        {-50, -50},
        {50, -50},
        {50, 50}
    });

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

        ImGui::SFML::Update(window, deltaClock.restart());
        ImGui::Begin("test window");
        static float x_offset = 0;
        ImGui::SliderFloat("x offset", &x_offset, 0.0f, 100.0f);
        static float rotation = 0;
        ImGui::SliderFloat("rotation", &rotation, 0.0f, 360.0f);
        ImGui::End();

        window.clear(sf::Color::White);

        // draw polygon
        {
            auto& shape = triangle.SFMLShape();
            shape.setFillColor(sf::Color::Black);
            shape.setRotation(rotation);
            shape.setPosition(100 + x_offset, 100);
            window.draw(shape);
        }

        // draw circle
        {
            auto& shape = circle.SFMLShape();
            shape.setFillColor(sf::Color::Red);
            shape.setRotation(rotation);
            shape.setPosition(100 + x_offset, 100);
            window.draw(shape);
        }


        ImGui::SFML::Render(window);
        window.display();
    }
    ImGui::SFML::Shutdown();

    return 0;
}