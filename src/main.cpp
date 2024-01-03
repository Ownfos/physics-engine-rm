#include "SFML/Graphics.hpp"
#include "imgui.h"
#include "imgui-SFML.h"

int main()
{
    sf::RenderWindow window(sf::VideoMode(800, 600), "physics!");
    if (!ImGui::SFML::Init(window)) return -1;

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
            sf::ConvexShape triangle(3);
            triangle.setFillColor(sf::Color::Black);

            // define the position of the triangle's points
            triangle.setPoint(0, {-50, -50});
            triangle.setPoint(1, {50, -50});
            triangle.setPoint(2, {50, 50});

            triangle.rotate(rotation);
            triangle.move(100 + x_offset, 100);
            window.draw(triangle);
        }

        // draw circle
        {
            sf::CircleShape circle(4);
            circle.setOrigin(4, 4);
            circle.setFillColor(sf::Color::Red);
            circle.rotate(rotation);
            circle.move(100 + x_offset, 100);
            window.draw(circle);
        }


        ImGui::SFML::Render(window);
        window.display();
    }
    ImGui::SFML::Shutdown();

    return 0;
}