#include <SDL2/SDL.h>  // For using SDL2
#include <iostream>    // For std::cerr


int main()
{
    // Initialize SDL (video subsystem)
    if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        std::cerr << "SDL Init failed: " << SDL_GetError() << std::endl;
        return -1;
    }

    // Create a window centered on the screen
    SDL_Window* window = SDL_CreateWindow(
        "Robotics Sim GUI",          // Window title
        SDL_WINDOWPOS_CENTERED,      // X position (centered)
        SDL_WINDOWPOS_CENTERED,      // Y position (centered)
        1000,                        // Width
        700,                         // Height
        SDL_WINDOW_SHOWN             // Make window visible
    );

    // Check if window creation failed
    if (!window)
    {
        std::cerr << "Window creation failed: " << SDL_GetError() << std::endl;
        return -1;
    }

    bool running = true;   
    SDL_Event event;  // Event structure for handling input/events

    while (running)
    {
        while (SDL_PollEvent(&event))
        {
            // Check if the user requested to close the window
            if (event.type == SDL_QUIT)
            {
                running = false; 
            }
        }

        // Delay to limit CPU usage (~60 frames per second)
        SDL_Delay(16);
    }

    // Clean up resources before exiting
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0; 
}