//  Copyright (C) 2003-2022 Sam Varner
//
//  This file is part of Caelum.
//
//  Caelum is free software: you can redistribute it and/or modify it under the terms of
//  the GNU General Public License as published by the Free Software Foundation, either
//  version 3 of the License, or (at your option) any later version.
//
//  Caelum is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with Caelum.
//  If not, see <http://www.gnu.org/licenses/>.

#include "sky.h"

#include "../media/texture-image.h"

#include <SDL/SDL.h>

#include <cstdlib>
#include <getopt.h>
#include <iostream>
#include <string>

using Vamos_Media::Texture_Image;

int main(int argc, char* argv[])
{
    // Parse the command line.
    auto divisions{16};
    auto height{0};
    auto width{0};
    Sky::Shape projection{Sky::Shape::sphere};
    auto help{false};
    auto version{false};
    option long_options[] = {{"divisions", required_argument, 0, 'd'},
                             {"height", required_argument, 0, 'h'},
                             {"projection", required_argument, 0, 'p'},
                             {"width", required_argument, 0, 'w'},
                             {"help", no_argument, 0, '?'},
                             {"version", no_argument, 0, 'v'},
                             {0, 0, 0, 0}};
    while (true)
    {
        auto option_index{0};
        auto c{getopt_long(argc, argv, "d:h:p:w:?v", long_options, &option_index)};
        // Detect the end of the options.
        if (c == -1)
            break;

        switch (c)
        {
        case 0:
            break;
        case 'd':
            divisions = atoi(optarg);
            break;
        case 'p':
            if (std::string("cylinder").starts_with(optarg))
                projection = Sky::Shape::cylinder;
            break;
        case 'w':
            width = atoi(optarg);
            break;
        case 'h':
            height = atoi(optarg);
            break;
        case '?':
            help = true;
            break;
        case 'v':
            version = true;
            break;
        default:
            abort();
        }
    }

    if (argc - optind != 1)
        // wrong number arguments
        help = true;

    if (help)
    {
        std::cout << "Usage:\tcaelum [options] image-file\n\n"
                  << "Run Caelum, a tool for creating sky boxes.\n\n"
                  << "Options:\n"
                  << "\t--divisions, -d DIV\tthe number of latitude divisions\n"
                  << "\t--height, -h HEIGHT\tthe height of the window\n"
                  << "\t--help\t\t\tdisplay this help message and exit\n"
                  << "\t--projection, -p SHAPE\tthe image projection: sphere or cylinder\n\n"
                  << "\t--version\t\tdisplay version information and exit\n"
                  << "\t--width, -w WIDTH\tthe width of the window\n\n"
                  << "Reports bugs to snick-a-doo@comcast.net" << std::endl;
        return EXIT_SUCCESS;
    }
    if (version)
    {
        std::cout << "Caelum 1.0 Copyright (C) 2003-2022 Sam Varner\n"
                  << "Caelum comes with ABSOLUTELY NO WARRANTY. You may redistribute\n"
                  << "copies of Caelum under the terms of the GNU General Public License.\n"
                  << "For more information about these matters, see the file named COPYING."
                  << std::endl;
        return EXIT_SUCCESS;
    }

    // Initialize GL.
    if (SDL_Init(SDL_INIT_VIDEO) != 0)
        exit(1);
    atexit(SDL_Quit);

    std::string file{argv[optind]};
    if (width == 0 || height == 0)
    {
        Texture_Image image(file);
        if (width == 0)
            width = image.width_pixels() * 2;
        if (height == 0)
            height = image.height_pixels() * 3 / 2;
    }

    if (SDL_SetVideoMode(width, height, 0, SDL_OPENGL | SDL_RESIZABLE | SDL_DOUBLEBUF) == 0)
        exit(1);

    // Make a window.
    std::string title{"Caelum: " + file};
    SDL_WM_SetCaption(title.c_str(), title.c_str());
    glEnable(GL_TEXTURE_2D);
    glClearColor(0.1, 0.1, 0.1, 0.0);
    SDL_EnableKeyRepeat(500, 30); // Initial delay and interval in ms.

    Sky sky{file, width/4, height/3, divisions, projection};
    sky.display();

    SDL_Event event;
    while (true)
    {
        while (SDL_PollEvent(&event))
        {
            switch (event.type)
            {
            case SDL_KEYDOWN:
                sky.key_press(event.key.keysym.sym);
                break;
            case SDL_QUIT:
                exit(1);
                break;
            }
        }
        SDL_Delay(10);
    }
    return EXIT_SUCCESS;
}
