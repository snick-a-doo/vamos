//  caelum.cc - Main program for making sky boxes from panoramas.
//
//  Copyright (C) 2003 Sam Varner
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#include "Sphere_Sky.h"
#include "Cylinder_Sky.h"

#include "../media/Texture_Image.h"

#include <iostream>
#include <cstdlib>
#include <string>
#include <getopt.h>

#include <SDL/SDL.h>

using Vamos_Media::Texture_Image;

int
main (int argc, char* argv [])
{
  // Parse the command line.
  int help_flag = 0;
  int version_flag = 0;
  int divisions = 16; // default
  int width = 0;
  int height = 0;
  while (true)
	{
	  static struct option long_options [] =
		{
		  // These options set flags.
		  {"help", no_argument, &help_flag, 1},
		  {"version", no_argument, &version_flag, 1},
		  // These options don't set a flag.
		  // We distinguish them by their indices.
		  {"divisions", required_argument, 0, 'd'},
		  {"width", required_argument, 0, 'w'},
		  {"height", required_argument, 0, 'h'},
		  {0, 0, 0, 0}
		};
	  // The option index.
	  int option_index = 0;     
	  int c = getopt_long (argc, argv, "d:w:h:",
						   long_options, &option_index);
	  // Detect the end of the options.
	  if (c == -1)
		break;
	  
	  switch (c)
		{
		case 0:
		  // If this option set a flag, do nothing else now.
		  if (long_options [option_index].flag != 0)
			break;
		  break;
		case 'd':
		  divisions = atoi (optarg);
		  break;
		case 'w':
		  width = atoi (optarg);
		  break;
		case 'h':
		  height = atoi (optarg);
		  break;

		case '?':
		  // `getopt_long' already printed an error message.
		  break;
			
		default:
		  abort ();
		}
	}

  if ((argc - optind) != 1)
	{
	  // wrong number arguments
	  help_flag = 1;
	}
  if (help_flag)
	{
	  std::cout << "Usage:\tcaelum [options] image-file\n\n"
				<< "Run Caelum, a tool for creating sky boxes.\n\n"
				<< "Options:\n"
				<< "\t--divisions, -d DIV\tthe number of latitude divisions\n"
				<< "\t--height, -h HEIGHT\tthe height of the window\n"
				<< "\t--help\t\t\tdisplay this help message and exit\n"
				<< "\t--version\t\tdisplay version information and exit\n"
				<< "\t--width, -w WIDTH\tthe width of the window\n\n"
				<< "Reports bugs to snick-a-doo@comcast.net"
				<< std::endl;
	  return EXIT_SUCCESS;
	}
  if (version_flag)
	{
	  std::cout << "Caelum 1.0\n"
				<< "Copyright (C) 2003 Sam Varner\n"
				<< "Caelum comes with ABSOLUTELY NO WARRANTY.\n"
				<< "You may redistribute copies of Caelum\n"
				<< "under the terms of the GNU General Public License.\n"
				<< "For more information about these matters, "
				<< "see the file named COPYING." 
				<< std::endl;
	  return EXIT_SUCCESS;
	}

  // Initialize GL.
  if (SDL_Init (SDL_INIT_VIDEO) != 0) 
    exit (1);
  atexit (SDL_Quit);
  
  std::string file = argv [optind];
  if ((width == 0) || (height == 0))
	{
	  // Load the image just to get the size.
	  try
		{
		  Texture_Image image (file);
          if (width == 0)
            width = image.width_pixels () * 2;
          if (height == 0)
            height = int (image.height_pixels () * 1.5);
		}
	  catch (Vamos_Media::Missing_Texture_File)
		{
		  std::cerr << "Couldn't find " << file << std::endl;
		  std::exit (EXIT_FAILURE);
		}
	}

  if (SDL_SetVideoMode (width, height, 0, 
                        (SDL_OPENGL | SDL_RESIZABLE | SDL_DOUBLEBUF)) == 0)
    exit (1);

  // Make window.
  std::string title = "Caelum: " + file;
  SDL_GL_SetAttribute (SDL_GL_STENCIL_SIZE, 1);
  SDL_WM_SetCaption (title.c_str (), title.c_str ());

  glEnable (GL_TEXTURE_2D);
  glClearColor (0.5, 0.5, 0.5, 0.0);

  SDL_EnableKeyRepeat (500, 30);
  // Initial delay and interval in ms.

  // Construct the sky object.
  try
	{
	  // Sphere_Sky caelum (divisions, file, width / 4, height / 3);
      Cylinder_Sky caelum (divisions, file, width / 4, height / 3);

      SDL_Event event;
      while (true)
        {
          while (SDL_PollEvent (&event))
            {
              switch (event.type) 
                {
                case SDL_KEYDOWN:
                  caelum.key_press (event.key.keysym.sym);
                  break;
                case SDL_QUIT:
                  exit (1);
                  break;
                }
            }
          SDL_Delay (10);
          caelum.display ();
        }
	}
  catch (Vamos_Media::Missing_Texture_File)
	{
	  std::cerr << "Couldn't find " << file << std::endl;
	  std::exit (EXIT_FAILURE);
	}
  
  return EXIT_SUCCESS;
}
