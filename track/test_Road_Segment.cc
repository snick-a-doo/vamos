#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE Road_Segment
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "Road_Segment.h"
#include "../geometry/Conversions.h"

using namespace Vamos_Geometry;
using namespace Vamos_Track;
using namespace std;

namespace Straight
{
  struct Straight_Fixture
  {
    Straight_Fixture () 
      : segment (100.0, 0.0, 10.0, 10.0, 5.0, 5.0) 
    {
      segment.build_elevation (&elevation, 0.0);
    };
    Road_Segment segment;
    Spline elevation;
  };

  BOOST_AUTO_TEST_CASE (start)
  {
    Straight_Fixture f;
    Three_Vector world_pos (0.0, 0.0, 0.0);
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_SMALL (track_pos.x, 1e-6);
    BOOST_CHECK_SMALL (track_pos.y, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (end)
  {
    Straight_Fixture f;
    Three_Vector world_pos (f.segment.length (), 0.0, 0.0);
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.x, f.segment.length (), 1e-6);
    BOOST_CHECK_SMALL (track_pos.y, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (off_start)
  {
    Straight_Fixture f;
    Three_Vector world_pos (-1.0, 1.0, 0.0);
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_CLOSE (off, -1.0, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.x, -1.0, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, 1.0, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (off_end)
  {
    Straight_Fixture f;
    Three_Vector world_pos (f.segment.length () + 1, 1.0, 0.0);
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_CLOSE (off, 1.0, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.x, f.segment.length () + 1, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, 1.0, 1e-6);
  }
}

namespace Moved_Straight
{
  struct Moved_Straight_Fixture : public Straight::Straight_Fixture
  {
    Moved_Straight_Fixture ()
    {
      segment.set_start (Three_Vector (1.0, 2.0, 3.0),
                         4.0, pi/2.0, -pi/4.0, std::vector <double> ());
    }
  };

  BOOST_AUTO_TEST_CASE (moved_start)
  {
    Moved_Straight_Fixture f;
    Three_Vector world_pos (1.0, 2.0, 3.0);
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_SMALL (track_pos.x, 1e-6);
    BOOST_CHECK_SMALL (track_pos.y, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (moved_end)
  {
    Moved_Straight_Fixture f;
    Three_Vector world_pos (1.0, f.segment.length () + 2.0, 3.0);
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_CLOSE (off, 0.0, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.x, f.segment.length (), 1e-6);
    BOOST_CHECK_SMALL (track_pos.y, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (moved_position)
  {
    Moved_Straight_Fixture f;
    Three_Vector world_pos = f.segment.position (100.0, 100.0);
    BOOST_CHECK_CLOSE (world_pos.x, -99.0, 1e-6);
    BOOST_CHECK_CLOSE (world_pos.y, 102.0, 1e-6);
  }
}

namespace Left_Turn 
{
  struct Left_Turn_Fixture
  {
    Left_Turn_Fixture ()
      : initial_length (pi * 50.0),
        initial_radius (100.0),
        initial_arc (initial_length / initial_radius),
        segment (initial_length, initial_radius, 10.0, 10.0, 5.0, 5.0)
    {
      segment.build_elevation (&elevation, 0.0);
      segment.set_start (Three_Vector (100.0, 2.0, 3.0),
                         111, pi / 2.0, 0.5, std::vector <double> ());
    }
    double initial_length;
    double initial_radius;
    double initial_arc;
    Road_Segment segment;
    Spline elevation;
  };

  BOOST_AUTO_TEST_CASE (no_change)
  {
    Left_Turn_Fixture f;
    BOOST_CHECK_EQUAL (f.segment.length (), f.initial_length);
    BOOST_CHECK_EQUAL (f.segment.radius (), f.initial_radius);
    BOOST_CHECK_CLOSE (f.segment.length () / f.segment.radius (),
                       f.initial_arc, 1e-6);
    BOOST_CHECK_CLOSE (f.segment.left_width (f.initial_length / 4.0), 
                       10.0, 1e-6);
    BOOST_CHECK_CLOSE (f.segment.right_width (f.initial_length / 2.0), 
                       10.0, 1e-6);
    BOOST_CHECK_CLOSE (f.segment.left_road_width (f.initial_length / 4.0), 
                       5.0, 1e-6);
    BOOST_CHECK_CLOSE (f.segment.right_road_width (f.initial_length), 
                       5.0, 1e-6);
    BOOST_CHECK_EQUAL (f.segment.start_coords().x, 100.0);
    BOOST_CHECK_EQUAL (f.segment.start_coords().y, 2.0);
    BOOST_CHECK_SMALL (f.segment.end_coords().x, 1e-6);  
    BOOST_CHECK_CLOSE (f.segment.end_coords().y, 102.0, 1e-4);  
  }

  BOOST_AUTO_TEST_CASE (change_length)
  {
    Left_Turn_Fixture f;
    f.segment.set_length (2.0 * f.initial_length);

    BOOST_CHECK_EQUAL (f.segment.length (), 2.0 * f.initial_length);
    BOOST_CHECK_EQUAL (f.segment.radius (), f.initial_radius);
    BOOST_CHECK_CLOSE (f.segment.length () / f.segment.radius (), 
                       f.initial_arc * 2.0, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (change_arc)
  {
    Left_Turn_Fixture f;
    f.segment.set_arc (f.initial_arc / 2.0);

    BOOST_CHECK_EQUAL (f.segment.length (), f.initial_length / 2);
    BOOST_CHECK_EQUAL (f.segment.radius (), f.initial_radius);
    BOOST_CHECK_CLOSE (f.segment.length () / f.segment.radius (), 
                       f.initial_arc / 2.0, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (scale)
  {
    Left_Turn_Fixture f;
    f.segment.scale (0.5);
    BOOST_CHECK_CLOSE (f.segment.length (), 
                       f.initial_length / 2.0, 1e-6);
    BOOST_CHECK_CLOSE (f.segment.radius (), 
                       f.initial_radius / 2.0, 1e-6);
    BOOST_CHECK_CLOSE (f.segment.arc (), f.initial_arc, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (start)
  {
    Left_Turn_Fixture f;
    Three_Vector world_pos (f.segment.start_coords ());
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_SMALL (track_pos.x, 1e-6);
    BOOST_CHECK_SMALL (track_pos.y, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (start_left)
  {
    Left_Turn_Fixture f;
    Three_Vector world_pos (f.segment.start_coords ());
    world_pos.x -= 10.0;
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_SMALL (track_pos.x, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, 10.0, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (start_right)
  {
    Left_Turn_Fixture f;
    Three_Vector world_pos (f.segment.start_coords ());
    world_pos.x += 10.0;
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_SMALL (track_pos.x, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, -10.0, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (end)
  {
    Left_Turn_Fixture f;
    Three_Vector world_pos (f.segment.end_coords ());
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.x, 50 * pi, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, 0.0, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (end_left)
  {
    Left_Turn_Fixture f;
    Three_Vector world_pos (f.segment.end_coords ());
    world_pos.y -= 10.0;
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_CLOSE (off, 0.0, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.x, 50.0 * pi, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, 10.0, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (end_right)
  {
    Left_Turn_Fixture f;
    Three_Vector world_pos (f.segment.end_coords ());
    world_pos.y += 10.0;
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_CLOSE (off, 0.0, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.x, 50.0 * pi, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, -10.0, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (position)
  {
    Left_Turn_Fixture f;
    Three_Vector world_pos = f.segment.position (pi * 25.0, 50.0);
    const double r = 50.0 / sqrt (2.0);
    BOOST_CHECK_CLOSE (world_pos.x, r, 1e-6);
    BOOST_CHECK_CLOSE (world_pos.y, r + 2.0, 1e-6);
  }
}

namespace Right_Turn
{
  struct Right_Turn_Fixture
  {
    Right_Turn_Fixture ()
      : segment (pi * 50.0, -100.0, 10.0, 10.0, 5.0, 5.0)
    {
      segment.build_elevation (&elevation, 0.0);
      segment.set_start (Three_Vector (100.0, 2.0, 3.0),
                         111, pi / 2.0, 0.5, std::vector <double> ());
    }
    Road_Segment segment;
    Spline elevation;
  };

  BOOST_AUTO_TEST_CASE (start)
  {
    Right_Turn_Fixture f;
    Three_Vector world_pos (f.segment.start_coords ());
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_SMALL (track_pos.x, 1e-6);
    BOOST_CHECK_SMALL (track_pos.y, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (start_left)
  {
    Right_Turn_Fixture f;
    Three_Vector world_pos (f.segment.start_coords ());
    world_pos.x -= 10.0;
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_SMALL (track_pos.x, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, 10.0, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (start_right)
  {
    Right_Turn_Fixture f;
    Three_Vector world_pos (f.segment.start_coords ());
    world_pos.x += 10.0;
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_SMALL (track_pos.x, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, -10.0, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (end)
  {
    Right_Turn_Fixture f;
    Three_Vector world_pos (f.segment.end_coords ());
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.x, 50 * pi, 1e-6);
    BOOST_CHECK_SMALL (track_pos.y, 1e-6);
  }
}

namespace Skewed
{
  struct Skewed_Fixture
  {
    Skewed_Fixture ()
      : segment (100.0, 0.0, 10.0, 10.0, 5.0, 5.0)
    {
      segment.set_start_skew (-1.0);
      segment.set_end_skew (1.0);
      segment.build_elevation (&elevation, 0.0);
    }
    Road_Segment segment;
    Spline elevation;
  };

  BOOST_AUTO_TEST_CASE (start_coordinates)
  {
    Skewed_Fixture f;
    Three_Vector world_pos;
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_SMALL (track_pos.x, 1e-6);
    BOOST_CHECK_SMALL (track_pos.y, 1e-6);
  }

  void start_left ()
  {
    Skewed_Fixture f;
    Three_Vector world_pos (-10.0, 10.0, 0.0);
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_SMALL (track_pos.x, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, 10.0, 1e-6);
  }

  void start_right ()
  {
    Skewed_Fixture f;
    Three_Vector world_pos (10.0, -10.0, 0.0);
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_SMALL (track_pos.x, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, -10.0, 1e-6);
  }

  void end_left ()
  {
    Skewed_Fixture f;
    Three_Vector world_pos (110.0, 10.0, 0.0);
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.x, 100.0, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, 10.0, 1e-6);
  }

  void end_right ()
  {
    Skewed_Fixture f;
    Three_Vector world_pos (90.0, -10.0, 0.0);
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.x, 100.0, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, -10.0, 1e-6);
  }
}

namespace Skewed_Left_Turn
{
  struct Skewed_Left_Turn_Fixture
  {
    Skewed_Left_Turn_Fixture ()
      : segment (pi * 50.0, 100.0, 10.0, 10.0, 5.0, 5.0)
      {
        segment.set_start (Three_Vector (100.0, 2.0, 3.0),
                           111, pi / 2.0, 0.0, std::vector <double> ());
        segment.set_start_skew (-0.5);
        segment.set_end_skew (0.5);
        segment.build_elevation (&elevation, 0.0);
      }
    Road_Segment segment;
    Spline elevation;
  };

  BOOST_AUTO_TEST_CASE (start)
  {
    Skewed_Left_Turn_Fixture f;
    Three_Vector world_pos (f.segment.start_coords ());
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_SMALL (track_pos.x, 1e-6);
    BOOST_CHECK_SMALL (track_pos.y, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (start_left)
  {
    Skewed_Left_Turn_Fixture f;
    Three_Vector world_pos (f.segment.start_coords ());
    world_pos.x -= 10.0;
    world_pos.y -= 5.0;
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_SMALL (track_pos.x, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, 10.0, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (start_right)
  {
    Skewed_Left_Turn_Fixture f;
    Three_Vector world_pos (f.segment.start_coords ());
    world_pos.x += 10.0;
    world_pos.y += 5.0;
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_SMALL (track_pos.x, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, -10.0, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (end)
  {
    Skewed_Left_Turn_Fixture f;
    Three_Vector world_pos (f.segment.end_coords ());
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.x, 50 * pi, 1e-6);
    BOOST_CHECK_SMALL (track_pos.y, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (end_left)
  {
    Skewed_Left_Turn_Fixture f;
    Three_Vector world_pos (f.segment.end_coords ());
    world_pos.x -= 5.0;
    world_pos.y -= 10.0;
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.x, 50.0 * pi, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, 10.0, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (end_right)
  {
    Skewed_Left_Turn_Fixture f;
    Three_Vector world_pos (f.segment.end_coords ());
    world_pos.x += 5.0;
    world_pos.y += 10.0;
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.x, 50.0 * pi, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, -10.0, 1e-6);
  }
}

namespace Skewed_Right_Turn
{
  struct Skewed_Right_Turn_Fixture
  {
    Skewed_Right_Turn_Fixture ()
      : segment (pi * 50.0, -100.0, 10.0, 10.0, 5.0, 5.0)
    {
      segment.set_start (Three_Vector (100.0, 2.0, 3.0),
                         111, pi / 2.0, 0.0, std::vector <double> ());
      segment.set_start_skew (0.5);
      segment.set_end_skew (-0.5);
      segment.build_elevation (&elevation, 0.0);
    }
    Road_Segment segment;
    Spline elevation;
  };

  BOOST_AUTO_TEST_CASE (start)
  {
    Skewed_Right_Turn_Fixture f;
    Three_Vector world_pos (f.segment.start_coords ());
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_SMALL (track_pos.x, 1e-6);
    BOOST_CHECK_SMALL (track_pos.y, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (start_left)
  {
    Skewed_Right_Turn_Fixture f;
    Three_Vector world_pos (f.segment.start_coords ());
    world_pos.x -= 10.0;
    world_pos.y += 5.0;
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_SMALL (track_pos.x, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, 10.0, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (start_right)
  {
    Skewed_Right_Turn_Fixture f;
    Three_Vector world_pos (f.segment.start_coords ());
    world_pos.x += 10.0;
    world_pos.y -= 5.0;
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_SMALL (track_pos.x, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, -10.0, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (end)
  {
    Skewed_Right_Turn_Fixture f;
    Three_Vector world_pos (f.segment.end_coords ());
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.x, 50 * pi, 1e-6);
    BOOST_CHECK_SMALL (track_pos.y, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (end_left)
  {
    Skewed_Right_Turn_Fixture f;
    Three_Vector world_pos (f.segment.end_coords ());
    world_pos.x -= 5.0;
    world_pos.y += 10.0;
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.x, 50.0 * pi, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, 10.0, 1e-6);
  }

  BOOST_AUTO_TEST_CASE (end_right)
  {
    Skewed_Right_Turn_Fixture f;
    Three_Vector world_pos (f.segment.end_coords ());
    world_pos.x += 5.0;
    world_pos.y -= 10.0;
    Three_Vector track_pos;
    double off = f.segment.coordinates (world_pos, track_pos);
    BOOST_CHECK_SMALL (off, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.x, 50.0 * pi, 1e-6);
    BOOST_CHECK_CLOSE (track_pos.y, -10.0, 1e-6);
  }
}
