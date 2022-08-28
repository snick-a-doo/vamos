#include "doctest.h"

#include "test.h"

#include <world/controls.h>

using namespace Vamos_World;
using namespace Vamos_Geometry;

auto constexpr RAW_MOTION_RANGE = 32767;

struct Control_Fixture : public Control_Handler
{
    virtual Control& joystick () { return m_joystick; }
    virtual Control& keyboard () { return m_keyboard; }
    virtual Control& mouse () { return m_mouse; }

    Control m_joystick;
    Control m_keyboard;
    Control m_mouse;
};

struct Keyboard_Fixture : public Control_Fixture
{
    Keyboard_Fixture();

    bool on_down_1(double, double arg)
    {
        down_1_called = true;
        down_1_arg = arg;
        return false;
    }

    bool on_down_2(double, double arg)
    {
        down_2_called = true;
        down_2_arg = arg;
        return true;
    }

    bool on_down_3(double, double)
    {
        down_3_called = true;
        return false;
    }

    bool on_up(double, double arg)
    {
        up_called = true;
        up_arg = arg;
        return false;
    }

    Control c;
    bool down_1_called;
    double down_1_arg;
    bool down_2_called;
    double down_2_arg;
    bool down_3_called;
    double down_3_arg;
    bool up_called;
    double up_arg;
};

Keyboard_Fixture::Keyboard_Fixture()
    : down_1_called(false),
      down_1_arg(0.0),
      down_2_called(false),
      down_2_arg(0.0),
      down_3_called(false),
      down_3_arg(0.0),
      up_called(false),
      up_arg(0.0)
{
    c.bind_action(12, Direct::down, (Control_Handler*)this,
                  (Callback_Function)&Keyboard_Fixture::on_down_1,
                  1.0);
    c.bind_action(12, Direct::none, (Control_Handler*)this,
                  (Callback_Function)&Keyboard_Fixture::on_down_2,
                  2.0);
    c.bind_action(12, Direct::down, (Control_Handler*)this,
                  (Callback_Function)&Keyboard_Fixture::on_down_3,
                  3.0);
    c.bind_action(13, Direct::up, (Control_Handler*)this,
                  (Callback_Function)&Keyboard_Fixture::on_up, 4.0);
}

TEST_CASE("press_12")
{
    Keyboard_Fixture f;
    f.c.press(12);
    CHECK(f.down_1_called);
    CHECK(f.down_2_called);
    CHECK(!f.down_3_called);
    CHECK(!f.up_called);

    CHECK(f.down_1_arg == 1.0);
    CHECK(f.down_2_arg == 2.0);
    CHECK(f.down_3_arg == 0.0);
    CHECK(f.up_arg == 0.0);
}

TEST_CASE("release_13")
{
    Keyboard_Fixture f;
    f.c.press(13);
    CHECK(!f.up_called);
    CHECK(f.up_arg == 0.0);

    f.c.release(13);
    CHECK(f.up_called);
    CHECK(f.up_arg == 4.0);
}

struct Motion_Fixture : public Control_Fixture
{
    Motion_Fixture();

    bool on_move_1_forward(double arg, double)
    {
        move_1_forward_arg = arg;
        return false;
    }

    bool on_move_1_backward(double arg, double)
    {
        move_1_backward_arg = arg;
        return false;
    }

    bool on_move_2(double arg, double)
    {
        move_2_arg = arg;
        return false;
    }

    Control c;
    double move_1_forward_arg;
    double move_1_backward_arg;
    double move_2_arg;
};

Motion_Fixture::Motion_Fixture()
{
    c.bind_motion(4, Direct::forward, (Control_Handler*)this,
                  (Callback_Function)&Motion_Fixture::on_move_1_forward,
                  1.0, 0.0, 0.0, 0.0);
    c.bind_motion(4, Direct::backward, (Control_Handler*)this,
                  (Callback_Function)&Motion_Fixture::on_move_1_backward,
                  -1.0, 0.0, 0.0, 0.0);
    c.bind_motion(7, Direct::none, (Control_Handler*)this,
                  (Callback_Function)&Motion_Fixture::on_move_2,
                  0.5, 0.5, 0.0, 0.0);
}

TEST_CASE("half range")
{
    Motion_Fixture f;
    f.c.move(4, -1000);
    CHECK(close(f.move_1_forward_arg, 1000.0 / RAW_MOTION_RANGE, 1e-3));
    CHECK(f.move_1_backward_arg == 0.0);

    f.c.move(4, 1000);
    CHECK(f.move_1_forward_arg == 0.0);
    CHECK(close(f.move_1_backward_arg, 1000.0 / RAW_MOTION_RANGE, 1e-3));
}

TEST_CASE("full range")
{
    Motion_Fixture f;
    f.c.move(7, 32767);
    CHECK(close(f.move_2_arg, 0.0, 1e-3));
    f.c.move(7, -32767);
    CHECK(close(f.move_2_arg, 1.0, 1e-3));
}

struct Deadband_Fixture : public Control_Fixture
{
    Deadband_Fixture();

    bool on_move(double arg, double)
    {
        move_arg = arg;
        return false;
    }

    Control c;
    double move_arg;
};

Deadband_Fixture::Deadband_Fixture()
{
    c.bind_motion(5, Direct::none, (Control_Handler*)this,
                  (Callback_Function)&Deadband_Fixture::on_move,
                  2.0, 1.0, 0.1, 0.2);
}

TEST_CASE("deadband")
{
    Deadband_Fixture f;

    // lower deadband
    f.c.move(5, 0);
    CHECK(close(f.move_arg, 1.0, 1e-3));
    f.c.move(5, RAW_MOTION_RANGE / 10);
    CHECK(close(f.move_arg, 1.0, 1e-3));
    f.c.move(5, -RAW_MOTION_RANGE / 10);
    CHECK(close(f.move_arg, 1.0, 1e-3));

    // upper deadband
    f.c.move(5, RAW_MOTION_RANGE);
    CHECK(close(f.move_arg, -1.0, 1e-3));
    f.c.move(5, RAW_MOTION_RANGE - RAW_MOTION_RANGE / 5 + 1);
    CHECK(close(f.move_arg, -1.0, 1e-3));
    f.c.move(5, -RAW_MOTION_RANGE + RAW_MOTION_RANGE / 5 - 1);
    CHECK(close(f.move_arg, 3.0, 1e-3));
    f.c.move(5, -RAW_MOTION_RANGE);
    CHECK(close(f.move_arg, 3.0, 1e-3));

    // negative active band
    f.c.move(5, RAW_MOTION_RANGE * 9 / 20);
    CHECK(close(f.move_arg, 0.0, 1e-3));

    // positive active band
    f.c.move(5, -RAW_MOTION_RANGE * 9 / 20);
    CHECK(close(f.move_arg, 2.0, 1e-3));
}
