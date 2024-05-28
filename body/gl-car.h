//  Copyright (C) 2001-2022 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify it under the terms of
//  the GNU General Public License as published by the Free Software Foundation, either
//  version 3 of the License, or (at your option) any later version.
//
//  Vamos is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with Vamos.
//  If not, see <http://www.gnu.org/licenses/>.

#ifndef VAMOS_BODY_GL_CAR_H_INCLUDED
#define VAMOS_BODY_GL_CAR_H_INCLUDED

#include "car.h"

#include <GL/glu.h>

#include <memory>
#include <string>
#include <vector>

namespace Vamos_Body
{
class Dashboard;
class Rear_View_Mirror;

/// A car that handles graphics, sound and input devices.
class Gl_Car : public Car
{
public:
    /// Make a car at a specific position an orientation.
    Gl_Car(const Vamos_Geometry::Three_Vector& position,
           const Vamos_Geometry::Three_Matrix& orientation);
    virtual ~Gl_Car();

    /// Overridden Car methods
    /// @{
    virtual void read(std::string const& data_dir = "",
                      std::string const& car_file = "") override;
    virtual void set_engine_sound(std::string const& file, double pitch, double volume,
                                  double throttle_volume_factor,
                                  double engine_speed_volume_factor) override;
    virtual void set_exterior_model(Vamos_Media::Model&& model) override;
    virtual void set_interior_model(Vamos_Media::Model&& model) override;
    virtual void set_perspective(double aspect) override;
    virtual void set_view(Vamos_Geometry::Three_Vector const& position, double field_of_view,
                          double near_plane, double far_plane, double pan_angle) override;
    virtual void add_rear_view(Vamos_Geometry::Three_Vector const& position,
                               Vamos_Geometry::Point<double> const&,
                               double direction, double field, double near_plane,
                               double far_plane, std::string const& mask_file) override;
    virtual void set_dashboard(std::unique_ptr<Dashboard> dash) override;
    virtual void draw() override;
    virtual void draw_interior() override;
    virtual Vamos_Geometry::Three_Vector draw_rear_view(double aspect, int index) override;
    virtual void make_rear_view_mask(int window_width, int window_height) override;
    virtual int get_n_mirrors() const override { return m_mirrors.size(); }
    virtual void view(double pan, Vamos_Geometry::Three_Vector const& view_pos) override;
    virtual void view() override;
    virtual void propagate(double time) override;
    virtual void set_paused(bool is_paused) override;
    /// @}

private:
    void draw_dashboard(); ///< Draw the gauges and readouts.
    void draw_dashboard_extras(); ///< Draw detailed information.

    double m_throttle_volume_factor; ///< Sensitivity of engine volume to throttle.
    double m_engine_speed_volume_factor; ///< Sensitivity of engine volume to RPMs.
    std::unique_ptr<Vamos_Media::Sample> mp_engine_sample; ///< The engine sound.
    GLuint m_body_list_id{0}; ///< Exterior model ID
    GLuint m_interior_list_id{0}; ///< Interior model ID
    std::unique_ptr<Dashboard> mp_dashboard; ///< The gauges and steering wheel.
    std::vector<std::unique_ptr<Rear_View_Mirror>> m_mirrors; ///< Rear view.
    double m_near_plane; ///< Nearest visible distance.
    double m_far_plane; ///< Farthest visible distance.
};
} // namespace Vamos_Body

#endif // VAMOS_BODY_GL_CAR_H_INCLUDED
