#include "internal.hpp"

auto
mav_sys::findComponent (uint8_t t_compid) -> std::shared_ptr<mav_comp>
{
    for (auto iter : this->components)
    {
        if ((iter)->getCompId () == t_compid)
        {
            return (iter);
        }
    }
    auto new_comp = std::make_shared<mav_comp> (t_compid);
    this->components.push_back (new_comp);
    return new_comp;
}

auto
mav_systems::findSystem (uint8_t t_sysid) -> std::shared_ptr<mav_sys>
{
    for (auto iter : this->systems)
    {
        if ((iter)->getSysId () == t_sysid)
        {
            return (iter);
        }
    }
    auto new_sys = std::make_shared<mav_sys> (t_sysid);
    this->systems.push_back (new_sys);
    return new_sys;
}