#include "internal.hpp"

mav_comp *mav_sys::findComponent(uint8_t t_compid)
{
    for(auto iter : this->components)
    {
        if ((iter)->getCompId() == t_compid)
        {
            return (iter);
        }
    }
    auto new_comp = new mav_comp(t_compid);
    this->components.push_back(new_comp);
    return new_comp;
}

mav_sys *mav_systems::findSystem(uint8_t t_sysid)
{
    for(auto iter : this->systems)
    {
        if ((iter)->getSysId() == t_sysid)
        {
            return (iter);
        }
    }
    auto new_sys = new mav_sys(t_sysid);
    this->systems.push_back(new_sys);
    return new_sys;
}