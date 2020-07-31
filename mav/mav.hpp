enum flight_mode {
    flight_mode_unknown,
    flight_mode_manual,
    flight_mode_search,
    flight_mode_rtl,
    flight_mode_goto,
    flight_mode_hold,
};

class MAV {
private:
    flight_mode mode{flight_mode_unknown};
    bool armed{false};
public:
    MAV() {};
    flight_mode getFlightMode() { return this->mode; };
    bool getArmed() { return this->armed; };
};