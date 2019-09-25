#pragma once

class AutoMission : public Mode {
public:
    virtual void do_command(const AP_Mission::Mission_Command& cmd) = 0;

    virtual bool verify() = 0;
};

class ModeTransition : public AutoMission {

public:
    using Copter::Mode::Mode;

    virtual void do_command(const AP_Mission::Mission_Command& cmd) override;
    virtual bool verify() override { return _state == AIR_HOLD; }
    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; }
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "TRANSITION"; }
    const char *name4() const override { return "TRNS"; }

private:

    enum TransitionState{
        UNDERWATER,
        SURFACE,
        TRANSITION_STATE,
        AIR_SPINUP,
        AIR_CLIMB,
        AIR_HOLD
    };
    TransitionState _state;
    
    float _hover_height;

    void underwater_run(float throttle);
    bool air_init();
    void air_run(float climb_rate);

};

class ModeBuoy : public Mode {

public:
    // inherit constructor
    using Copter::Mode::Mode;

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "BUOY"; }
    const char *name4() const override { return "BUOY"; }

private:
    bool inflatable_deployed = false;
    bool not_climbing_maybe = false;
    uint32_t not_climb_start;

    AP_HAL::UARTDriver *uart;
};

class ModeDelayedBuoy : public Mode {

public:
    // inherit constructor
    using Copter::Mode::Mode;

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "BUOY"; }
    const char *name4() const override { return "BUOY"; }

private:

};

/* NAVIATOR MODE: AUTO for FIG8 */
class ModeNV_AUTO : public AutoMission {

public:
    // inherit constructor
    using Copter::Mode::Mode;

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    virtual void do_command(const AP_Mission::Mission_Command& cmd) override;
    virtual bool verify() override;

    AP_Mission::Mission_Command current_cmd;

    bool is_autopilot() const override { return true; }
    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return false; };
    
    void run_mission();

protected:

    const char *name() const override { return "NV_AUTO"; }
    const char *name4() const override { return "NVAP"; }
    int32_t heading_error(int32_t current, int32_t target, uint8_t dir);

private:

    void process_mission();
    bool cr_fs_check = false;
    uint32_t cr_fs_check_start;
    bool att_fs_check = false;
    uint32_t att_fs_check_start;
};
