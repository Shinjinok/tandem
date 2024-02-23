#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
   /*  AP_Arming *ap_arming = AP_Arming::get_singleton();
    uint32_t current_time = AP_HAL::millis();
    
    const char* health = "";
    if(current_time - ap_arming->can.last_update_time > 1500 ) {
        ap_arming->can.alive = false;
    }
    else{
        ap_arming->can.alive = true;
        if (ap_arming->can.received_health == 0){
            health = "Ok";
        }
        else{
            health = "Bad";
        }
        
    }

   if (ap_arming->can.my_id == 10){
        if(ap_arming->can.alive){
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "This is PFC, get SFC health %s",health);
        }
        else{
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "This is PFC, SFC dead");
        }
   }
   if (ap_arming->can.my_id == 11){
        if(ap_arming->can.alive){
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "This is SFC, get PFC health %s",health);
        }
        else{
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "This is SFC, PFC dead");
        }
   } */

}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
