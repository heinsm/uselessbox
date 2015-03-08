#ifndef statemachine_H_
#define statemachine_H_

#include <stdbool.h>

typedef enum
{
    ss_idle,
    ss_powerup,
    ss_alarming,
    ss_reseting,
    ss_before_shutdown,
    ss_shutdown,
    ss_scare_setup,
    ss_scare_step1,
    ss_scare_step2,
    ss_scare_step3,
    ss_timeout_then_reset,
    ss_reseting_retry,
    ss_offence,
    ss_suspicion_setup,
    ss_suspicion_step1,
    ss_suspicion_step2,
    ss_suspicion_step3,
    ss_slow_finger_setup,
    ss_slow_finger_step1,
    ss_slow_finger_step2,
    ss_END,   //not valid; marks end of enum
} statemachine_states_t;

typedef enum
{
    sa_transition_next,
    sa_arm_reset,
    sa_arm_alarm,
    sa_arm_motion,
    sa_arm_off,
    sa_shutdown,
    sa_shutdown_done,
    sa_timeout,
    sa_scare_timeout,
    sa_scare_exit,
    sa_suspicion_timeout,
    sa_suspicion_exit,
    sa_slowfinger_timeout,
    sa_END,   //not valid; marks end of enum
} statemachine_actions_t;

typedef struct
{
    int             id;
    volatile bool   state_haschanged;
} statemachine_cid;

/*
 * Inits threadman state
 *
 *  pcid        pointer to a state client identifier struct; used in other api calls to identify caller
 *
 * returns EOK; otherwise EERR type of failure
 */
int statemachine_init( statemachine_cid* pcid );

/*
 * finalizes threadman state
 *
 * pcid     pointer to state client identifier struct used during init
 *
 * returns EOK always;
 */
int statemachine_fini( statemachine_cid* pcid );

/*
 * Produces stimuli to the internal statemachine via the specified action
 *
 * thread-safe: yes
 *
 * action       action to apply
 * pnew_state   pointer to receive the new state
 *
 * returns EOK on success; EErr type otherwise describing the failure
 *
 */
int statemachine_next_state(statemachine_actions_t action, statemachine_states_t* pnew_state);

/*
 * Wait for a state change;
 * States only transition from calls to statemachine_next_state
 *
 * thread-safe: yes
 *
 * pcid             pointer to a state client identifier struct;
 * pnew_state       pointer for receiving the new state; if wait is cancelled, receives current state
 *
 * returns EOK on successful wait; EErr type otherwise describing failure
 */
int statemachine_wait_state_change( statemachine_cid* pcid, statemachine_states_t* pnew_state );


/*
 * Cancels all blocked callers to statemachine_wait_state_change
 *
 * thread-safe: yes
 **
 */
int statemachine_cancel_waitfor( statemachine_cid* pcid );

/*
 * Retrieves current state
 *
 * thread-safe: yes
 *
 * returns current state;
 */
statemachine_states_t statemachine_get_current_state();

/*
 * Converts state enum to string literal
 *
 * returns state literal for corresponding state; otherwise "unknown" literal
 */
const char* statemachine_get_statename( statemachine_states_t state );

/*
 * Converts action enum to string literal
 *
 * returns action literal for corresponding action; otherwise "unknown" literal
 */
const char* statemachine_get_actionname( statemachine_actions_t action );


#endif
