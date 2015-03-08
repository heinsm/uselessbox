#include "statemachine.h"
#include "util.h"

#include <errno.h>
#include <sys/types.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/queue.h>
#include <assert.h>

#define STDPRINT_NAME                       __FILE__ ":"
#define statemachine_CLIENT_MAXCOUNT        10              //some reasonable size to assert on

typedef struct ss_client_entry
{
    SLIST_ENTRY(ss_client_entry)            entries;
    statemachine_cid*                       pcid;
} ss_client_entry_t;

typedef SLIST_HEAD(, ss_client_entry)       sscids_head_t;

static statemachine_states_t                state = ss_shutdown;
static pthread_mutex_t                      statemachine_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t                       signal_state_haschanged = PTHREAD_COND_INITIALIZER;
static sscids_head_t                        sscids_head;
static int                                  sscid_count = 0;
static int                                  sscid_uniqueid = 0;

static const char*          statemachine_state_names[] =
    {
        "ss_idle",
        "ss_powerup",
        "ss_alarming",
        "ss_reseting",
        "ss_before_shutdown",
        "ss_shutdown",
        "ss_scare_setup",
        "ss_scare_step1",
        "ss_scare_step2",
        "ss_scare_step3",
        "ss_timeout_then_reset",
        "ss_reseting_retry",
        "ss_offence",
        "ss_suspicion_setup",
        "ss_suspicion_step1",
        "ss_suspicion_step2",
        "ss_suspicion_step3",
        "ss_slow_finger_setup",
        "ss_slow_finger_step1",
        "ss_slow_finger_step2",
    };

static const char*          statemachine_action_names[] =
    {
        "sa_transition_next",
        "sa_arm_reset",
        "sa_arm_alarm",
        "sa_arm_motion",
        "sa_arm_off",
        "sa_shutdown",
        "sa_shutdown_done",
        "sa_timeout",
        "sa_scare_timeout",
        "sa_scare_exit",
        "sa_suspicion_timeout",
        "sa_suspicion_exit",
        "sa_slowfinger_timeout",
    };

int statemachine_init( statemachine_cid* pcid )
{
    //do following:
    //lock the machine
    //allocate a new client entry for tracking
    //set the client id
    //inc the client id for next time
    //inc the list counter
    //unlock the machine

    int ret = EAGAIN;   //assume failure

    pthread_mutex_lock( &statemachine_mutex );

    ss_client_entry_t* pcid_entry;

    assert( sscid_count < statemachine_CLIENT_MAXCOUNT );

    if ( sscid_count <= 0 )
    {
        //we init for first use
        state = ss_powerup;
        SLIST_INIT(&sscids_head);
    }

    pcid_entry = malloc( sizeof( ss_client_entry_t ) );

    if ( pcid_entry != NULL )
    {
        pcid_entry->pcid = pcid;

        //init the client entry
        pcid->id = sscid_uniqueid;         //uniquely id this client
        pcid->state_haschanged = true;      //arrange for this client to immediate return on next waitfor

        //track the client
        SLIST_INSERT_HEAD( &sscids_head, pcid_entry, entries );

        sscid_uniqueid++;
        sscid_count++;

        ret = EOK;
    }
    else
    {
        //entry malloc failed
        ret = errno;
    }

    pthread_mutex_unlock( &statemachine_mutex );


    return ret;
}

int statemachine_fini( statemachine_cid* pcid )
{
    //do following:
    //lock the machine
    //decr list counter
    //find matching entry and remove it (dellocate the client tracking entry)
    //unlock the machine

    int ret = EAGAIN;   //assume failure;
    bool found_client = false;

    pthread_mutex_lock( &statemachine_mutex );

    //find matching entry and remove it
    ss_client_entry_t* ssce;
    ss_client_entry_t* ssce_next;

    SLIST_FOREACH_SAFE(ssce, &sscids_head, entries, ssce_next)
    {
        if ( ssce->pcid->id == pcid->id )
        {
            SLIST_REMOVE( &sscids_head, ssce, ss_client_entry, entries);
            free(ssce);
            found_client = true;
            sscid_count--;
            break;
        }
    }

    //did we remove successfully?
    if ( found_client )
    {
        ret = EOK;
    }

    //is statemachine now terminated?
    //we check this after removal to ensure the client cids were actually valid and removed
    if ( sscid_count <= 0 )
    {
        //has no effect and no one is listening,
        //but this makes things tidy
        state = ss_shutdown;
    }

    pthread_mutex_unlock( &statemachine_mutex );

    return ret;
}

/*
 * Internal function to retrieve current statemachine state
 * note: no statemachien lock is acquired
 */
static inline statemachine_states_t statemachine_get_current_state_nolock()
{
    return state;
}

/*
 * Internal function to handle setting
 * specific client for statechange and notify
 *
 * pcid     pointer to client id struct for notification
 *
 */
static int statemachine_set_state_change_forclient_nolock( statemachine_cid* pcid )
{
    //set specific clients state change and notify
    pcid->state_haschanged = true;
    return pthread_cond_broadcast( &signal_state_haschanged );
}

/*
 * Internal function to handle setting
 * all clients for statechange and notify
 *
 * Note: Callers should hold statemachine_mutex lock
 *
 */
static int statemachine_set_state_change_nolock( statemachine_states_t new_state )
{
    //set all clients state change and notify
    ss_client_entry_t* ssce;
    SLIST_FOREACH( ssce, &sscids_head, entries )
    {
        ssce->pcid->state_haschanged = true;
    }

    state = new_state;
    return pthread_cond_broadcast( &signal_state_haschanged );
}

int statemachine_next_state(statemachine_actions_t action, statemachine_states_t* pnew_state)
{
    //do following:
    //lock the machine
    //stimulate statemachine with action
    //notify all clients of state change
    //unlock the machine

    int ret = EAGAIN;       //assume failure
    pthread_mutex_lock( &statemachine_mutex );

    statemachine_states_t current_state = statemachine_get_current_state_nolock();
    statemachine_states_t next_state = current_state;     //assume no state change

    switch ( current_state )
    {
        case ss_idle:
        {
            switch ( action )
            {
                case sa_transition_next:
                case sa_arm_off:
                case sa_timeout:
                    next_state = ss_idle;
                    break;
                case sa_arm_alarm:
                case sa_arm_motion:
                    next_state = ss_alarming;
                    break;
                case sa_arm_reset:
                    next_state = ss_reseting;
                    break;
                case sa_shutdown:
                case sa_shutdown_done:
                    next_state = ss_before_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }

            break;
        }

        case ss_powerup:
        {
            switch ( action )
            {
                case sa_transition_next:
                case sa_timeout:
                    next_state = ss_powerup;
                    break;
                case sa_arm_off:
                    next_state = ss_idle;
                    break;
                case sa_arm_alarm:
                case sa_arm_motion:
                    next_state = ss_alarming;
                    break;
                case sa_arm_reset:
                    next_state = ss_reseting;
                    break;
                case sa_shutdown:
                case sa_shutdown_done:
                    next_state = ss_before_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }

            break;
        }

        case ss_alarming:
        {
            switch ( action )
            {
                case sa_transition_next:
                case sa_arm_alarm:
                case sa_arm_motion:
                case sa_timeout:
                    next_state = ss_alarming;
                    break;
                case sa_arm_off:
                    next_state = ss_idle;
                    break;
                case sa_arm_reset:
                    next_state = ss_reseting;
                    break;
                case sa_shutdown:
                case sa_shutdown_done:
                    next_state = ss_before_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }

            break;
        }

        case ss_reseting:
        {
            switch ( action )
            {
                case sa_transition_next:
                case sa_arm_reset:
                case sa_timeout:
                    next_state = ss_reseting;
                    break;
                case sa_arm_off:
                    next_state = ss_idle;
                    break;
                case sa_arm_alarm:
                    next_state = ss_alarming;
                    break;
                case sa_arm_motion:
                    next_state = ss_scare_setup;
                    break;
                case sa_shutdown:
                case sa_shutdown_done:
                    next_state = ss_before_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }

            break;
        }

        case ss_scare_setup:
        {
            switch ( action )
            {
                case sa_scare_exit:
                    next_state = ss_reseting_retry;
                    break;
                case sa_transition_next:
                case sa_arm_reset:
                case sa_arm_motion:
                    next_state = ss_scare_setup;
                    break;
                case sa_arm_alarm:
                case sa_arm_off:
                    next_state = ss_scare_step1;
                    break;
                case sa_shutdown:
                case sa_shutdown_done:
                    next_state = ss_before_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }

            break;
        }

        case ss_scare_step1:
        {
            switch ( action )
            {
                case sa_scare_exit:
                    next_state = ss_scare_step3;
                    break;
                case sa_scare_timeout:
                case sa_arm_reset:
                case sa_arm_off:
                    next_state = ss_scare_step2;
                    break;
                case sa_transition_next:
                case sa_arm_alarm:
                case sa_arm_motion:
                    next_state = ss_scare_step1;
                    break;
                case sa_shutdown:
                case sa_shutdown_done:
                    next_state = ss_before_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }

            break;
        }

        case ss_scare_step2:
        {
            switch ( action )
            {
                case sa_scare_exit:
                    next_state = ss_scare_step3;
                    break;
                case sa_arm_motion:
                    next_state = ss_scare_step2;
                    break;
                case sa_scare_timeout:
                case sa_arm_alarm:
                    next_state = ss_scare_step1;
                    break;
                case sa_transition_next:
                case sa_arm_reset:
                case sa_arm_off:
                    next_state = ss_offence;
                    break;
                case sa_shutdown:
                case sa_shutdown_done:
                    next_state = ss_before_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }

            break;
        }

        case ss_scare_step3:
        {
            switch ( action )
            {
                case sa_scare_exit:
                case sa_transition_next:
                case sa_scare_timeout:
                case sa_arm_alarm:
                case sa_arm_motion:
                    next_state = ss_scare_step3;
                    break;
                case sa_arm_reset:
                    next_state = ss_reseting_retry;
                    break;
                case sa_arm_off:
                    next_state = ss_suspicion_setup;
                    break;
                case sa_shutdown:
                case sa_shutdown_done:
                    next_state = ss_before_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }

            break;
        }

        case ss_timeout_then_reset:
        {
            switch ( action )
            {
                case sa_transition_next:
                case sa_arm_reset:
                case sa_arm_off:
                    next_state = ss_timeout_then_reset;
                    break;
                case sa_timeout:
                    next_state = ss_reseting;
                    break;
                case sa_arm_alarm:
                case sa_arm_motion:
                    next_state = ss_scare_setup;
                    break;
                case sa_shutdown:
                case sa_shutdown_done:
                    next_state = ss_before_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }

            break;
        }

        case ss_reseting_retry:
        {
            switch ( action )
            {
                case sa_transition_next:
                case sa_arm_reset:
                case sa_timeout:
                case sa_arm_motion:
                    next_state = ss_reseting_retry;
                    break;
                case sa_arm_off:
                    next_state = ss_suspicion_setup;
                    break;
                case sa_arm_alarm:
                    next_state = ss_offence;
                    break;
                case sa_shutdown:
                case sa_shutdown_done:
                    next_state = ss_before_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }
            break;
        }

        case ss_offence:
        {
            switch ( action )
            {
                case sa_transition_next:
                case sa_arm_alarm:
                case sa_arm_motion:
                case sa_arm_off:
                case sa_timeout:
                    next_state = ss_offence;
                    break;
                case sa_arm_reset:
                    next_state = ss_timeout_then_reset;
                    break;
                case sa_shutdown:
                case sa_shutdown_done:
                    next_state = ss_before_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }
            break;
        }

        case ss_suspicion_setup:
        {
            switch ( action )
            {
                case sa_transition_next:
                case sa_arm_off:
                case sa_arm_reset:
                    next_state = ss_suspicion_setup;
                    break;
                case sa_suspicion_timeout:
                    next_state = ss_suspicion_step1;
                    break;
                case sa_suspicion_exit:
                    next_state = ss_slow_finger_step2;
                    break;
                case sa_arm_alarm:
                case sa_arm_motion:
                    next_state = ss_slow_finger_setup;
                    break;
                case sa_shutdown:
                case sa_shutdown_done:
                    next_state = ss_before_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }
            break;
        }

        case ss_suspicion_step1:
        {
            switch ( action )
            {
                case sa_transition_next:
                case sa_arm_off:
                case sa_arm_reset:
                    next_state = ss_suspicion_step1;
                    break;
                case sa_suspicion_timeout:
                    next_state = ss_suspicion_step2;
                    break;
                case sa_suspicion_exit:
                    next_state = ss_slow_finger_step2;
                    break;
                case sa_arm_alarm:
                case sa_arm_motion:
                    next_state = ss_slow_finger_setup;
                    break;
                case sa_shutdown:
                case sa_shutdown_done:
                    next_state = ss_before_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }
            break;
        }

        case ss_suspicion_step2:
        {
            switch ( action )
            {
                case sa_transition_next:
                case sa_arm_off:
                case sa_arm_reset:
                    next_state = ss_suspicion_step2;
                    break;
                case sa_suspicion_timeout:
                    next_state = ss_suspicion_step3;
                    break;
                case sa_suspicion_exit:
                    next_state = ss_slow_finger_step2;
                    break;
                case sa_arm_alarm:
                case sa_arm_motion:
                    next_state = ss_slow_finger_setup;
                    break;
                case sa_shutdown:
                case sa_shutdown_done:
                    next_state = ss_before_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }
            break;
        }

        case ss_suspicion_step3:
        {
            switch ( action )
            {
                case sa_transition_next:
                case sa_arm_reset:
                case sa_suspicion_timeout:
                    next_state = ss_suspicion_step3;
                    break;
                case sa_suspicion_exit:
                    next_state = ss_slow_finger_step2;
                    break;
                case sa_arm_off:
                    next_state = ss_suspicion_setup;
                    break;
                case sa_arm_alarm:
                case sa_arm_motion:
                    next_state = ss_slow_finger_setup;
                    break;
                case sa_shutdown:
                case sa_shutdown_done:
                    next_state = ss_before_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }
            break;
        }

        case ss_slow_finger_setup:
        {
            switch ( action )
            {
                case sa_transition_next:
                case sa_arm_alarm:
                case sa_arm_motion:
                case sa_arm_off:
                    next_state = ss_slow_finger_setup;
                    break;
                case sa_slowfinger_timeout:
                    next_state = ss_slow_finger_step1;
                    break;
                case sa_arm_reset:
                    next_state = ss_slow_finger_step2;
                    break;
                case sa_shutdown:
                case sa_shutdown_done:
                    next_state = ss_before_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }
            break;
        }

        case ss_slow_finger_step1:
        {
            switch ( action )
            {
                case sa_transition_next:
                case sa_arm_alarm:
                case sa_arm_motion:
                case sa_arm_off:
                    next_state = ss_slow_finger_step1;
                    break;
                case sa_slowfinger_timeout:
                    next_state = ss_slow_finger_setup;
                    break;
                case sa_arm_reset:
                    next_state = ss_slow_finger_step2;
                    break;
                case sa_shutdown:
                case sa_shutdown_done:
                    next_state = ss_before_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }
            break;
        }

        case ss_slow_finger_step2:
        {
            switch ( action )
            {
                case sa_transition_next:
                case sa_arm_reset:
                case sa_slowfinger_timeout:
                    next_state = ss_slow_finger_step2;
                    break;
                case sa_arm_alarm:
                case sa_arm_motion:
                    next_state = ss_scare_setup;
                    break;
                case sa_arm_off:
                    next_state = ss_timeout_then_reset;
                    break;
                case sa_shutdown:
                case sa_shutdown_done:
                    next_state = ss_before_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }
            break;
        }

        case ss_before_shutdown:
        {
            switch ( action )
            {
                case sa_transition_next:
                case sa_arm_reset:
                case sa_arm_alarm:
                case sa_arm_motion:
                case sa_arm_off:
                case sa_shutdown:
                case sa_timeout:
                    next_state = ss_before_shutdown;
                    break;

                case sa_shutdown_done:
                    next_state = ss_shutdown;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }

            break;
        }

        case ss_shutdown:
        {
            switch ( action )
            {
                case sa_transition_next:
                case sa_arm_reset:
                case sa_arm_alarm:
                case sa_arm_motion:
                case sa_arm_off:
                case sa_shutdown:
                case sa_shutdown_done:
                case sa_timeout:
                    next_state = ss_shutdown;
                    break;

                default:
                    //unknown action (should never happen)
                    print_stderr( STDPRINT_NAME "unknown action specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
                    ret = EINVAL;

                    break;
            }

            break;
        }


        default:
        {
            //unknown current state (should never happen)
            print_stderr( STDPRINT_NAME "unknown current state specified; action=%s currentstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ) );
            ret = EINVAL;

            break;
        }
    }

    print_stdout( STDPRINT_NAME "state change details; action=%s currentstate=%s nextstate=%s\n", statemachine_get_actionname( action ), statemachine_get_statename( current_state ), statemachine_get_statename( next_state ) );

    //pass new state to caller
    if (pnew_state != NULL)
    {
        *pnew_state = next_state;
    }

    //notify all clients only if state changes occurred
    if ( current_state != next_state )
    {
        statemachine_set_state_change_nolock( next_state );
    }

    pthread_mutex_unlock( &statemachine_mutex );

    return ret;

}

int statemachine_wait_state_change( statemachine_cid* pcid, statemachine_states_t* pnew_state )
{
    //do following:
    //lock the machien
    //block until state changed
    //  we check if this specific clients last statechange was notified
    //  if not, we continue to block
    //read state for returning
    //unlock the machine

    pthread_mutex_lock( &statemachine_mutex );

    //block until state changed
    //we check if this clients last statechange was notified
    //if not, we continue to block
    while (!pcid->state_haschanged)
    {
        pthread_cond_wait( &signal_state_haschanged, &statemachine_mutex );
    }

    pcid->state_haschanged = false;

    //read state
    *pnew_state = statemachine_get_current_state_nolock();

    pthread_mutex_unlock( &statemachine_mutex );

    return EOK;
}

int statemachine_cancel_waitfor( statemachine_cid* pcid )
{
    return statemachine_set_state_change_forclient_nolock( pcid );
}

statemachine_states_t statemachine_get_current_state()
{
    statemachine_states_t ret_state;

    pthread_mutex_lock( &statemachine_mutex );

    ret_state = state;

    pthread_mutex_unlock( &statemachine_mutex );

    return ret_state;
}

const char* statemachine_get_statename( statemachine_states_t value )
{
    if ( value < ss_END )
    {
        return statemachine_state_names[ value ];
    }
    else
    {
        return "unknown";
    }
}

const char* statemachine_get_actionname( statemachine_actions_t value )
{
    if ( value < sa_END )
    {
        return statemachine_action_names[ value ];
    }
    else
    {
        return "unknown";
    }
}
