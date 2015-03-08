#include "util.h"
#include "statemachine.h"

#include <wiringPi.h>
#include <piFace.h>

#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <stdarg.h>
#include <errno.h>
#include <pthread.h>
#include <sys/queue.h>
#include <sys/types.h>
#include <signal.h>

#define STDPRINT_NAME           __FILE__ ":"
#define GPIO_BASE               0
#define FINGER_MTR_EN           (GPIO_BASE+0)
#define FINGER_MTR_IN1          (GPIO_BASE+1)
#define FINGER_MTR_IN2          (GPIO_BASE+2)
#define BOX_INT_SWITCH1         (GPIO_BASE+3)
#define BOX_EXT_SWITCH1         (GPIO_BASE+4)

#define ARM_MOVEMENT_FWD_OVERRUN_USEC       200000 //200msec
#define STATE_DEBOUNCE_USEC                 400000 //400msec
#define STATE_SCARE1_VIB_USEC               500000  //500msec
#define STATE_SCARE2_VIB_USEC               500000  //500msec
#define STATE_SCARE_EXIT_USEC               3000000 //3sec
#define STATE_TIMEOUT_RESET_USEC            10000000 //10sec
#define STATE_SUSPICION_EXIT_USEC           45000000 //45sec
#define STATE_SUSPICION_PEEK_MIN_USEC       1000000 //1sec
#define STATE_SUSPICION_PEEK_MAX_USEC       12000000 //8sec
#define STATE_SUSPICION_PEEK_OPEN_MIN_USEC  400000
#define STATE_SUSPICION_PEEK_OPEN_MAX_USEC  600000
#define STATE_SUSPICION_PEEK_LEN_MIN_USEC   1000000
#define STATE_SUSPICION_PEEK_LEN_MAX_USEC   3000000
#define STATE_SLOWFINGER_DUTYFULL_USEC      200000 //200ms
#define STATE_SLOWFINGER_DUTYON_USEC        100000
#define STATE_SLOWFINGER_DUTYOFF_USEC       (STATE_SLOWFINGER_DUTYFULL_USEC-STATE_SLOWFINGER_DUTYON_USEC)

typedef enum
{
    am_idle,
    am_fwd,
    am_bwd,
} arm_movement_state_t;

typedef struct
{
    int                     usec;
    statemachine_actions_t  action;
} state_timer_action_t;

typedef struct
{
    bool                int_switch1;
    bool                ext_switch1;
    pthread_mutex_t     mutex_swstates;
} box_swstates_t;

static pthread_mutex_t          mutex_exit;
static pthread_cond_t           signal_exit;
static volatile bool            flag_exit;
static box_swstates_t           box_swstates;
static arm_movement_state_t     arm_movement_state;

static int init_rand()
{
    srand(time(NULL));
    return EOK;
}

static int get_random_number(int min_num, int max_num)
{
    int result=0,low_num=0,hi_num=0;
    if(min_num<max_num)
    {
        low_num=min_num;
        hi_num=max_num+1; // this is done to include max_num in output.
    }else{
        low_num=max_num+1;// this is done to include max_num in output.
        hi_num=min_num;
    }

    result = (rand()%(hi_num-low_num))+low_num;
    return result;
}

// Define the function to be called when ctrl-c (SIGINT) signal is sent to process
static void signal_callback_handler(int signum)
{
   printf("Caught signal %d\n",signum);

   // signal cleanup and close up stuff here
   flag_exit = true;
   pthread_cond_broadcast(&signal_exit);
}

static void wait_for_exit()
{
    pthread_mutex_lock(&mutex_exit);

    //wait forever for exit signals
    while (!flag_exit)
    {
        pthread_cond_wait(&signal_exit, &mutex_exit);
    }

    pthread_mutex_unlock(&mutex_exit);
}

static int arm_movement_stop()
{
    print_stdout( STDPRINT_NAME "arm movement stop\n");
    digitalWrite(FINGER_MTR_EN, LOW);
    arm_movement_state = am_idle;

    return EOK;
}

static int arm_movement_forward()
{
    print_stdout( STDPRINT_NAME "arm movement forward\n");

    digitalWrite(FINGER_MTR_IN1, LOW);
    digitalWrite(FINGER_MTR_IN2, HIGH);

    digitalWrite(FINGER_MTR_EN, HIGH);
    arm_movement_state = am_fwd;

    return EOK;
}

static int arm_movement_backward()
{
    print_stdout( STDPRINT_NAME "arm movement backward\n");

    if (box_swstates.int_switch1 == false)
    {
        digitalWrite(FINGER_MTR_IN1, HIGH);
        digitalWrite(FINGER_MTR_IN2, LOW);

        digitalWrite(FINGER_MTR_EN, HIGH);
        arm_movement_state = am_bwd;
    }
    else
    {
        print_stdout( STDPRINT_NAME "arm movement skipped\n");
        if (box_swstates.ext_switch1 == true)
        {
            statemachine_next_state(sa_arm_alarm, NULL);
        }
        else
        {
            statemachine_next_state(sa_arm_off, NULL);
        }
    }

    return EOK;
}

static int init_pins()
{
    //init pins
    pinMode(FINGER_MTR_EN, OUTPUT);
    pinMode(FINGER_MTR_IN1, OUTPUT);
    pinMode(FINGER_MTR_IN2, OUTPUT);
    pinMode(BOX_EXT_SWITCH1, INPUT);
    pinMode(BOX_INT_SWITCH1, INPUT);

    return EOK;
}

static int init_gpio()
{
    wiringPiSetup();

    return EOK;
}

static int init_box_swstate( box_swstates_t* pbss )
{
    pbss->ext_switch1 = false;
    pbss->int_switch1 = false;
    pthread_mutex_init(&pbss->mutex_swstates, NULL);

    return EOK;
}

static void set_box_swstate( box_swstates_t* pbss )
{
    pthread_mutex_lock(&pbss->mutex_swstates);

    box_swstates_t bss_tmp;

    //forward arm movements need to run a little longer to ensure togglesw flops
    //fully (otherwise it sometimes sits exactly halfway)
    //we delay the sampling and state change for a small moment
    if (arm_movement_state == am_fwd)
    {
        usleep(ARM_MOVEMENT_FWD_OVERRUN_USEC);
    }


    //we don't bother debouncing as the statemachine is designed to be bounce tolerant
    //under test, seen the arm movement is cleaner and less prone of getting stuck when first
    //making contact with switches (as the debounce without arm movement stop causes overshoot and
    //the debounce with arm movement stop cause jerky undershoots... both resulting in undesired behavior
    bss_tmp.int_switch1 = digitalRead(BOX_INT_SWITCH1);
    bss_tmp.ext_switch1 = digitalRead(BOX_EXT_SWITCH1);

    //only issue state changes if we had changes!
    if ((bss_tmp.int_switch1 != pbss->int_switch1)
        || (bss_tmp.ext_switch1 != pbss->ext_switch1))
    {
        pbss->int_switch1 = bss_tmp.int_switch1;
        pbss->ext_switch1 = bss_tmp.ext_switch1;

        //apply actions to statemachine
        if ( (pbss->int_switch1 == false)
                && (pbss->ext_switch1 == false) )
        {
            statemachine_next_state(sa_arm_reset, NULL);
        }

        if ( (pbss->int_switch1 == true)
                && (pbss->ext_switch1 == true) )
        {
            statemachine_next_state(sa_arm_alarm, NULL);
        }

        if ( (pbss->int_switch1 == false)
                && (pbss->ext_switch1 == true) )
        {
            statemachine_next_state(sa_arm_motion, NULL);
        }

        if ( (pbss->int_switch1 == true)
                && (pbss->ext_switch1 == false) )
        {
            statemachine_next_state(sa_arm_off, NULL);
        }
    }

    pthread_mutex_unlock(&pbss->mutex_swstates);

}

static void callback_box_ext_switch1()
{
    print_stdout( STDPRINT_NAME "box EXT switch interrupt!!\n");

    set_box_swstate(&box_swstates);
}

static void callback_box_int_switch1()
{
    print_stdout( STDPRINT_NAME "box INT switch interrupt!!\n");

    set_box_swstate(&box_swstates);
}

static int install_pin_isr()
{
    wiringPiISR(BOX_EXT_SWITCH1, INT_EDGE_BOTH, callback_box_ext_switch1);
    wiringPiISR(BOX_INT_SWITCH1, INT_EDGE_BOTH, callback_box_int_switch1);

    return EOK;
}

static void* timer_action_entry(void* parg)
{
    state_timer_action_t* timer_action = (state_timer_action_t*)parg;
    usleep(timer_action->usec);

    statemachine_next_state(timer_action->action,NULL);
    free(parg);

    return NULL;
}

static int setup_timer_action(int usec, statemachine_actions_t action)
{
    //setup timer; on expiry action is set

    int ret = EOK;
    state_timer_action_t* parg = malloc(sizeof(state_timer_action_t));

    if (parg != NULL)
    {
        parg->usec = usec;
        parg->action = action;

        print_stdout( STDPRINT_NAME "setting up timer for %2.3f secs\n", ((float)usec / 1000000));

        pthread_t pid;
        pthread_attr_t attr;
        pthread_attr_init( &attr );
        pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_DETACHED );
        pthread_create( &pid, &attr, timer_action_entry, parg );

        pthread_attr_destroy( &attr );
        return EOK;
    }
    else
    {
        ret = errno;
    }

    return ret;
}

void* statemachine_thread_entry(void* args)
{

    __unused(args);

    statemachine_states_t current_state = ss_powerup;
    bool finished = false;
    statemachine_cid ss_cid;

    statemachine_init(&ss_cid);


    while ( !finished )
    {
        statemachine_wait_state_change( &ss_cid, &current_state);

        print_stdout( STDPRINT_NAME "wakingup to handle state change; currentstate=%s \n", statemachine_get_statename( current_state ) );
        switch (current_state)
        {
            case ss_idle:
            {
                arm_movement_stop();
                break;
            }

            case ss_powerup:
            {
                set_box_swstate(&box_swstates);
                break;
            }

            case ss_alarming:
            {
                arm_movement_forward();
                break;
            }

            case ss_reseting:
            {
                arm_movement_backward();
                break;
            }

            case ss_scare_setup:
            {
                setup_timer_action(STATE_SCARE_EXIT_USEC, sa_scare_exit);

                if (box_swstates.int_switch1 == false)
                {
                    arm_movement_backward();
                }
                else
                {
                    arm_movement_forward();
                }

                break;
            }

            case ss_scare_step1:
            {
                setup_timer_action(STATE_SCARE1_VIB_USEC, sa_scare_timeout);
                arm_movement_forward();
                break;
            }

            case ss_scare_step2:
            {
                setup_timer_action(STATE_SCARE2_VIB_USEC, sa_scare_timeout);
                arm_movement_backward();
                break;
            }

            case ss_scare_step3:
            {
                arm_movement_forward();
                break;
            }

            case ss_timeout_then_reset:
            {
                setup_timer_action(STATE_TIMEOUT_RESET_USEC, sa_timeout);
                arm_movement_stop();
                break;
            }

            case ss_reseting_retry:
            {
                arm_movement_backward();
                break;
            }

            case ss_offence:
            {
                arm_movement_forward();
                break;
            }

            case ss_suspicion_setup:
            {
                setup_timer_action(STATE_SUSPICION_EXIT_USEC, sa_suspicion_exit);
                setup_timer_action(
                    get_random_number(
                        STATE_SUSPICION_PEEK_MIN_USEC,
                        STATE_SUSPICION_PEEK_MAX_USEC),
                    sa_suspicion_timeout);

                arm_movement_stop();
                break;
            }

            case ss_suspicion_step1:
            {
                setup_timer_action(
                    get_random_number(
                        STATE_SUSPICION_PEEK_OPEN_MIN_USEC,
                        STATE_SUSPICION_PEEK_OPEN_MAX_USEC),
                    sa_suspicion_timeout);

                    arm_movement_forward();
                break;
            }

            case ss_suspicion_step2:
            {
                setup_timer_action(
                    get_random_number(
                        STATE_SUSPICION_PEEK_LEN_MIN_USEC,
                        STATE_SUSPICION_PEEK_LEN_MAX_USEC),
                    sa_suspicion_timeout);

                    arm_movement_stop();
                break;
            }

            case ss_suspicion_step3:
            {
                arm_movement_backward();
                break;
            }

            case ss_slow_finger_setup:
            {
                setup_timer_action(STATE_SLOWFINGER_DUTYON_USEC, sa_slowfinger_timeout);
                arm_movement_forward();
                break;
            }

            case ss_slow_finger_step1:
            {
                setup_timer_action(STATE_SLOWFINGER_DUTYOFF_USEC, sa_slowfinger_timeout);
                arm_movement_stop();
                break;
            }

            case ss_slow_finger_step2:
            {
                arm_movement_backward();
                break;
            }

            case ss_before_shutdown:
            {
                arm_movement_stop();
                statemachine_next_state(sa_shutdown_done, &current_state);
                break;
            }

            case ss_shutdown:
            {
                finished = true;
                break;
            }

            default:
            {
                //unknown state (should never happen)
                print_stderr( STDPRINT_NAME "unknown state entered; currentstate=%d\n", current_state );

                break;
            }
        }
    }


    return NULL;
}

int wait_for_shutdown(statemachine_cid* pss_cid)
{
    statemachine_states_t current_state = ss_powerup;

    while ( current_state != ss_shutdown )
    {
        statemachine_wait_state_change( pss_cid, &current_state);
    }

    return EOK;
}

int main(int c, char **v)
{
    pthread_t pid;
    statemachine_cid ss_main_cid;

    flag_exit = false;
    // Register signal and signal handler
    signal(SIGINT, signal_callback_handler);
    signal(SIGHUP, signal_callback_handler);

    init_rand();
    util_init();
    set_verbose_lvl(verblvl_moremore);

    statemachine_init(&ss_main_cid);
    init_box_swstate(&box_swstates);
    init_gpio();
    init_pins();
    install_pin_isr();

    arm_movement_stop();

    //kick off statemachine monitoring thread
    pthread_create( &pid, NULL, statemachine_thread_entry, NULL );

    //all actions are conducted async to main thread
    //we just wait here until signaled to term
    wait_for_exit();

    //kick off shutdown cleanup
    statemachine_next_state(sa_shutdown, NULL);

    //wait here until shutdown cleanup complete
    wait_for_shutdown(&ss_main_cid);

    util_fini();

    printf("clean exit!\n");

    return 0;
}
